#!/usr/bin/env python
"""
guidar_services.py  –  ROS 1 service node for GuidAR

Hosts three services consumed by the Unity AR client:
  /server/generateTrajectory   – diffusion costmap → A* path → JointTrajectory
  /server/modifyTrajectory     – accept edited B-spline control points
  /executeTrajectory           – flip the MPCC execution flag

Subscribes to Vicon for live obstacle poses (bomb_1, chair_1, table_1).
"""

import sys
import os
import threading
import numpy as np
import torch
import torch.nn.functional as F

import rospy
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

from skimage.graph import route_through_array
from scipy.interpolate import BSpline

# ---------------------------------------------------------------------------
# Adjust this path so Python can find the guidAR-Diffusion package
# ---------------------------------------------------------------------------
DIFFUSION_ROOT = os.path.expanduser("~/zach_ws/src/guidAR-Diffusion")
sys.path.insert(0, DIFFUSION_ROOT)

from train import ExpertEnsemble
from MoE.ddpm import DDPM
from DataGenerator.sim import Costmap
from utils.spline import generate_clamped_spline, clamped_knots

# ===================================================================
#  CONFIG
# ===================================================================
OBSTACLE_CLASSES = ["chair", "table", "bomb"]
VICON_TOPICS = {
    "bomb":  "/vicon/bomb_1/bomb_1",
    "chair": "/vicon/chair_1/chair_1",
    "table": "/vicon/table_1/table_1",
}
ROBOT_TOPIC = "/vicon/Rosbot_AR_2/Rosbot_AR_2"
CHECKPOINT  = os.path.join(DIFFUSION_ROOT, "checkpoints/checkpoint_epoch8.pt")

H, W = 128, 128                # costmap resolution
METERS_PER_PIXEL = 0.05        # map scale  (tune to your Vicon workspace)
MAP_ORIGIN_X = -3.2            # world-frame x of pixel (0,0)
MAP_ORIGIN_Y = -3.2            # world-frame y of pixel (0,0)

NUM_CTRL_PTS = 10
SPLINE_DEGREE = 3
DDPM_TIMESTEPS = 1000


# ===================================================================
#  Coordinate helpers
# ===================================================================
def world_to_pixel(x_w, y_w):
    """World (metres, Vicon frame) → pixel (row, col) in the costmap."""
    col = int(np.clip((x_w - MAP_ORIGIN_X) / METERS_PER_PIXEL, 0, W - 1))
    row = int(np.clip((y_w - MAP_ORIGIN_Y) / METERS_PER_PIXEL, 0, H - 1))
    return row, col


def pixel_to_world(row, col):
    """Pixel (row, col) → world metres."""
    x_w = col * METERS_PER_PIXEL + MAP_ORIGIN_X
    y_w = row * METERS_PER_PIXEL + MAP_ORIGIN_Y
    return x_w, y_w


# ===================================================================
#  GuidAR Service Node
# ===================================================================
class GuidARServices:
    def __init__(self):
        rospy.init_node("server", anonymous=False)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"[GuidAR] Using device: {self.device}")

        # ----- live Vicon state -----
        self.lock = threading.Lock()
        self.obstacle_poses = {cls: None for cls in OBSTACLE_CLASSES}
        self.robot_pose = None

        for cls, topic in VICON_TOPICS.items():
            rospy.Subscriber(topic, TransformStamped, self._vicon_cb, callback_args=cls)
        rospy.Subscriber(ROBOT_TOPIC, TransformStamped, self._robot_cb)

        # ----- diffusion model -----
        n_classes = len(OBSTACLE_CLASSES)
        conditioning_channels = 4 + 4 * (n_classes - 1) + 1
        self.model = ExpertEnsemble(OBSTACLE_CLASSES, conditioning_channels).to(self.device)

        ckpt = torch.load(CHECKPOINT, map_location=self.device)
        self.model.load_state_dict(ckpt["model_state_dict"])
        self.model.eval()
        rospy.loginfo("[GuidAR] Diffusion model loaded.")

        self.ddpm = DDPM(timesteps=DDPM_TIMESTEPS, device=self.device)

        # ----- trajectory state (shared with MPCC via /reference_trajectory) -----
        self.current_knots = None
        self.current_ctrl_x = None
        self.current_ctrl_y = None
        self.is_executing = False

        # ----- publishers -----
        self.traj_pub = rospy.Publisher(
            "/reference_trajectory", JointTrajectory, queue_size=1, latch=True
        )

        # ----- service servers -----
        # generateTrajectory uses custom messages compiled from your GuidAR package.
        # Import them here so the node fails fast if they're missing.
        from GuidAR.srv import (              # <-- adjust package name
            generateTraj, generateTrajResponse,
            modifyTrajectory, modifyTrajectoryResponse,
        )
        self.gen_srv = rospy.Service(
            "/server/generateTrajectory", generateTraj, self._handle_generate
        )
        self.mod_srv = rospy.Service(
            "/server/modifyTrajectory", modifyTrajectory, self._handle_modify
        )
        self.exec_srv = rospy.Service(
            "/executeTrajectory", Empty, self._handle_execute
        )

        rospy.loginfo("[GuidAR] All services advertised.")

    # ------------------------------------------------------------------
    #  Vicon callbacks
    # ------------------------------------------------------------------
    def _vicon_cb(self, msg, cls):
        t = msg.transform.translation
        with self.lock:
            self.obstacle_poses[cls] = (t.x, t.y)

    def _robot_cb(self, msg):
        t = msg.transform.translation
        with self.lock:
            self.robot_pose = (t.x, t.y)

    # ------------------------------------------------------------------
    #  Build conditioning tensors from live Vicon data
    # ------------------------------------------------------------------
    def _build_batch_from_vicon(self, goal_world):
        """
        Mimics CostmapDataset.__getitem__ but uses live obstacle positions.
        Returns the same (features, targets, positions, radii, goal, angles)
        tuple the model expects — with batch dim 1.
        """
        with self.lock:
            poses_snapshot = dict(self.obstacle_poses)

        # Build obstacles_by_class in the same format as the dataset
        obstacles_by_class = {}
        for cls in OBSTACLE_CLASSES:
            if poses_snapshot[cls] is not None:
                r, c = world_to_pixel(*poses_snapshot[cls])
                obstacles_by_class[cls] = [{
                    "pos": np.array([r, c]),
                    "rad": 2,           # default radius; adjust per-object if needed
                    "angle": 0.0,
                }]
            else:
                obstacles_by_class[cls] = []

        # Goal
        goal_r, goal_c = world_to_pixel(*goal_world)
        goal = np.array([goal_r, goal_c])

        # --- Costmap conditioning (mirrors CostmapDataset) ---
        cm = Costmap(H=H, W=W)
        cm.goal = goal
        costmaps, radii_maps, binary_occ, sin_maps, cos_maps = cm.calculateCost(
            obstacles_by_class
        )

        goal_map = np.zeros((H, W), dtype=np.float32)
        goal_map[goal[0], goal[1]] = 1.0
        goal_t = torch.from_numpy(goal_map).float().unsqueeze(0)

        keys = list(costmaps.keys())
        keys_to_i = {k: i for i, k in enumerate(keys)}

        bin_stack = torch.stack([torch.from_numpy(binary_occ[k]).float() for k in keys])
        rad_stack = torch.stack([torch.from_numpy(radii_maps[k]).float() for k in keys])
        sin_stack = torch.stack([torch.from_numpy(sin_maps[k]).float() for k in keys])
        cos_stack = torch.stack([torch.from_numpy(cos_maps[k]).float() for k in keys])

        features, targets = {}, {}
        for key in keys:
            i = keys_to_i[key]
            curr_bin = bin_stack[i : i + 1]
            curr_rad = rad_stack[i : i + 1]
            curr_sin = sin_stack[i : i + 1]
            curr_cos = cos_stack[i : i + 1]

            other_bin = torch.cat([bin_stack[:i], bin_stack[i + 1 :]])
            other_rad = torch.cat([rad_stack[:i], rad_stack[i + 1 :]])
            other_sin = torch.cat([sin_stack[:i], sin_stack[i + 1 :]])
            other_cos = torch.cat([cos_stack[:i], cos_stack[i + 1 :]])

            x = torch.cat(
                [curr_bin, curr_rad, curr_sin, curr_cos,
                 other_bin, other_rad, other_sin, other_cos, goal_t],
                dim=0,
            )
            features[key] = x.unsqueeze(0)                # add batch dim
            targets[key] = (
                torch.from_numpy(costmaps[key].copy()).float().unsqueeze(0).unsqueeze(0)
            )

        # positions / radii dicts (list-of-dicts with batch dim)
        positions = {}
        radii_dict = {}
        for cls, obs_list in obstacles_by_class.items():
            positions[cls] = [tuple(o["pos"]) for o in obs_list]
            radii_dict[cls] = [o["rad"] for o in obs_list]

        angles = {cls: torch.tensor([o["angle"] for o in obstacles_by_class[cls]])
                  for cls in OBSTACLE_CLASSES}

        return features, targets, [positions], [radii_dict], goal, angles

    # ------------------------------------------------------------------
    #  Diffusion inference  →  fused costmap  →  A* path
    # ------------------------------------------------------------------
    @torch.no_grad()
    def _generate_costmap_and_path(self, goal_world):
        features, targets, positions, radii, goal, angles = self._build_batch_from_vicon(
            goal_world
        )

        # --- per-expert DDPM sampling ---
        diffused = {}
        for cls in OBSTACLE_CLASSES:
            cond = features[cls].to(self.device)
            gt = targets[cls].to(self.device)
            generated = self.ddpm.sample(self.model.experts[cls], cond, shape=gt.shape)
            diffused[cls] = [generated]

        # --- logsumexp fusion ---
        stacked = torch.stack([v[0].squeeze(1) for v in diffused.values()], dim=1)
        fused = torch.logsumexp(stacked, dim=1).unsqueeze(1)
        fused_np = fused[0, 0].cpu().numpy()

        # --- A* through fused costmap ---
        mn, mx = fused_np.min(), fused_np.max()
        normed = (fused_np - mn) / (mx - mn + 1e-8) + 1e-8

        goal_r, goal_c = int(np.clip(goal[0], 0, H - 1)), int(np.clip(goal[1], 0, W - 1))
        path_result = route_through_array(
            normed, [0, 0], [goal_r, goal_c], fully_connected=True, geometric=True
        )
        path_arr = np.array(path_result[0])
        x_px = path_arr[:, 1]  # col
        y_px = path_arr[:, 0]  # row

        # --- fit B-spline ---
        x_s, y_s, P, U = generate_clamped_spline(x_px, y_px, SPLINE_DEGREE, NUM_CTRL_PTS)

        # convert spline samples to world coords
        world_pts = [pixel_to_world(y_s[i], x_s[i]) for i in range(len(x_s))]
        ctrl_world = [pixel_to_world(P[i, 1], P[i, 0]) for i in range(len(P))]

        return world_pts, ctrl_world, P, U

    # ------------------------------------------------------------------
    #  /server/generateTrajectory
    # ------------------------------------------------------------------
    def _handle_generate(self, req):
        from GuidAR.srv import generateTrajResponse

        goal_x = req.goal.x
        goal_y = req.goal.y
        rospy.loginfo(f"[GuidAR] generateTrajectory goal=({goal_x:.2f}, {goal_y:.2f})")

        world_pts, ctrl_world, P, U = self._generate_costmap_and_path((goal_x, goal_y))

        # Build JointTrajectory  (same format mpcc_ros trajectorycb expects)
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "vicon/world"

        # Parameterise by cumulative arc-length
        arc = [0.0]
        for i in range(1, len(world_pts)):
            dx = world_pts[i][0] - world_pts[i - 1][0]
            dy = world_pts[i][1] - world_pts[i - 1][1]
            arc.append(arc[-1] + np.hypot(dx, dy))

        # Downsample to ~200 points to keep the message lean
        total_len = arc[-1]
        n_out = min(len(world_pts), 200)
        indices = np.linspace(0, len(world_pts) - 1, n_out, dtype=int)

        for idx in indices:
            pt = JointTrajectoryPoint()
            pt.positions = [world_pts[idx][0], world_pts[idx][1]]
            pt.time_from_start = rospy.Duration.from_sec(arc[idx])
            traj.points.append(pt)

        # Cache knots/ctrl pts for later modify calls
        self.current_knots = U.tolist()
        self.current_ctrl_x = [c[0] for c in ctrl_world]
        self.current_ctrl_y = [c[1] for c in ctrl_world]

        # Also publish on /reference_trajectory so MPCC picks it up
        self.traj_pub.publish(traj)

        resp = generateTrajResponse()
        resp.trajectory = traj
        rospy.loginfo(f"[GuidAR] Sent trajectory with {len(traj.points)} pts, "
                      f"length={total_len:.2f} m")
        return resp

    # ------------------------------------------------------------------
    #  /server/modifyTrajectory
    # ------------------------------------------------------------------
    def _handle_modify(self, req):
        from GuidAR.srv import modifyTrajectoryResponse

        knots = np.array(req.knots)
        ctrl_x = np.array(req.ctrl_pts_x)
        ctrl_y = np.array(req.ctrl_pts_y)
        n = len(ctrl_x)

        rospy.loginfo(f"[GuidAR] modifyTrajectory: {n} ctrl pts, {len(knots)} knots")

        # Refit B-spline from Unity's edited control points
        k = SPLINE_DEGREE
        t0, t1 = knots[k], knots[-k - 1]
        t_eval = np.linspace(t0, t1, 500)

        splx = BSpline(knots, ctrl_x, k)
        sply = BSpline(knots, ctrl_y, k)
        xs = splx(t_eval)
        ys = sply(t_eval)

        # Build JointTrajectory with arc-length parameterisation
        arc = [0.0]
        for i in range(1, len(xs)):
            arc.append(arc[-1] + np.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "vicon/world"

        for i in range(len(xs)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(xs[i]), float(ys[i])]
            pt.time_from_start = rospy.Duration.from_sec(arc[i])
            traj.points.append(pt)

        # Cache & publish
        self.current_knots = knots.tolist()
        self.current_ctrl_x = ctrl_x.tolist()
        self.current_ctrl_y = ctrl_y.tolist()

        self.traj_pub.publish(traj)
        rospy.loginfo(f"[GuidAR] Published modified trajectory, length={arc[-1]:.2f} m")

        return modifyTrajectoryResponse()

    # ------------------------------------------------------------------
    #  /executeTrajectory
    # ------------------------------------------------------------------
    def _handle_execute(self, req):
        self.is_executing = True
        rospy.loginfo("[GuidAR] Execution enabled.")
        return EmptyResponse()

    # ------------------------------------------------------------------
    def spin(self):
        rospy.loginfo("[GuidAR] Node ready — waiting for service calls.")
        rospy.spin()


# ===================================================================
if __name__ == "__main__":
    try:
        node = GuidARServices()
        node.spin()
    except rospy.ROSInterruptException:
        pass
