#!/usr/bin/env python
"""
modify_stress_test.py — Sends modifyTrajectory service calls at 30 Hz
to benchmark the C++ trajectory_modifier node.
"""
import rospy
import math
from GuidAR.srv import modifyTrajectory, modifyTrajectoryRequest

KNOTS = [0.0, 0.0, 0.0, 0.0,
         0.142857, 0.285714, 0.428571, 0.571429, 0.714286, 0.857143,
         1.0, 1.0, 1.0, 1.0]

BASE_X = [0.0, 0.3, 0.8, 1.5, 2.0, 1.8, 1.2, 0.5, -0.2, -0.5]
BASE_Y = [0.0, 0.5, 1.2, 1.0, 0.3, -0.5, -1.0, -0.8, -0.3, 0.2]


def main():
    rospy.init_node("modify_stress_test", anonymous=True)
    rospy.loginfo("Waiting for /server/modifyTrajectory ...")
    rospy.wait_for_service("/server/modifyTrajectory")
    proxy = rospy.ServiceProxy("/server/modifyTrajectory", modifyTrajectory)

    rate = rospy.Rate(30)
    count = 0

    rospy.loginfo("Sending at 30 Hz — Ctrl+C to stop")

    while not rospy.is_shutdown():
        # Slowly animate the control points so each call is unique
        t = count * 0.05
        ctrl_x = [x + 0.3 * math.sin(t + i * 0.7) for i, x in enumerate(BASE_X)]
        ctrl_y = [y + 0.3 * math.cos(t + i * 0.5) for i, y in enumerate(BASE_Y)]

        req = modifyTrajectoryRequest()
        req.knots = KNOTS
        req.ctrl_pts_x = ctrl_x
        req.ctrl_pts_y = ctrl_y

        try:
            proxy(req)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")

        count += 1
        if count % 30 == 0:
            rospy.loginfo(f"Sent {count} calls")

        rate.sleep()


if __name__ == "__main__":
    main()
