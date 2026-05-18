/**
 * trajectory_modifier.cpp
 *
 * Fast C++ node for trajectory modification and execution.
 * Hosts:
 *   /server/modifyTrajectory  – B-spline eval + smooth + publish
 *   /executeTrajectory        – toggle MPCC execution flag
 *
 * The diffusion-based /server/generateTrajectory stays in Python.
 * Both nodes publish to /server/TransferTrajectory.
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <GuidAR/modifyTrajectory.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace {

// =====================================================================
//  B-Spline helpers
// =====================================================================

/// Cox-de Boor basis function (recursive, degree p)
double basisFunction(int i, int p, double t, const std::vector<double>& knots)
{
    if (p == 0)
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;

    double left = 0.0, right = 0.0;
    double denom_l = knots[i + p] - knots[i];
    double denom_r = knots[i + p + 1] - knots[i + 1];

    if (denom_l > 1e-14)
        left = (t - knots[i]) / denom_l * basisFunction(i, p - 1, t, knots);
    if (denom_r > 1e-14)
        right = (knots[i + p + 1] - t) / denom_r * basisFunction(i + 1, p - 1, t, knots);

    return left + right;
}

/// Evaluate a 1-D B-spline at a set of parameter values
std::vector<double> evalBSpline(const std::vector<double>& knots,
                                const std::vector<double>& ctrl,
                                int degree,
                                const std::vector<double>& t_eval)
{
    const int n_ctrl = static_cast<int>(ctrl.size());
    std::vector<double> out(t_eval.size(), 0.0);

    for (size_t k = 0; k < t_eval.size(); ++k) {
        double t = t_eval[k];
        // Clamp to valid range to handle endpoint
        if (t >= knots.back())
            t = knots.back() - 1e-10;

        double val = 0.0;
        for (int i = 0; i < n_ctrl; ++i)
            val += ctrl[i] * basisFunction(i, degree, t, knots);
        out[k] = val;
    }
    return out;
}

// =====================================================================
//  1-D Gaussian smoothing (simple FIR approximation)
// =====================================================================
std::vector<double> gaussianSmooth(const std::vector<double>& data, double sigma)
{
    // Build a discrete Gaussian kernel (truncated at 3*sigma)
    int radius = static_cast<int>(std::ceil(3.0 * sigma));
    if (radius < 1) radius = 1;
    int ksize = 2 * radius + 1;
    std::vector<double> kernel(ksize);
    double sum = 0.0;
    for (int i = 0; i < ksize; ++i) {
        double x = i - radius;
        kernel[i] = std::exp(-0.5 * x * x / (sigma * sigma));
        sum += kernel[i];
    }
    for (auto& v : kernel) v /= sum;

    // Convolve with reflect-padding
    const int n = static_cast<int>(data.size());
    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) {
        double acc = 0.0;
        for (int j = 0; j < ksize; ++j) {
            int idx = i + j - radius;
            // Reflect at boundaries
            if (idx < 0) idx = -idx;
            if (idx >= n) idx = 2 * n - 2 - idx;
            
	    if (idx < 0) idx = 0;
	    if (idx >= n) idx = n - 1;
	    
	    acc += data[idx] * kernel[j];
        }
        out[i] = acc;
    }
    return out;
}

// =====================================================================
//  Linspace helper
// =====================================================================
std::vector<double> linspace(double a, double b, int n)
{
    std::vector<double> v(n);
    if (n == 1) { v[0] = a; return v; }
    double step = (b - a) / (n - 1);
    for (int i = 0; i < n; ++i)
        v[i] = a + i * step;
    return v;
}

} // anonymous namespace


// =====================================================================
//  Node class
// =====================================================================
class TrajectoryModifier {
public:
    TrajectoryModifier(ros::NodeHandle& nh)
        : nh_(nh)
        , spline_degree_(3)
        , smooth_sigma_(3.0)
        , n_eval_(200)
        , n_subsample_(50)
    {
        // Publishers
        traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
            "/server/TransferTrajectory", 1, true /*latch*/);

        exec_pub_ = nh_.advertise<std_msgs::Bool>(
            "/guidar/execute", 1, true /*latch*/);

        // Services
        modify_srv_ = nh_.advertiseService(
            "/server/modifyTrajectory",
            &TrajectoryModifier::handleModify, this);

        exec_srv_ = nh_.advertiseService(
            "/executeTrajectory",
            &TrajectoryModifier::handleExecute, this);

        ROS_INFO("[TrajectoryModifier] Services ready.");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Publisher exec_pub_;
    ros::ServiceServer modify_srv_;
    ros::ServiceServer exec_srv_;

    int spline_degree_;
    double smooth_sigma_;
    int n_eval_;
    int n_subsample_;

    // ------------------------------------------------------------------
    bool handleModify(GuidAR::modifyTrajectory::Request& req,
                      GuidAR::modifyTrajectory::Response& /*res*/)
    {
        ros::WallTime t0 = ros::WallTime::now();

        const auto& knots   = req.knots;
        const auto& ctrl_x  = req.ctrl_pts_x;
        const auto& ctrl_y  = req.ctrl_pts_y;
        const int k = spline_degree_;

        if (knots.size() < 2 || ctrl_x.empty() || ctrl_y.empty()) {
            ROS_WARN("[TrajectoryModifier] Empty modify request.");
            return true;
        }

        // Evaluation range: [knots[k], knots[n-k-1])
        double t0_spline = knots[k];
        double t1_spline = knots[knots.size() - k - 1];
        auto t_eval = linspace(t0_spline, t1_spline, n_eval_);

        // Evaluate B-spline
        std::vector<double> kv(knots.begin(), knots.end());
        auto xs = evalBSpline(kv, std::vector<double>(ctrl_x.begin(), ctrl_x.end()), k, t_eval);
        auto ys = evalBSpline(kv, std::vector<double>(ctrl_y.begin(), ctrl_y.end()), k, t_eval);

        // Gaussian smooth
        xs = gaussianSmooth(xs, smooth_sigma_);
        ys = gaussianSmooth(ys, smooth_sigma_);

        // Pin endpoints
        xs.front() = ctrl_x.front();  xs.back() = ctrl_x.back();
        ys.front() = ctrl_y.front();  ys.back() = ctrl_y.back();

        // Subsample
        auto idx = linspace(0, static_cast<double>(xs.size() - 1), n_subsample_);
        std::vector<double> sx(n_subsample_), sy(n_subsample_);
        for (int i = 0; i < n_subsample_; ++i) {
            int ii = static_cast<int>(std::round(idx[i]));
            sx[i] = xs[ii];
            sy[i] = ys[ii];
        }

        // Arc-length parameterisation
        std::vector<double> arc(n_subsample_, 0.0);
        for (int i = 1; i < n_subsample_; ++i) {
            double dx = sx[i] - sx[i - 1];
            double dy = sy[i] - sy[i - 1];
            arc[i] = arc[i - 1] + std::hypot(dx, dy);
        }

	// ---- Ensure minimum length for MPCC ----
	const double MIN_LENGTH = 4.5;  // comfortably above MPCC's 4.0m requirement
	if (arc.back() < MIN_LENGTH) {
	    // Extend along the direction of the last two points
	    double dx = sx.back() - sx[sx.size() - 2];
	    double dy = sy.back() - sy[sy.size() - 2];
	    double seg = std::hypot(dx, dy);
	    if (seg > 1e-8) {
		dx /= seg;
		dy /= seg;
	    } else {
		dx = 1.0; dy = 0.0;
	    }

	    while (arc.back() < MIN_LENGTH) {
		double step = 0.05;  // 5cm increments
		double new_x = sx.back() + dx * step;
		double new_y = sy.back() + dy * step;
		arc.push_back(arc.back() + step);
		sx.push_back(new_x);
		sy.push_back(new_y);
	    }
	    ROS_INFO("[TrajectoryModifier] Extended trajectory from %.2f to %.2f m",
		     arc[n_subsample_ - 1], arc.back());
	}

        // Build JointTrajectory
        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now();
        traj.header.frame_id = "vicon/world";
        traj.points.reserve(n_subsample_);

        for (int i = 0; i < n_subsample_; ++i) {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions = {sx[i], sy[i]};
            pt.time_from_start = ros::Duration(arc[i]);
            traj.points.push_back(pt);
        }

        traj_pub_.publish(traj);

        double elapsed_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
        ROS_INFO("[TrajectoryModifier] Modified trajectory: %.2f m, %d pts (%.1f ms)",
                 arc.back(), n_subsample_, elapsed_ms);
        return true;
    }

    // ------------------------------------------------------------------
    bool handleExecute(std_srvs::Empty::Request& /*req*/,
                       std_srvs::Empty::Response& /*res*/)
    {
        std_msgs::Bool msg;
        msg.data = true;
        exec_pub_.publish(msg);
        ROS_INFO("[TrajectoryModifier] Execution enabled.");
        return true;
    }
};


// =====================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_modifier");
    ros::NodeHandle nh;
    TrajectoryModifier node(nh);
    ros::spin();
    return 0;
}
