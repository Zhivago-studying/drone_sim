#include <ros/ros.h>

#include <data_process/CameraAngle.h>
#include <gazebo_msgs/ModelStates.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrapToPi(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

std::string cleanNamespace(const std::string &ns)
{
    if (ns.empty() || ns == "/")
        return "";
    return ns.front() == '/' ? ns.substr(1) : ns;
}

}  // namespace

class AngleErrorCal
{
public:
    AngleErrorCal()
        : pnh_("~")
    {
        self_name_ = cleanNamespace(ros::this_node::getNamespace());
        pnh_.param<std::string>("self_name", self_name_, self_name_);
        target_name_ = defaultTargetName(self_name_);
        pnh_.param<std::string>("target_name", target_name_, target_name_);
        pnh_.param<std::string>("csv_dir", csv_dir_, defaultCsvDir());
        pnh_.param<std::string>("csv_file", csv_file_, self_name_ + "_angle_error.csv");

        pnh_.param("min_range", min_range_, 0.05);
        pnh_.param("max_match_error", max_match_error_, 1.0);

        if (!isSupportedPair(self_name_, target_name_))
        {
            ROS_WARN("[ANGLE ERROR] this evaluator is intended for iris_0 <-> iris_1 only; got self='%s', target='%s'",
                     self_name_.c_str(), target_name_.c_str());
        }

        boost::filesystem::create_directories(csv_dir_);
        const boost::filesystem::path csv_path = boost::filesystem::path(csv_dir_) / csv_file_;
        csv_.open(csv_path.string().c_str(), std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
        {
            ROS_FATAL("[ANGLE ERROR] failed to open csv file: %s", csv_path.string().c_str());
            ros::shutdown();
            return;
        }

        csv_ << "stamp,self,det_index,matched_id,alpha_meas,theta_meas,"
             << "alpha_gt,theta_gt,alpha_error,theta_error,error,within_threshold\n";
        csv_.flush();

        model_sub_ = nh_.subscribe("/gazebo/model_states", 20, &AngleErrorCal::modelCallback, this);
        angle_sub_ = nh_.subscribe("camera_angle", 20, &AngleErrorCal::angleCallback, this);

        ROS_INFO("[ANGLE ERROR] self=%s target=%s csv=%s max_match_error=%.3f rad",
                 self_name_.c_str(), target_name_.c_str(), csv_path.string().c_str(), max_match_error_);
    }

    ~AngleErrorCal()
    {
        shutdownAndReport();
    }

    void shutdownAndReport()
    {
        if (reported_)
            return;

        if (csv_.is_open())
            csv_.flush();

        reportSummary();

        if (csv_.is_open())
            csv_.close();

        reported_ = true;
    }

private:
    struct Vec3
    {
        Vec3() = default;
        Vec3(double x_in, double y_in, double z_in)
            : x(x_in), y(y_in), z(z_in)
        {
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct GtAngle
    {
        std::string target_name;
        double alpha = 0.0;
        double theta = 0.0;
        bool valid = false;
    };

    std::string defaultCsvDir() const
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
            return std::string(home) + "/swarm_localization/src/test/logs";
        return "/tmp/angle_error_cal";
    }

    std::string defaultTargetName(const std::string &self_name) const
    {
        if (self_name == "iris_0")
            return "iris_1";
        if (self_name == "iris_1")
            return "iris_0";
        return "iris_1";
    }

    bool isSupportedPair(const std::string &self_name, const std::string &target_name) const
    {
        return (self_name == "iris_0" && target_name == "iris_1") ||
               (self_name == "iris_1" && target_name == "iris_0");
    }

    int findModelIndex(const gazebo_msgs::ModelStates::ConstPtr &msg, const std::string &name) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == name)
                return static_cast<int>(i);
        }
        return -1;
    }

    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        const int self_model_idx = findModelIndex(msg, self_name_);
        if (self_model_idx < 0)
        {
            ROS_WARN_THROTTLE(10.0, "[ANGLE ERROR] model_states has no self model '%s'", self_name_.c_str());
            has_gt_ = false;
            return;
        }

        const int target_model_idx = findModelIndex(msg, target_name_);
        if (target_model_idx < 0)
        {
            ROS_WARN_THROTTLE(10.0, "[ANGLE ERROR] model_states has no target model '%s'", target_name_.c_str());
            has_gt_ = false;
            return;
        }

        const Vec3 pi{msg->pose[self_model_idx].position.x,
                      msg->pose[self_model_idx].position.y,
                      msg->pose[self_model_idx].position.z};
        const Vec3 pj{msg->pose[target_model_idx].position.x,
                      msg->pose[target_model_idx].position.y,
                      msg->pose[target_model_idx].position.z};

        const double dx = pj.x - pi.x;
        const double dy = pj.y - pi.y;
        const double dz = pj.z - pi.z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (range < min_range_)
        {
            ROS_WARN_THROTTLE(10.0, "[ANGLE ERROR] %s -> %s range %.3f m is too small",
                              self_name_.c_str(), target_name_.c_str(), range);
            has_gt_ = false;
            return;
        }

        gt_angle_.target_name = target_name_;
        gt_angle_.alpha = std::atan2(dy, dx);
        gt_angle_.theta = std::asin(clamp(dz / range, -1.0, 1.0));
        gt_angle_.valid = true;
        has_gt_ = true;
    }

    void angleCallback(const data_process::CameraAngle::ConstPtr &msg)
    {
        if (!has_gt_)
        {
            ROS_WARN_THROTTLE(10.0, "[ANGLE ERROR] no valid GT angles for %s yet", self_name_.c_str());
            return;
        }

        const size_t n_det = std::min<size_t>(msg->count, std::min(msg->alpha.size(), msg->theta.size()));
        if (n_det == 0)
            return;

        if (n_det != msg->count)
        {
            ROS_WARN_THROTTLE(10.0,
                              "[ANGLE ERROR] camera_angle count mismatch: count=%u alpha=%zu theta=%zu, use %zu",
                              msg->count, msg->alpha.size(), msg->theta.size(), n_det);
        }

        const double stamp = msg->header.stamp.toSec();
        for (size_t det_idx = 0; det_idx < n_det; ++det_idx)
        {
            const double alpha_meas = wrapToPi(static_cast<double>(msg->alpha[det_idx]));
            const double theta_meas = wrapToPi(static_cast<double>(msg->theta[det_idx]));

            if (!gt_angle_.valid)
                continue;

            const double alpha_err = wrapToPi(alpha_meas - gt_angle_.alpha);
            const double theta_err = wrapToPi(theta_meas - gt_angle_.theta);
            const double error = std::sqrt(alpha_err * alpha_err + theta_err * theta_err);
            const bool within_threshold = error <= max_match_error_;

            if (!within_threshold)
            {
                ROS_WARN_THROTTLE(10.0,
                                  "[ANGLE ERROR] large detection error: %s -> %s angle error %.3f rad > %.3f rad",
                                  self_name_.c_str(), target_name_.c_str(), error, max_match_error_);
                ++over_threshold_count_;
            }

            const bool valid_meas = std::isfinite(alpha_meas) && std::isfinite(theta_meas);
            if (!valid_meas)
            {
                ROS_WARN_THROTTLE(10.0,
                                  "[ANGLE ERROR] %s det_idx=%zu alpha=%.4f theta=%.4f NaN, skip",
                                  self_name_.c_str(), det_idx, alpha_meas, theta_meas);
            }

            csv_ << stamp << ','
                 << self_name_ << ','
                 << det_idx << ','
                 << gt_angle_.target_name << ','
                 << alpha_meas << ','
                 << theta_meas << ','
                 << gt_angle_.alpha << ','
                 << gt_angle_.theta << ','
                 << alpha_err << ','
                 << theta_err << ','
                 << error << ','
                 << (within_threshold ? 1 : 0) << '\n';

            if (!valid_meas)
                continue;

            sum_alpha_sq_ += alpha_err * alpha_err;
            sum_theta_sq_ += theta_err * theta_err;
            ++sample_count_;
        }

        csv_.flush();
    }

    void reportSummary() const
    {
        if (sample_count_ == 0)
        {
            ROS_WARN("[ANGLE ERROR] %s no valid samples; RMSE unavailable", self_name_.c_str());
            return;
        }

        const double rmse_alpha = std::sqrt(sum_alpha_sq_ / static_cast<double>(sample_count_));
        const double rmse_theta = std::sqrt(sum_theta_sq_ / static_cast<double>(sample_count_));

        ROS_INFO("============================================");
        ROS_INFO("[ANGLE ERROR] %s -> %s summary", self_name_.c_str(), target_name_.c_str());
        ROS_INFO("samples: %lu", static_cast<unsigned long>(sample_count_));
        ROS_INFO("over_threshold: %lu  threshold=%.3f rad",
                 static_cast<unsigned long>(over_threshold_count_), max_match_error_);
        ROS_INFO("RMSE_alpha: %.6f rad (%.3f deg)", rmse_alpha, rmse_alpha * 180.0 / kPi);
        ROS_INFO("RMSE_theta: %.6f rad (%.3f deg)", rmse_theta, rmse_theta * 180.0 / kPi);
        ROS_INFO("============================================");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber model_sub_;
    ros::Subscriber angle_sub_;

    GtAngle gt_angle_;
    std::string self_name_;
    std::string target_name_;
    std::string csv_dir_;
    std::string csv_file_;
    bool has_gt_ = false;
    double min_range_ = 0.05;
    double max_match_error_ = 1.0;

    std::ofstream csv_;
    uint64_t sample_count_ = 0;
    uint64_t over_threshold_count_ = 0;
    double sum_alpha_sq_ = 0.0;
    double sum_theta_sq_ = 0.0;
    bool reported_ = false;
};

AngleErrorCal *g_angle_error_node = nullptr;

void sigintHandler(int)
{
    if (g_angle_error_node != nullptr)
        g_angle_error_node->shutdownAndReport();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "angle_error_cal", ros::init_options::NoSigintHandler);
    AngleErrorCal node;
    g_angle_error_node = &node;
    std::signal(SIGINT, sigintHandler);
    ros::spin();
    node.shutdownAndReport();
    g_angle_error_node = nullptr;
    return 0;
}
