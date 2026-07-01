#include <ros/ros.h>

#include <data_process/CameraAngleMatch.h>
#include <data_process/UwbProcessed.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

namespace
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

constexpr double kEps = 1e-9;

double wrapAngle(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

bool isFinite(const Eigen::VectorXd &v)
{
    for (int i = 0; i < v.size(); ++i)
    {
        if (!std::isfinite(v[i]))
            return false;
    }
    return true;
}

bool isFinite6(const Vector6d &v)
{
    for (int i = 0; i < 6; ++i)
    {
        if (!std::isfinite(v[i]))
            return false;
    }
    return true;
}

bool makeDirRecursive(const std::string &path)
{
    if (path.empty())
        return true;

    std::string current;
    if (path[0] == '/')
        current = "/";

    std::stringstream ss(path);
    std::string item;
    while (std::getline(ss, item, '/'))
    {
        if (item.empty())
            continue;
        if (!current.empty() && current.back() != '/')
            current += "/";
        current += item;

        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            return false;
    }
    return true;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::Point &p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::Vector3 &v)
{
    return Eigen::Vector3d(v.x, v.y, v.z);
}

Eigen::Vector3d odomPosition(const nav_msgs::Odometry &msg)
{
    return pointToEigen(msg.pose.pose.position);
}

Eigen::Vector3d odomVelocity(const nav_msgs::Odometry &msg)
{
    return vectorToEigen(msg.twist.twist.linear);
}
}  // namespace

class DelayRelativeEkf
{
public:
    DelayRelativeEkf()
        : nh_(), pnh_("~")
    {
        pnh_.param("uav_id", uav_id_, inferUavId());
        pnh_.param("uav_num", uav_num_, 4);
        pnh_.param("initial_spacing", initial_spacing_, 2.0);
        pnh_.param("rate", rate_, 30.0);
        pnh_.param("history_keep_time", history_keep_time_, 1.0);
        pnh_.param("max_history_match_dt", max_history_match_dt_, 0.06);
        pnh_.param("delay_threshold", delay_threshold_, 0.035);
        pnh_.param("max_observation_delay", max_observation_delay_, 0.60);
        pnh_.param("max_future_observation_dt", max_future_observation_dt_, 0.05);
        pnh_.param("max_dgo_pair_dt", max_dgo_pair_dt_, 0.12);
        pnh_.param("accel_noise_std", accel_noise_std_, 0.8);
        pnh_.param("dgo_position_noise_std", dgo_position_noise_std_, 0.18);
        pnh_.param("uwb_noise_std", uwb_noise_std_, 0.05);
        pnh_.param("camera_alpha_noise_std", camera_alpha_noise_std_, 0.05);
        pnh_.param("camera_theta_noise_std", camera_theta_noise_std_, 0.05);
        pnh_.param("initial_position_std", initial_position_std_, 0.75);
        pnh_.param("initial_velocity_std", initial_velocity_std_, 0.50);
        pnh_.param("max_measurement_nis", max_measurement_nis_, 100.0);
        pnh_.param("max_current_correction_norm", max_current_correction_norm_, 1.0);
        pnh_.param("max_delayed_correction_norm", max_delayed_correction_norm_, 0.6);
        pnh_.param("use_delayed_compensation", use_delayed_compensation_, true);
        pnh_.param("use_camera_alpha", use_camera_alpha_, true);
        pnh_.param("use_camera_theta", use_camera_theta_, true);
        pnh_.param("use_gazebo_initial_offsets", use_gazebo_initial_offsets_, true);
        pnh_.param("origin_capture_delay", origin_capture_delay_, 1.0);
        pnh_.param("stop_after_mission_complete", stop_after_mission_complete_, true);
        pnh_.param("dekf_stop_stage", dekf_stop_stage_, 7);
        pnh_.param("csv_dir", csv_dir_, std::string(""));

        if (uav_num_ <= 1 || uav_num_ > 32 || uav_id_ < 0 || uav_id_ >= uav_num_)
        {
            ROS_FATAL("[DEKF] invalid uav_id=%d uav_num=%d", uav_id_, uav_num_);
            ros::shutdown();
            return;
        }
        if (rate_ <= 0.0)
            rate_ = 30.0;
        if (history_keep_time_ < 0.2)
            history_keep_time_ = 0.2;
        if (delay_threshold_ < 0.0)
            delay_threshold_ = 0.0;
        if (max_history_match_dt_ <= 0.0)
            max_history_match_dt_ = 0.06;
        sanitizeStddev(accel_noise_std_, 0.8, "accel_noise_std");
        sanitizeStddev(dgo_position_noise_std_, 0.18, "dgo_position_noise_std");
        sanitizeStddev(uwb_noise_std_, 0.05, "uwb_noise_std");
        sanitizeStddev(camera_alpha_noise_std_, 0.05, "camera_alpha_noise_std");
        sanitizeStddev(camera_theta_noise_std_, 0.05, "camera_theta_noise_std");
        sanitizeStddev(initial_position_std_, 0.75, "initial_position_std");
        sanitizeStddev(initial_velocity_std_, 0.50, "initial_velocity_std");

        drone_names_.reserve(uav_num_);
        for (int i = 0; i < uav_num_; ++i)
            drone_names_.push_back("iris_" + std::to_string(i));

        initFallbackOrigins();

        dgo_histories_.resize(uav_num_);
        filters_.resize(uav_num_);
        for (int i = 0; i < uav_num_; ++i)
        {
            filters_[i].target_id = i;
            if (i == uav_id_)
                continue;
            filters_[i].pub = nh_.advertise<nav_msgs::Odometry>(
                "dekf_relative/" + drone_names_[i], 10);
        }

        for (int i = 0; i < uav_num_; ++i)
        {
            const std::string topic = "/" + drone_names_[i] + "/dgo_estimate";
            dgo_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                topic, 20,
                boost::bind(&DelayRelativeEkf::dgoCallback, this, _1, i)));
        }

        uwb_sub_ = nh_.subscribe("uwb_processed", 20,
                                 &DelayRelativeEkf::uwbCallback, this);
        camera_sub_ = nh_.subscribe("camera_angle_match", 10,
                                    &DelayRelativeEkf::cameraCallback, this);
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                   &DelayRelativeEkf::modelStatesCallback, this);
        stage_sub_ = nh_.subscribe("/formation/stage", 1,
                                   &DelayRelativeEkf::stageCallback, this);

        openCsv();

        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_),
                                 &DelayRelativeEkf::timerCallback, this);

        ROS_INFO("[DEKF] ns=%s uav_id=%d uav_num=%d rate=%.1fHz "
                 "delay_threshold=%.3f max_obs_delay=%.3f history=%.2fs "
                 "dgo_std=%.3f uwb_std=%.3f cam_alpha_std=%.3f cam_theta_std=%.3f "
                 "delayed_comp=%d stop_stage=%d csv_dir=%s",
                 ros::this_node::getNamespace().c_str(),
                 uav_id_, uav_num_, rate_,
                 delay_threshold_, max_observation_delay_, history_keep_time_,
                 dgo_position_noise_std_, uwb_noise_std_,
                 camera_alpha_noise_std_, camera_theta_noise_std_,
                 use_delayed_compensation_ ? 1 : 0,
                 dekf_stop_stage_, csv_dir_.empty() ? "(disabled)" : csv_dir_.c_str());
    }

private:
    enum class ObsType
    {
        DGO_POSITION,
        UWB_RANGE,
        CAMERA_ALPHA,
        CAMERA_THETA
    };

    struct OdomSample
    {
        ros::Time stamp;
        nav_msgs::Odometry msg;
    };

    struct Observation
    {
        ObsType type = ObsType::DGO_POSITION;
        int target_id = -1;
        ros::Time stamp;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double value = 0.0;
    };

    struct HistoryRecord
    {
        ros::Time stamp;
        Vector6d x_pred = Vector6d::Zero();
        Matrix6d P_pred = Matrix6d::Identity();
        Matrix6d A = Matrix6d::Identity();
        Matrix6d update_transition = Matrix6d::Identity();
    };

    struct PairFilter
    {
        int target_id = -1;
        bool initialized = false;
        ros::Time stamp;
        Vector6d x = Vector6d::Zero();
        Matrix6d P = Matrix6d::Identity();
        std::deque<HistoryRecord> history;
        ros::Time last_dgo_obs_stamp;
        ros::Publisher pub;

        size_t dgo_updates = 0;
        size_t uwb_updates = 0;
        size_t camera_updates = 0;
        size_t delayed_updates = 0;
        size_t rejected_updates = 0;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;
    std::vector<ros::Subscriber> dgo_subs_;
    ros::Subscriber uwb_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber model_sub_;
    ros::Subscriber stage_sub_;

    int uav_id_ = 0;
    int uav_num_ = 4;
    double initial_spacing_ = 2.0;
    double rate_ = 30.0;
    double history_keep_time_ = 1.0;
    double max_history_match_dt_ = 0.06;
    double delay_threshold_ = 0.035;
    double max_observation_delay_ = 0.60;
    double max_future_observation_dt_ = 0.05;
    double max_dgo_pair_dt_ = 0.12;
    double accel_noise_std_ = 0.8;
    double dgo_position_noise_std_ = 0.18;
    double uwb_noise_std_ = 0.05;
    double camera_alpha_noise_std_ = 0.05;
    double camera_theta_noise_std_ = 0.05;
    double initial_position_std_ = 0.75;
    double initial_velocity_std_ = 0.50;
    double max_measurement_nis_ = 100.0;
    double max_current_correction_norm_ = 1.0;
    double max_delayed_correction_norm_ = 0.6;
    bool use_delayed_compensation_ = true;
    bool use_camera_alpha_ = true;
    bool use_camera_theta_ = true;
    bool use_gazebo_initial_offsets_ = true;
    double origin_capture_delay_ = 1.0;
    bool stop_after_mission_complete_ = true;
    int dekf_stop_stage_ = 7;
    uint8_t mission_stage_ = 0;
    bool has_stage_ = false;

    std::vector<std::string> drone_names_;
    std::vector<Eigen::Vector3d> origins_;
    std::vector<bool> origin_received_;
    bool origin_locked_ = false;
    ros::Time origin_capture_start_;

    std::vector<std::deque<OdomSample>> dgo_histories_;
    std::vector<PairFilter> filters_;
    std::deque<Observation> pending_observations_;

    std::string csv_dir_;
    std::ofstream csv_;

    static int inferUavId()
    {
        std::string ns = ros::this_node::getNamespace();
        if (!ns.empty() && ns.front() == '/')
            ns = ns.substr(1);
        const std::string prefix = "iris_";
        const size_t pos = ns.rfind(prefix);
        if (pos == std::string::npos)
            return 0;
        return std::atoi(ns.substr(pos + prefix.size()).c_str());
    }

    static void sanitizeStddev(double &value, double fallback, const char *name)
    {
        if (!std::isfinite(value) || value <= 0.0)
        {
            ROS_WARN("[DEKF] invalid %s=%.6f, using %.6f",
                     name, value, fallback);
            value = fallback;
        }
    }

    void initFallbackOrigins()
    {
        origins_.assign(uav_num_, Eigen::Vector3d::Zero());
        origin_received_.assign(uav_num_, false);
        for (int i = 0; i < uav_num_; ++i)
        {
            const int row = i / 2;
            const int col = i % 2;
            origins_[i] = Eigen::Vector3d(col * initial_spacing_,
                                          row * initial_spacing_,
                                          0.0);
        }
    }

    void openCsv()
    {
        if (csv_dir_.empty())
            return;
        if (!makeDirRecursive(csv_dir_))
        {
            ROS_WARN("[DEKF] failed to create csv_dir=%s", csv_dir_.c_str());
            return;
        }
        const std::string path =
            csv_dir_ + "/" + drone_names_[uav_id_] + "_dekf_debug.csv";
        csv_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
        {
            ROS_WARN("[DEKF] cannot open CSV: %s", path.c_str());
            return;
        }
        csv_ << "time,stage,target,source,accepted,delayed,obs_stamp,age,"
             << "residual_norm,nis,x,y,z,vx,vy,vz,"
             << "dgo_updates,uwb_updates,camera_updates,delayed_updates,rejected\n";
        ROS_INFO("[DEKF] debug CSV: %s", path.c_str());
    }

    void dgoCallback(const nav_msgs::Odometry::ConstPtr &msg, int id)
    {
        if (id < 0 || id >= uav_num_)
            return;
        OdomSample s;
        s.stamp = msg->header.stamp;
        s.msg = *msg;
        dgo_histories_[id].push_back(s);
        pruneOdomHistory(dgo_histories_[id], msg->header.stamp);
    }

    void uwbCallback(const data_process::UwbProcessed::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->target_ids.size() &&
                           i < msg->distances.size(); ++i)
        {
            const int target = static_cast<int>(msg->target_ids[i]);
            const double dist = msg->distances[i];
            if (target < 0 || target >= uav_num_ || target == uav_id_)
                continue;
            if (!std::isfinite(dist) || dist <= 0.05)
                continue;

            Observation obs;
            obs.type = ObsType::UWB_RANGE;
            obs.target_id = target;
            obs.stamp = msg->header.stamp;
            obs.value = dist;
            pending_observations_.push_back(obs);
        }
        prunePending();
    }

    void cameraCallback(const data_process::CameraAngleMatch::ConstPtr &msg)
    {
        const size_t n = std::min(
            msg->id.size(), std::min(msg->alpha.size(), msg->theta.size()));
        for (size_t i = 0; i < n; ++i)
        {
            const int target = static_cast<int>(msg->id[i]);
            if (target < 0 || target >= uav_num_ || target == uav_id_)
                continue;

            if (use_camera_alpha_ && std::isfinite(msg->alpha[i]))
            {
                Observation obs;
                obs.type = ObsType::CAMERA_ALPHA;
                obs.target_id = target;
                obs.stamp = msg->header.stamp;
                obs.value = static_cast<double>(msg->alpha[i]);
                pending_observations_.push_back(obs);
            }
            if (use_camera_theta_ && std::isfinite(msg->theta[i]))
            {
                Observation obs;
                obs.type = ObsType::CAMERA_THETA;
                obs.target_id = target;
                obs.stamp = msg->header.stamp;
                obs.value = static_cast<double>(msg->theta[i]);
                pending_observations_.push_back(obs);
            }
        }
        prunePending();
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        if (!use_gazebo_initial_offsets_ || origin_locked_)
            return;

        const ros::Time now = ros::Time::now();
        if (origin_capture_start_.isZero())
            origin_capture_start_ = now;

        for (int i = 0; i < uav_num_; ++i)
        {
            if (origin_received_[i])
                continue;
            const auto it = std::find(msg->name.begin(), msg->name.end(),
                                      drone_names_[i]);
            if (it == msg->name.end())
                continue;
            const size_t idx = std::distance(msg->name.begin(), it);
            if (idx >= msg->pose.size())
                continue;
            origins_[i] = pointToEigen(msg->pose[idx].position);
            origin_received_[i] = true;
        }

        const bool all_received = std::all_of(
            origin_received_.begin(), origin_received_.end(),
            [](bool v) { return v; });
        if (all_received &&
            (now - origin_capture_start_).toSec() >= origin_capture_delay_)
        {
            origin_locked_ = true;
            std::ostringstream oss;
            oss << "[DEKF] locked initial Gazebo origins:";
            for (int i = 0; i < uav_num_; ++i)
            {
                oss << " " << drone_names_[i] << "=("
                    << origins_[i].x() << "," << origins_[i].y() << ","
                    << origins_[i].z() << ")";
            }
            ROS_INFO_STREAM(oss.str());
        }
    }

    void stageCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        mission_stage_ = msg->data;
        has_stage_ = true;
    }

    bool stoppedByMissionStage() const
    {
        return stop_after_mission_complete_ &&
               has_stage_ &&
               static_cast<int>(mission_stage_) >= dekf_stop_stage_;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (stoppedByMissionStage())
        {
            ROS_INFO_THROTTLE(
                5.0,
                "[DEKF] stopped after mission complete: stage=%u >= %d",
                static_cast<unsigned int>(mission_stage_),
                dekf_stop_stage_);
            return;
        }

        const ros::Time now = ros::Time::now();
        if (now.isZero())
            return;

        for (int target = 0; target < uav_num_; ++target)
        {
            if (target == uav_id_)
                continue;
            ensureInitialized(filters_[target], now);
            predictTo(filters_[target], now);
        }

        std::vector<Observation> observations;
        observations.reserve(pending_observations_.size() + uav_num_);
        while (!pending_observations_.empty())
        {
            observations.push_back(pending_observations_.front());
            pending_observations_.pop_front();
        }
        collectDgoObservations(observations);

        std::sort(observations.begin(), observations.end(),
                  [](const Observation &a, const Observation &b) {
                      if (a.stamp == b.stamp)
                          return static_cast<int>(a.type) <
                                 static_cast<int>(b.type);
                      return a.stamp < b.stamp;
                  });

        for (const Observation &obs : observations)
            processObservation(obs);

        for (int target = 0; target < uav_num_; ++target)
        {
            if (target == uav_id_)
                continue;
            publishFilter(filters_[target]);
        }
    }

    void ensureInitialized(PairFilter &f, const ros::Time &stamp)
    {
        if (f.initialized)
            return;

        f.x.setZero();
        f.x.segment<3>(0) = origins_[f.target_id] - origins_[uav_id_];
        f.P.setZero();
        f.P.block<3, 3>(0, 0).diagonal().setConstant(
            initial_position_std_ * initial_position_std_);
        f.P.block<3, 3>(3, 3).diagonal().setConstant(
            initial_velocity_std_ * initial_velocity_std_);
        f.stamp = stamp;
        f.initialized = true;

        HistoryRecord rec;
        rec.stamp = stamp;
        rec.x_pred = f.x;
        rec.P_pred = f.P;
        rec.A.setIdentity();
        rec.update_transition.setIdentity();
        f.history.push_back(rec);
    }

    void predictTo(PairFilter &f, const ros::Time &stamp)
    {
        if (!f.initialized)
            return;

        double dt = (stamp - f.stamp).toSec();
        if (dt < -1e-6)
            return;
        if (dt < 1e-6)
            return;
        dt = std::min(dt, 0.25);

        Matrix6d A = Matrix6d::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 3> G;
        G.setZero();
        G.block<3, 3>(0, 0) =
            0.5 * dt * dt * Eigen::Matrix3d::Identity();
        G.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d Qa =
            accel_noise_std_ * accel_noise_std_ *
            Eigen::Matrix3d::Identity();

        f.x = A * f.x;
        f.P = A * f.P * A.transpose() + G * Qa * G.transpose();
        symmetrize(f.P);
        f.stamp = stamp;

        HistoryRecord rec;
        rec.stamp = stamp;
        rec.x_pred = f.x;
        rec.P_pred = f.P;
        rec.A = A;
        rec.update_transition.setIdentity();
        f.history.push_back(rec);
        pruneFilterHistory(f);
    }

    void collectDgoObservations(std::vector<Observation> &observations)
    {
        if (dgo_histories_[uav_id_].empty())
            return;
        const OdomSample &self = dgo_histories_[uav_id_].back();

        for (int target = 0; target < uav_num_; ++target)
        {
            if (target == uav_id_ || dgo_histories_[target].empty())
                continue;
            const OdomSample &peer = dgo_histories_[target].back();
            const double pair_dt =
                std::fabs((peer.stamp - self.stamp).toSec());
            if (pair_dt > max_dgo_pair_dt_)
                continue;

            ros::Time obs_stamp = self.stamp < peer.stamp ?
                                  self.stamp : peer.stamp;
            PairFilter &f = filters_[target];
            if (!f.last_dgo_obs_stamp.isZero() &&
                obs_stamp <= f.last_dgo_obs_stamp)
                continue;
            f.last_dgo_obs_stamp = obs_stamp;

            Observation obs;
            obs.type = ObsType::DGO_POSITION;
            obs.target_id = target;
            obs.stamp = obs_stamp;
            obs.position = origins_[target] - origins_[uav_id_] +
                           odomPosition(peer.msg) - odomPosition(self.msg);
            obs.velocity = odomVelocity(peer.msg) - odomVelocity(self.msg);
            if (!obs.position.allFinite() || !obs.velocity.allFinite())
                continue;
            observations.push_back(obs);

            if (f.initialized && f.dgo_updates == 0)
            {
                f.x.segment<3>(0) = obs.position;
                f.x.segment<3>(3) = obs.velocity;
            }
        }
    }

    void processObservation(const Observation &obs)
    {
        if (obs.target_id < 0 || obs.target_id >= uav_num_ ||
            obs.target_id == uav_id_)
            return;
        PairFilter &f = filters_[obs.target_id];
        if (!f.initialized)
            return;

        const double age = (f.stamp - obs.stamp).toSec();
        if (age < -max_future_observation_dt_)
        {
            logUpdate(f, obs, false, false, age,
                      std::numeric_limits<double>::quiet_NaN(), 0.0);
            ++f.rejected_updates;
            return;
        }
        if (age > max_observation_delay_)
        {
            logUpdate(f, obs, false, false, age,
                      std::numeric_limits<double>::quiet_NaN(), 0.0);
            ++f.rejected_updates;
            return;
        }

        bool accepted = false;
        bool delayed = false;
        if (use_delayed_compensation_ && age > delay_threshold_)
        {
            delayed = true;
            accepted = applyDelayedUpdate(f, obs, age);
        }

        if (!accepted && (!delayed || age <= 2.0 * delay_threshold_))
        {
            delayed = false;
            accepted = applyCurrentUpdate(f, obs, age);
        }

        if (!accepted)
            ++f.rejected_updates;
    }

    bool applyCurrentUpdate(PairFilter &f,
                            const Observation &obs,
                            double age)
    {
        Eigen::VectorXd z, h;
        Eigen::MatrixXd H, R;
        if (!buildMeasurement(obs, f.x, z, h, H, R))
            return false;

        Eigen::VectorXd residual = z - h;
        normalizeResidual(obs.type, residual);
        if (!isFinite(residual))
            return false;

        Eigen::MatrixXd S = H * f.P * H.transpose() + R;
        Eigen::MatrixXd S_inv;
        if (!invertSmall(S, S_inv))
            return false;

        const double nis = (residual.transpose() * S_inv * residual)(0, 0);
        if (max_measurement_nis_ > 0.0 && nis > max_measurement_nis_)
        {
            logUpdate(f, obs, false, false, age, nis, residual.norm());
            return false;
        }

        Eigen::MatrixXd K = f.P * H.transpose() * S_inv;
        Vector6d dx = K * residual;
        limitCorrection(dx, max_current_correction_norm_);
        if (!isFinite6(dx))
            return false;

        f.x += dx;

        const Matrix6d I = Matrix6d::Identity();
        const Matrix6d I_KH = I - K * H;
        f.P = I_KH * f.P * I_KH.transpose() + K * R * K.transpose();
        symmetrize(f.P);

        if (!f.history.empty())
            f.history.back().update_transition =
                I_KH * f.history.back().update_transition;

        incrementCounter(f, obs.type, false);
        logUpdate(f, obs, true, false, age, nis, residual.norm());
        return true;
    }

    bool applyDelayedUpdate(PairFilter &f,
                            const Observation &obs,
                            double age)
    {
        int hist_idx = -1;
        if (!findHistory(f, obs.stamp, hist_idx))
        {
            logUpdate(f, obs, false, true, age,
                      std::numeric_limits<double>::quiet_NaN(), 0.0);
            return false;
        }

        const HistoryRecord &rec = f.history[hist_idx];
        Eigen::VectorXd z, h;
        Eigen::MatrixXd H, R;
        if (!buildMeasurement(obs, rec.x_pred, z, h, H, R))
            return false;

        Eigen::VectorXd residual = z - h;
        normalizeResidual(obs.type, residual);
        if (!isFinite(residual))
            return false;

        Eigen::MatrixXd S = H * rec.P_pred * H.transpose() + R;
        Eigen::MatrixXd S_inv;
        if (!invertSmall(S, S_inv))
            return false;

        const double nis = (residual.transpose() * S_inv * residual)(0, 0);
        if (max_measurement_nis_ > 0.0 && nis > max_measurement_nis_)
        {
            logUpdate(f, obs, false, true, age, nis, residual.norm());
            return false;
        }

        Eigen::MatrixXd K_s = rec.P_pred * H.transpose() * S_inv;
        Vector6d delta = K_s * residual;
        limitCorrection(delta, max_delayed_correction_norm_);
        if (!isFinite6(delta))
            return false;

        for (size_t k = static_cast<size_t>(hist_idx) + 1;
             k < f.history.size(); ++k)
        {
            delta = f.history[k].update_transition *
                    f.history[k].A * delta;
        }
        limitCorrection(delta, max_delayed_correction_norm_);
        if (!isFinite6(delta))
            return false;

        f.x += delta;

        // 论文 DEKF 的核心是对延迟观测产生的历史修正进行传播。
        // 这里不再对当前 P 做一次普通量测收缩，避免同一旧观测被重复计数。
        incrementCounter(f, obs.type, true);
        logUpdate(f, obs, true, true, age, nis, residual.norm());
        return true;
    }

    bool buildMeasurement(const Observation &obs,
                          const Vector6d &x,
                          Eigen::VectorXd &z,
                          Eigen::VectorXd &h,
                          Eigen::MatrixXd &H,
                          Eigen::MatrixXd &R) const
    {
        const Eigen::Vector3d p = x.segment<3>(0);
        switch (obs.type)
        {
        case ObsType::DGO_POSITION:
            z.resize(3);
            z = obs.position;
            h.resize(3);
            h = p;
            H = Eigen::MatrixXd::Zero(3, 6);
            H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            R = Eigen::MatrixXd::Identity(3, 3) *
                dgo_position_noise_std_ * dgo_position_noise_std_;
            return true;

        case ObsType::UWB_RANGE:
        {
            const double dist = p.norm();
            if (dist < 0.10 || !std::isfinite(dist))
                return false;
            z.resize(1);
            z[0] = obs.value;
            h.resize(1);
            h[0] = dist;
            H = Eigen::MatrixXd::Zero(1, 6);
            H.block<1, 3>(0, 0) = p.transpose() / dist;
            R = Eigen::MatrixXd::Identity(1, 1) *
                uwb_noise_std_ * uwb_noise_std_;
            return true;
        }

        case ObsType::CAMERA_ALPHA:
        {
            const double rho2 = p.x() * p.x() + p.y() * p.y();
            if (rho2 < 1e-6)
                return false;
            z.resize(1);
            z[0] = obs.value;
            h.resize(1);
            h[0] = std::atan2(p.y(), p.x());
            H = Eigen::MatrixXd::Zero(1, 6);
            H(0, 0) = -p.y() / rho2;
            H(0, 1) = p.x() / rho2;
            R = Eigen::MatrixXd::Identity(1, 1) *
                camera_alpha_noise_std_ * camera_alpha_noise_std_;
            return true;
        }

        case ObsType::CAMERA_THETA:
        {
            const double rho2 = p.x() * p.x() + p.y() * p.y();
            const double rho = std::sqrt(rho2);
            const double r2 = rho2 + p.z() * p.z();
            if (rho < 1e-6 || r2 < 1e-6)
                return false;
            z.resize(1);
            z[0] = obs.value;
            h.resize(1);
            h[0] = std::atan2(p.z(), rho);
            H = Eigen::MatrixXd::Zero(1, 6);
            H(0, 0) = -p.z() * p.x() / (rho * r2);
            H(0, 1) = -p.z() * p.y() / (rho * r2);
            H(0, 2) = rho / r2;
            R = Eigen::MatrixXd::Identity(1, 1) *
                camera_theta_noise_std_ * camera_theta_noise_std_;
            return true;
        }
        }
        return false;
    }

    static void normalizeResidual(ObsType type, Eigen::VectorXd &residual)
    {
        if ((type == ObsType::CAMERA_ALPHA ||
             type == ObsType::CAMERA_THETA) &&
            residual.size() == 1)
        {
            residual[0] = wrapAngle(residual[0]);
        }
    }

    static bool invertSmall(const Eigen::MatrixXd &S, Eigen::MatrixXd &S_inv)
    {
        if (S.rows() != S.cols() || S.rows() == 0)
            return false;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
        if (!lu.isInvertible())
            return false;
        S_inv = lu.inverse();
        return S_inv.allFinite();
    }

    static void symmetrize(Matrix6d &P)
    {
        P = 0.5 * (P + P.transpose());
        for (int i = 0; i < 6; ++i)
        {
            if (!std::isfinite(P(i, i)) || P(i, i) < 1e-9)
                P(i, i) = 1e-9;
        }
    }

    static void limitCorrection(Vector6d &dx, double max_norm)
    {
        if (max_norm <= 0.0)
            return;
        const double n = dx.norm();
        if (n > max_norm && n > kEps)
            dx *= max_norm / n;
    }

    bool findHistory(const PairFilter &f,
                     const ros::Time &stamp,
                     int &idx) const
    {
        idx = -1;
        double best_dt = std::numeric_limits<double>::max();
        for (size_t i = 0; i < f.history.size(); ++i)
        {
            const double dt = std::fabs((f.history[i].stamp - stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                idx = static_cast<int>(i);
            }
        }
        return idx >= 0 && best_dt <= max_history_match_dt_;
    }

    void incrementCounter(PairFilter &f, ObsType type, bool delayed)
    {
        if (delayed)
            ++f.delayed_updates;

        switch (type)
        {
        case ObsType::DGO_POSITION:
            ++f.dgo_updates;
            break;
        case ObsType::UWB_RANGE:
            ++f.uwb_updates;
            break;
        case ObsType::CAMERA_ALPHA:
        case ObsType::CAMERA_THETA:
            ++f.camera_updates;
            break;
        }
    }

    const char *sourceName(ObsType type) const
    {
        switch (type)
        {
        case ObsType::DGO_POSITION:
            return "dgo";
        case ObsType::UWB_RANGE:
            return "uwb";
        case ObsType::CAMERA_ALPHA:
            return "camera_alpha";
        case ObsType::CAMERA_THETA:
            return "camera_theta";
        }
        return "unknown";
    }

    void logUpdate(const PairFilter &f,
                   const Observation &obs,
                   bool accepted,
                   bool delayed,
                   double age,
                   double nis,
                   double residual_norm)
    {
        if (!csv_.is_open())
            return;
        csv_ << std::fixed << std::setprecision(6)
             << ros::Time::now().toSec() << ","
             << static_cast<unsigned int>(mission_stage_) << ","
             << obs.target_id << ","
             << sourceName(obs.type) << ","
             << (accepted ? 1 : 0) << ","
             << (delayed ? 1 : 0) << ","
             << obs.stamp.toSec() << ","
             << age << ","
             << residual_norm << ","
             << nis << ","
             << f.x[0] << "," << f.x[1] << "," << f.x[2] << ","
             << f.x[3] << "," << f.x[4] << "," << f.x[5] << ","
             << f.dgo_updates << ","
             << f.uwb_updates << ","
             << f.camera_updates << ","
             << f.delayed_updates << ","
             << f.rejected_updates << "\n";
    }

    void publishFilter(const PairFilter &f)
    {
        if (!f.initialized || !f.pub)
            return;

        nav_msgs::Odometry msg;
        msg.header.stamp = f.stamp;
        msg.header.frame_id = drone_names_[uav_id_] + "/initial_enu";
        msg.child_frame_id = drone_names_[f.target_id] + "/relative";
        msg.pose.pose.orientation.w = 1.0;
        msg.pose.pose.position.x = f.x[0];
        msg.pose.pose.position.y = f.x[1];
        msg.pose.pose.position.z = f.x[2];
        msg.twist.twist.linear.x = f.x[3];
        msg.twist.twist.linear.y = f.x[4];
        msg.twist.twist.linear.z = f.x[5];

        for (int r = 0; r < 6; ++r)
        {
            for (int c = 0; c < 6; ++c)
            {
                if (r < 3 && c < 3)
                    msg.pose.covariance[r * 6 + c] = f.P(r, c);
                if (r >= 3 && c >= 3)
                    msg.twist.covariance[(r - 3) * 6 + (c - 3)] = f.P(r, c);
            }
        }

        f.pub.publish(msg);
    }

    void pruneOdomHistory(std::deque<OdomSample> &hist,
                          const ros::Time &now)
    {
        const double keep = history_keep_time_ + max_observation_delay_ + 0.5;
        while (!hist.empty() && (now - hist.front().stamp).toSec() > keep)
            hist.pop_front();
    }

    void pruneFilterHistory(PairFilter &f)
    {
        while (!f.history.empty() &&
               (f.stamp - f.history.front().stamp).toSec() >
                   history_keep_time_)
        {
            f.history.pop_front();
        }
    }

    void prunePending()
    {
        const size_t max_pending = 1000;
        while (pending_observations_.size() > max_pending)
            pending_observations_.pop_front();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dekf");
    DelayRelativeEkf node;
    ros::spin();
    return 0;
}
