/**
 * INS ESKF 评估节点
 *
 * 订阅:
 *   /gazebo/model_states                 — Gazebo 真实位姿/速度
 *   /<drone>/ins_estimate                 — EKF 估计值 (nav_msgs/Odometry)
 *
 * 计算指标:
 *   ATE      — 绝对轨迹误差 (RMSE, GT/EST 均使用相对初始 ENU 位置)
 *   RPE(Δ)   — 相对平移误差 (Δ=1s,5s,10s)
 *   Attitude — 姿态误差 (RMSE, deg)
 *   Velocity — 速度误差 (RMSE, m/s)
 *   NEES     — 归一化估计误差平方 (位置/速度)
 *   Drift    — 漂移率 (ATE/时长)
 *
 * 输出:
 *   - 屏幕打印指标摘要
 *   - CSV 轨迹文件 (供 evo 等工具分析)
 *
 * 使用方法:
 *   roslaunch data_process ins_eskf_test.launch
 *   Ctrl-C 停止时自动打印所有指标
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdio>
#include <csignal>
#include <cerrno>
#include <deque>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <limits>
#include <sys/stat.h>
#include <sys/types.h>

// =============================================================================
//  InsEkfTest 类
// =============================================================================

class InsEkfTest
{
public:
    InsEkfTest() : nh_("~")
    {
        // 读取无人机名称列表，默认 iris_0 ~ iris_3
        std::vector<std::string> default_names = {"iris_0", "iris_1", "iris_2", "iris_3"};
        nh_.param("drone_names", drone_names_, default_names);
        num_drones_ = drone_names_.size();

        // 输出 CSV 目录
        nh_.param("csv_dir", csv_dir_, std::string("."));
        nh_.param("reference_name", reference_name_, std::string("iris_0"));
        nh_.param("initial_spacing", initial_spacing_, 2.0);
        nh_.param("max_ekf_align_dt", max_ekf_align_dt_, 0.12);
        nh_.param("max_model_align_dt", max_model_align_dt_, 0.03);
        ensureOutputDir();

        // 初始化数据缓冲区
        gt_data_.resize(num_drones_);
        est_data_.resize(num_drones_);
        relative_samples_.resize(num_drones_);
        relative_error_sum_.assign(num_drones_, 0.0);
        relative_callbacks_.assign(num_drones_, 0);
        relative_reject_reference_.assign(num_drones_, 0);
        relative_reject_model_.assign(num_drones_, 0);

        // 订阅每架无人机的估计值。真值统一来自 /gazebo/model_states。
        // 注意: /mavros/local_position/odom 是 PX4 EKF2 估计值，不是真值。
        for (int i = 0; i < num_drones_; i++)
        {
            std::string est_topic = "/" + drone_names_[i] + "/ins_estimate";

            ins_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(est_topic, 2000,
                    boost::bind(&InsEkfTest::insCallback, this, _1, i)));
        }

        model_states_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 2000,
            &InsEkfTest::modelStatesCallback, this);

        // 定时输出进度信息 (每 10 秒)
        print_timer_ = nh_.createTimer(ros::Duration(10.0),
                                        &InsEkfTest::printTimerCallback, this);

        ROS_INFO("[INS TEST] 评估节点已启动，监控 %d 架无人机, relative reference=%s, max_ekf_align_dt=%.3f, max_model_align_dt=%.3f",
                 num_drones_, reference_name_.c_str(), max_ekf_align_dt_, max_model_align_dt_);
        for (const auto& name : drone_names_)
            ROS_INFO("  无人机: %s  (真值=/gazebo/model_states)", name.c_str());
    }

    ~InsEkfTest()
    {
        printFinalMetrics();
        writeAllCSV();
    }

private:
    // =====================================================================
    //  数据结构
    // =====================================================================

    struct TimeSeries
    {
        std::vector<ros::Time> t;
        std::vector<Eigen::Vector3d> pos;
        std::vector<Eigen::Vector3d> vel;
        std::vector<Eigen::Quaterniond> quat;    // (w,x,y,z)
        std::vector<Eigen::Matrix3d> pos_cov;     // 位置协方差 3×3
        std::vector<Eigen::Matrix3d> vel_cov;     // 速度协方差 3×3
        bool has_pos_origin = false;
        Eigen::Vector3d pos_origin{0.0, 0.0, 0.0};
    };

    struct AlignedSample
    {
        ros::Time t;
        Eigen::Vector3d est_pos, gt_pos;
        Eigen::Vector3d est_vel, gt_vel;
        Eigen::Quaterniond est_quat, gt_quat;
        Eigen::Matrix3d est_pos_cov, est_vel_cov;
    };

    struct ModelStateSample
    {
        ros::Time t;
        std::vector<Eigen::Vector3d> pos;
    };

    struct RelativeErrorSample
    {
        double timestamp = 0.0;
        double ekf_dt = 0.0;
        double model_dt = 0.0;
        Eigen::Vector3d rel_gt{0.0, 0.0, 0.0};
        Eigen::Vector3d rel_est{0.0, 0.0, 0.0};
        Eigen::Vector3d err{0.0, 0.0, 0.0};
        double error_norm = 0.0;
        double cumulative_rmse = 0.0;
    };

    // =====================================================================
    //  成员变量
    // =====================================================================

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> ins_subs_;
    ros::Subscriber model_states_sub_;
    ros::Timer print_timer_;

    int num_drones_;
    std::vector<std::string> drone_names_;
    std::string csv_dir_;
    std::string reference_name_;
    double initial_spacing_ = 2.0;
    double max_ekf_align_dt_ = 0.12;
    double max_model_align_dt_ = 0.03;
    std::vector<TimeSeries> gt_data_;
    std::vector<TimeSeries> est_data_;
    std::deque<ModelStateSample> model_states_cache_;
    std::vector<std::vector<RelativeErrorSample>> relative_samples_;
    std::vector<double> relative_error_sum_;
    std::vector<size_t> relative_callbacks_;
    std::vector<size_t> relative_reject_reference_;
    std::vector<size_t> relative_reject_model_;
    static constexpr size_t kMaxModelCacheSize = 5000;

    void ensureOutputDir() const
    {
        if (csv_dir_.empty() || csv_dir_ == ".")
            return;

        boost::system::error_code ec;
        boost::filesystem::create_directories(csv_dir_, ec);
        if (ec)
        {
            ROS_WARN("[INS TEST] failed to create csv_dir: %s", csv_dir_.c_str());
        }
    }

    // =====================================================================
    //  估计值回调 (ins_estimate) —— 同时存储协方差用于 NEES
    // =====================================================================

    void insCallback(const nav_msgs::Odometry::ConstPtr& msg, int drone_id)
    {
        TimeSeries& est = est_data_[drone_id];
        est.t.push_back(msg->header.stamp);
        est.pos.emplace_back(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
        est.vel.emplace_back(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z);
        est.quat.emplace_back(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);

        // 提取协方差
        Eigen::Matrix3d P_pos, P_vel;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                P_pos(r, c) = msg->pose.covariance[r * 6 + c];
                P_vel(r, c) = msg->twist.covariance[r * 6 + c];
            }
        }
        est.pos_cov.push_back(P_pos);
        est.vel_cov.push_back(P_vel);

        processRelativeSample(drone_id, msg->header.stamp, est.pos.back());
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        ModelStateSample sample;
        sample.t = ros::Time::now();
        sample.pos.resize(num_drones_, Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));

        bool has_any = false;
        for (int d = 0; d < num_drones_; ++d)
        {
            const int idx = findModelIndex(msg, drone_names_[d]);
            if (idx < 0)
                continue;

            const Eigen::Vector3d pos(msg->pose[idx].position.x,
                                      msg->pose[idx].position.y,
                                      msg->pose[idx].position.z);
            sample.pos[d] = pos;
            has_any = true;

            TimeSeries& gt = gt_data_[d];
            if (!gt.has_pos_origin)
            {
                gt.pos_origin = pos;
                gt.has_pos_origin = true;
            }

            gt.t.push_back(sample.t);
            gt.pos.push_back(pos - gt.pos_origin);

            if (static_cast<size_t>(idx) < msg->twist.size())
            {
                gt.vel.emplace_back(msg->twist[idx].linear.x,
                                    msg->twist[idx].linear.y,
                                    msg->twist[idx].linear.z);
            }
            else
            {
                gt.vel.emplace_back(Eigen::Vector3d::Zero());
            }

            gt.quat.emplace_back(msg->pose[idx].orientation.w,
                                 msg->pose[idx].orientation.x,
                                 msg->pose[idx].orientation.y,
                                 msg->pose[idx].orientation.z);
        }

        if (!has_any)
            return;

        model_states_cache_.push_back(sample);
        while (model_states_cache_.size() > kMaxModelCacheSize)
            model_states_cache_.pop_front();
    }

    // =====================================================================
    //  数据对齐: 为每个估计值找到最近的真值 (双指针 O(N))
    // =====================================================================

    std::vector<AlignedSample> alignData(const TimeSeries& est,
                                          const TimeSeries& gt,
                                          double max_dt = 0.05) const
    {
        std::vector<AlignedSample> aligned;
        if (est.t.empty() || gt.t.empty())
            return aligned;

        aligned.reserve(est.t.size());
        size_t gt_idx = 0;

        for (size_t i = 0; i < est.t.size(); i++)
        {
            double best_dt = 1e9;
            size_t best_j  = gt_idx;

            for (size_t j = gt_idx; j < gt.t.size(); j++)
            {
                double dt = std::fabs((est.t[i] - gt.t[j]).toSec());
                if (dt < best_dt)
                {
                    best_dt = dt;
                    best_j  = j;
                    gt_idx  = j;
                }
                else if (gt.t[j] > est.t[i] && dt > best_dt)
                    break;
            }

            if (best_dt < max_dt)
            {
                AlignedSample s;
                s.t          = est.t[i];
                s.est_pos    = est.pos[i];
                s.est_vel    = est.vel[i];
                s.est_quat   = est.quat[i];
                s.gt_pos     = gt.pos[best_j];
                s.gt_vel     = gt.vel[best_j];
                s.gt_quat    = gt.quat[best_j];
                s.est_pos_cov = est.pos_cov[i];
                s.est_vel_cov = est.vel_cov[i];
                aligned.push_back(s);
            }
        }
        return aligned;
    }

    int findDroneIndex(const std::string& name) const
    {
        for (int i = 0; i < num_drones_; ++i)
        {
            if (drone_names_[i] == name)
                return i;
        }
        return -1;
    }

    int findModelIndex(const gazebo_msgs::ModelStates::ConstPtr& msg,
                       const std::string& name) const
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == name)
                return static_cast<int>(i);
        }
        return -1;
    }

    int parseUavIndex(const std::string& name) const
    {
        const std::string prefix = "iris_";
        const size_t pos = name.find(prefix);
        if (pos == std::string::npos)
            return 0;

        try
        {
            return std::stoi(name.substr(pos + prefix.size()));
        }
        catch (...)
        {
            return 0;
        }
    }

    Eigen::Vector3d initialOffset(const std::string& name) const
    {
        const int idx = parseUavIndex(name);
        return Eigen::Vector3d(((idx % 2) != (idx / 2)) ? initial_spacing_ : 0.0,
                               (idx / 2) * initial_spacing_,
                               0.0);
    }

    bool selectNearestEst(const TimeSeries& series,
                          const ros::Time& ref_stamp,
                          double max_dt,
                          Eigen::Vector3d& out_pos,
                          double& nearest_dt) const
    {
        if (ref_stamp.isZero() || series.t.empty())
        {
            nearest_dt = -1.0;
            return false;
        }

        double best_dt = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < series.t.size(); ++i)
        {
            const double dt = std::fabs((series.t[i] - ref_stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best_idx = i;
            }
        }

        nearest_dt = best_dt;
        if (best_dt > max_dt)
            return false;

        out_pos = series.pos[best_idx];
        return true;
    }

    bool selectNearestModelState(const ros::Time& ref_stamp,
                                 ModelStateSample& out,
                                 double& nearest_dt) const
    {
        if (ref_stamp.isZero() || model_states_cache_.empty())
        {
            nearest_dt = -1.0;
            return false;
        }

        double best_dt = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < model_states_cache_.size(); ++i)
        {
            const double dt = std::fabs((model_states_cache_[i].t - ref_stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best_idx = i;
            }
        }

        nearest_dt = best_dt;
        if (best_dt > max_model_align_dt_)
            return false;

        out = model_states_cache_[best_idx];
        return true;
    }

    void processRelativeSample(int self_idx,
                               const ros::Time& stamp,
                               const Eigen::Vector3d& self_est_pos)
    {
        const int reference_idx = findDroneIndex(reference_name_);
        if (reference_idx < 0 || self_idx == reference_idx)
            return;

        if (self_idx < 0 || self_idx >= num_drones_)
            return;

        ++relative_callbacks_[self_idx];

        const TimeSeries& ref_est = est_data_[reference_idx];
        Eigen::Vector3d ref_est_pos;
        double ekf_dt = -1.0;
        if (!selectNearestEst(ref_est, stamp, max_ekf_align_dt_, ref_est_pos, ekf_dt))
        {
            ++relative_reject_reference_[self_idx];
            return;
        }

        ModelStateSample model;
        double model_dt = -1.0;
        if (!selectNearestModelState(stamp, model, model_dt) ||
            model.pos.size() <= static_cast<size_t>(std::max(self_idx, reference_idx)) ||
            !model.pos[self_idx].allFinite() ||
            !model.pos[reference_idx].allFinite())
        {
            ++relative_reject_model_[self_idx];
            return;
        }

        const Eigen::Vector3d self_offset = initialOffset(drone_names_[self_idx]);
        const Eigen::Vector3d ref_offset = initialOffset(drone_names_[reference_idx]);

        RelativeErrorSample sample;
        sample.timestamp = stamp.toSec();
        sample.ekf_dt = ekf_dt;
        sample.model_dt = model_dt;
        sample.rel_gt = model.pos[self_idx] - model.pos[reference_idx];
        sample.rel_est = (self_offset + self_est_pos) - (ref_offset + ref_est_pos);
        sample.err = sample.rel_gt - sample.rel_est;
        sample.error_norm = sample.err.norm();

        relative_error_sum_[self_idx] += sample.err.squaredNorm();
        sample.cumulative_rmse =
            std::sqrt(relative_error_sum_[self_idx] /
                      static_cast<double>(relative_samples_[self_idx].size() + 1));
        relative_samples_[self_idx].push_back(sample);
    }

    // =====================================================================
    //  指标计算
    // =====================================================================

    /**
     * ATE: 绝对轨迹误差 (RMSE)，单位 m
     * GT 与 EST 都已在数据入口转换为相对初始 ENU 位置.
     */
    static double computeATE(const std::vector<AlignedSample>& data)
    {
        if (data.empty()) return 0.0;

        double sum = 0.0;
        for (const auto& s : data)
        {
            Eigen::Vector3d err = s.est_pos - s.gt_pos;
            sum += err.squaredNorm();
        }
        return std::sqrt(sum / data.size());
    }

    /** 漂移率: ATE / 时长，单位 m/s (即 m/min 除以 60) */
    static double computeDriftRate(const std::vector<AlignedSample>& data)
    {
        if (data.size() < 2) return 0.0;
        double ate = computeATE(data);
        double dur = (data.back().t - data.front().t).toSec();
        return dur > 0.0 ? ate / dur : 0.0;
    }

    /** RPE: 相对平移误差 (relative translation error)，单位 m */
    static double computeRPE(const std::vector<AlignedSample>& data, double delta)
    {
        if (data.size() < 2) return 0.0;

        std::vector<double> errors;
        double window = 0.2 * delta;

        for (size_t i = 0; i < data.size(); i++)
        {
            double target = data[i].t.toSec() + delta;
            double best_dt = 1e9;
            size_t best_j  = i;

            for (size_t j = i + 1; j < data.size(); j++)
            {
                double tj = data[j].t.toSec();
                double dt = std::fabs(tj - target);
                if (dt < best_dt && dt < window)
                {
                    best_dt = dt;
                    best_j  = j;
                }
                if (tj > target + window)
                    break;
            }

            if (best_j > i)
            {
                Eigen::Vector3d est_disp = data[best_j].est_pos - data[i].est_pos;
                Eigen::Vector3d gt_disp  = data[best_j].gt_pos - data[i].gt_pos;
                errors.push_back((est_disp - gt_disp).norm());
            }
        }

        if (errors.empty()) return 0.0;
        double sum_sq = 0.0;
        for (double e : errors) sum_sq += e * e;
        return std::sqrt(sum_sq / errors.size());
    }

    /**
     * 姿态误差 (RMSE)，单位 deg
     * 归一化四元数后计算，避免数值问题导致 NaN
     */
    static double computeAttitudeError(const std::vector<AlignedSample>& data)
    {
        if (data.empty()) return 0.0;
        double sum_sq = 0.0;
        for (const auto& s : data)
        {
            // 归一化四元数，防止数值误差导致 cos_angle 越界
            // ins_estimate 与 Gazebo model_states GT 均按 body -> world/ENU 比较.
            Eigen::Quaterniond qe = s.est_quat.normalized();
            Eigen::Quaterniond qg = s.gt_quat.normalized();

            // e_theta = arccos(0.5 * (tr(R_err) - 1))
            Eigen::Matrix3d R_err = qe.toRotationMatrix()
                                  * qg.toRotationMatrix().transpose();
            double cos_angle = 0.5 * (R_err.trace() - 1.0);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            double angle_rad = std::acos(cos_angle);
            sum_sq += angle_rad * angle_rad;
        }
        return std::sqrt(sum_sq / data.size()) * 180.0 / M_PI;
    }

    /** 速度误差 (RMSE)，单位 m/s */
    static double computeVelocityError(const std::vector<AlignedSample>& data)
    {
        if (data.empty()) return 0.0;
        double sum_sq = 0.0;
        for (const auto& s : data)
            sum_sq += (s.est_vel - s.gt_vel).squaredNorm();
        return std::sqrt(sum_sq / data.size());
    }

    /**
     * NEES: 归一化估计误差平方 (Normalized Estimation Error Squared)
     * NEES = e^T * P^{-1} * e / dim
     * 理想值 ≈ 1.0 (表示协方差与真实误差一致)
     * > 1.0: 协方差偏乐观 (under-confident)
     * < 1.0: 协方差偏保守 (over-confident)
     */
    static double computeNEES_pos(const std::vector<AlignedSample>& data)
    {
        if (data.empty()) return 0.0;

        double sum = 0.0;
        int count = 0;
        for (const auto& s : data)
        {
            Eigen::Vector3d e = s.est_pos - s.gt_pos;
            // 检查协方差是否有效 (正定)
            if (s.est_pos_cov.determinant() < 1e-30)
                continue;
            sum += (e.transpose() * s.est_pos_cov.inverse() * e).value() / 3.0;
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    static double computeNEES_vel(const std::vector<AlignedSample>& data)
    {
        if (data.empty()) return 0.0;
        double sum = 0.0;
        int count = 0;
        for (const auto& s : data)
        {
            Eigen::Vector3d e = s.est_vel - s.gt_vel;
            if (s.est_vel_cov.determinant() < 1e-30)
                continue;
            sum += (e.transpose() * s.est_vel_cov.inverse() * e).value() / 3.0;
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    // =====================================================================
    //  打印输出
    // =====================================================================

    void printMetricsFinal(const std::string& name,
                           const std::vector<AlignedSample>& data) const
    {
        if (data.empty())
        {
            fprintf(stdout, "[INS TEST] %s: 无对齐数据\n", name.c_str());
            fflush(stdout);
            return;
        }

        double duration = (data.back().t - data.front().t).toSec();
        double ate    = computeATE(data);
        double drift  = computeDriftRate(data);
        double rpe_1s = computeRPE(data, 1.0);
        double rpe_5s = computeRPE(data, 5.0);
        double rpe_10s = computeRPE(data, 10.0);
        double att_err = computeAttitudeError(data);
        double vel_err = computeVelocityError(data);
        double nees_p  = computeNEES_pos(data);
        double nees_v  = computeNEES_vel(data);

        fprintf(stdout, "\n============================================\n");
        fprintf(stdout, "  %s  评估结果\n", name.c_str());
        fprintf(stdout, "  采样数: %zu  |  时长: %.1f 秒\n", data.size(), duration);
        fprintf(stdout, "  ATE (RMSE):         %.4f m\n", ate);
        fprintf(stdout, "  漂移率:             %.4f m/s  (%.2f m/min)\n", drift, drift * 60.0);
        fprintf(stdout, "  RPE @ 1s:           %.4f m\n", rpe_1s);
        fprintf(stdout, "  RPE @ 5s:           %.4f m\n", rpe_5s);
        fprintf(stdout, "  RPE @ 10s:          %.4f m\n", rpe_10s);
        fprintf(stdout, "  姿态误差 (RMSE):    %.4f deg\n", att_err);
        fprintf(stdout, "  速度误差 (RMSE):    %.4f m/s\n", vel_err);
        fprintf(stdout, "  NEES pos (理想=1):  %.4f\n", nees_p);
        fprintf(stdout, "  NEES vel (理想=1):  %.4f\n", nees_v);
        fprintf(stdout, "============================================\n");
        fflush(stdout);
    }

    void printFinalMetrics() const
    {
        fprintf(stdout, "\n=============== INS 评估总结 ===============\n");
        for (int d = 0; d < num_drones_; d++)
        {
            auto aligned = alignData(est_data_[d], gt_data_[d], max_model_align_dt_);
            printMetricsFinal(drone_names_[d], aligned);
        }
        printRelativeMetricsFinal();
        fprintf(stdout, "============================================\n");
        fflush(stdout);
    }

    void printRelativeMetricsFinal() const
    {
        const int reference_idx = findDroneIndex(reference_name_);
        if (reference_idx < 0)
        {
            fprintf(stdout, "\n[INS TEST] relative EKF: reference %s not found\n",
                    reference_name_.c_str());
            return;
        }

        fprintf(stdout, "\n=============== EKF 相对 %s 评估 ===============\n",
                reference_name_.c_str());
        for (int d = 0; d < num_drones_; ++d)
        {
            if (d == reference_idx)
                continue;

            const auto& samples = relative_samples_[d];
            if (samples.empty())
            {
                fprintf(stdout,
                        "  %s relative to %s: no valid samples "
                        "(callbacks=%zu, reject_reference=%zu, reject_model=%zu)\n",
                        drone_names_[d].c_str(), reference_name_.c_str(),
                        relative_callbacks_[d],
                        relative_reject_reference_[d],
                        relative_reject_model_[d]);
                continue;
            }

            const double rmse = std::sqrt(relative_error_sum_[d] /
                                          static_cast<double>(samples.size()));

            fprintf(stdout, "  %s relative to %s\n",
                    drone_names_[d].c_str(), reference_name_.c_str());
            fprintf(stdout, "    samples=%zu, RMSE=%.4f m, callbacks=%zu, reject_reference=%zu, reject_model=%zu\n",
                    samples.size(), rmse,
                    relative_callbacks_[d],
                    relative_reject_reference_[d],
                    relative_reject_model_[d]);
        }
    }

    void printTimerCallback(const ros::TimerEvent&)
    {
        for (int d = 0; d < num_drones_; d++)
        {
            size_t n_est = est_data_[d].t.size();
            size_t n_gt  = gt_data_[d].t.size();
            ROS_DEBUG("[INS TEST] %s: 估计 %zu | 真值 %zu",
                      drone_names_[d].c_str(), n_est, n_gt);
        }
    }

    // =====================================================================
    //  CSV 输出: TUM-ish 格式，供 evo 分析
    // =====================================================================

    void writeCSV(const std::string& name,
                  const std::vector<AlignedSample>& data) const
    {
        if (data.empty()) return;

        // 按时间戳排序 (解决 IMU/Flow 回调交叉调用导致的非单调时间戳)
        std::vector<AlignedSample> sorted = data;
        std::sort(sorted.begin(), sorted.end(),
                  [](const AlignedSample& a, const AlignedSample& b) {
                      return a.t.toSec() < b.t.toSec();
                  });

        // 真值轨迹
        std::string gt_file = csv_dir_ + "/" + name + "_gt_traj.csv";
        std::ofstream f_gt(gt_file);
        if (!f_gt.is_open())
        {
            ROS_WARN("[INS TEST] cannot write ground-truth CSV: %s", gt_file.c_str());
            return;
        }
        f_gt << std::fixed << std::setprecision(9);
        double last_t_gt = -1.0;
        for (const auto& s : sorted)
        {
            double t = s.t.toSec();
            if (t == last_t_gt) continue;
            last_t_gt = t;
            f_gt << t << " "
                 << s.gt_pos.x() << " " << s.gt_pos.y() << " " << s.gt_pos.z() << " "
                 << s.gt_quat.x() << " " << s.gt_quat.y() << " "
                 << s.gt_quat.z() << " " << s.gt_quat.w() << "\n";
        }
        f_gt.close();
        ROS_DEBUG("[INS TEST] 真值轨迹已保存: %s (%zu 采样)", gt_file.c_str(), sorted.size());

        // 估计轨迹
        std::string est_file = csv_dir_ + "/" + name + "_est_traj.csv";
        std::ofstream f_est(est_file);
        if (!f_est.is_open())
        {
            ROS_WARN("[INS TEST] cannot write estimate CSV: %s", est_file.c_str());
            return;
        }
        f_est << std::fixed << std::setprecision(9);
        double last_t_est = -1.0;
        for (const auto& s : sorted)
        {
            double t = s.t.toSec();
            if (t == last_t_est) continue;
            last_t_est = t;
            f_est << t << " "
                  << s.est_pos.x() << " " << s.est_pos.y() << " " << s.est_pos.z() << " "
                  << s.est_quat.x() << " " << s.est_quat.y() << " "
                  << s.est_quat.z() << " " << s.est_quat.w() << "\n";
        }
        f_est.close();
        ROS_DEBUG("[INS TEST] 估计轨迹已保存: %s (%zu 采样)", est_file.c_str(), sorted.size());

        std::string vel_file = csv_dir_ + "/" + name + "_vel_error.csv";
        std::ofstream f_vel(vel_file);
        if (!f_vel.is_open())
        {
            ROS_WARN("[INS TEST] cannot write velocity error CSV: %s", vel_file.c_str());
            return;
        }
        f_vel << std::fixed << std::setprecision(9);
        f_vel << "timestamp,est_vx,est_vy,est_vz,gt_vx,gt_vy,gt_vz,"
              << "err_vx,err_vy,err_vz,err_norm\n";
        double last_t_vel = -1.0;
        for (const auto& s : sorted)
        {
            double t = s.t.toSec();
            if (t == last_t_vel) continue;
            last_t_vel = t;
            Eigen::Vector3d err = s.est_vel - s.gt_vel;
            f_vel << t << ","
                  << s.est_vel.x() << "," << s.est_vel.y() << "," << s.est_vel.z() << ","
                  << s.gt_vel.x() << "," << s.gt_vel.y() << "," << s.gt_vel.z() << ","
                  << err.x() << "," << err.y() << "," << err.z() << ","
                  << err.norm() << "\n";
        }
        f_vel.close();
        ROS_DEBUG("[INS TEST] 速度误差已保存: %s (%zu 采样)", vel_file.c_str(), sorted.size());
    }

    void writeRelativeCSV(const std::string& self_name,
                          const std::string& reference_name,
                          const std::vector<RelativeErrorSample>& samples) const
    {
        if (samples.empty())
            return;

        std::vector<RelativeErrorSample> sorted = samples;
        std::sort(sorted.begin(), sorted.end(),
                  [](const RelativeErrorSample& a, const RelativeErrorSample& b) {
                      return a.timestamp < b.timestamp;
                  });

        const std::string path = csv_dir_ + "/" + self_name + "_relative_to_" +
                                 reference_name + "_ekf_error.csv";
        std::ofstream f(path);
        if (!f.is_open())
        {
            ROS_WARN("[INS TEST] cannot write relative EKF error CSV: %s", path.c_str());
            return;
        }

        f << std::fixed << std::setprecision(9);
        f << "timestamp,ekf_dt,model_dt,"
          << "x_gt,y_gt,z_gt,x_est,y_est,z_est,"
          << "err_x,err_y,err_z,error_norm_m,cumulative_rmse_m\n";

        double last_t = -1.0;
        for (const auto& s : sorted)
        {
            if (s.timestamp == last_t)
                continue;
            last_t = s.timestamp;

            f << s.timestamp << ","
              << s.ekf_dt << "," << s.model_dt << ","
              << s.rel_gt.x() << "," << s.rel_gt.y() << "," << s.rel_gt.z() << ","
              << s.rel_est.x() << "," << s.rel_est.y() << "," << s.rel_est.z() << ","
              << s.err.x() << "," << s.err.y() << "," << s.err.z() << ","
              << s.error_norm << "," << s.cumulative_rmse << "\n";
        }
        f.close();

        ROS_INFO("[INS TEST] saved relative EKF error CSV: %s (%zu samples)",
                 path.c_str(), sorted.size());
    }

    void writeAllCSV() const
    {
        for (int d = 0; d < num_drones_; d++)
        {
            auto aligned = alignData(est_data_[d], gt_data_[d], max_model_align_dt_);
            writeCSV(drone_names_[d], aligned);
        }

        const int reference_idx = findDroneIndex(reference_name_);
        if (reference_idx < 0)
            return;

        for (int d = 0; d < num_drones_; ++d)
        {
            if (d == reference_idx)
                continue;

            writeRelativeCSV(drone_names_[d], reference_name_, relative_samples_[d]);
        }
    }
};

// =============================================================================
//  main
// =============================================================================

static volatile bool g_shutdown = false;
void signalHandler(int) { g_shutdown = true; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ins_eskf_test", ros::init_options::NoSigintHandler);
    signal(SIGINT, signalHandler);

    InsEkfTest node;

    while (ros::ok() && !g_shutdown)
        ros::spinOnce();

    fprintf(stdout, "\n=== 收到 Ctrl-C，正在计算评估指标 ===\n");
    fflush(stdout);

    return 0;
}
