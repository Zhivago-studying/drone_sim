/**
 * INS ESKF 评估节点
 *
 * 订阅:
 *   /<drone>/mavros/local_position/odom  — 地面真值 (SITL, 带 header.stamp)
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
#include <fstream>
#include <iomanip>
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
        ensureOutputDir();

        // 初始化数据缓冲区
        gt_data_.resize(num_drones_);
        est_data_.resize(num_drones_);

        // 订阅每架无人机的真值 (odom) 和估计值 (ins_estimate)
        // 使用 /mavros/local_position/odom 作为真值:
        //   优点: 有 header.stamp 与 INS 时间戳直接对齐
        //   在 SITL 下 odom 数据来自 Gazebo 真值
        for (int i = 0; i < num_drones_; i++)
        {
            std::string gt_topic  = "/" + drone_names_[i] + "/mavros/local_position/odom";
            std::string est_topic = "/" + drone_names_[i] + "/ins_estimate";

            gt_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(gt_topic, 2000,
                    boost::bind(&InsEkfTest::gtCallback, this, _1, i)));
            ins_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(est_topic, 2000,
                    boost::bind(&InsEkfTest::insCallback, this, _1, i)));
        }

        // 定时输出进度信息 (每 10 秒)
        print_timer_ = nh_.createTimer(ros::Duration(10.0),
                                        &InsEkfTest::printTimerCallback, this);

        ROS_INFO("[INS TEST] 评估节点已启动，监控 %d 架无人机", num_drones_);
        for (const auto& name : drone_names_)
            ROS_INFO("  无人机: %s  (真值=<odom>)", name.c_str());
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

    // =====================================================================
    //  成员变量
    // =====================================================================

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> gt_subs_;
    std::vector<ros::Subscriber> ins_subs_;
    ros::Timer print_timer_;

    int num_drones_;
    std::vector<std::string> drone_names_;
    std::string csv_dir_;
    std::vector<TimeSeries> gt_data_;
    std::vector<TimeSeries> est_data_;

    void ensureOutputDir() const
    {
        if (csv_dir_.empty() || csv_dir_ == ".")
            return;

        struct stat st;
        if (stat(csv_dir_.c_str(), &st) == 0)
        {
            if (S_ISDIR(st.st_mode))
                return;
            ROS_WARN("[INS TEST] csv_dir exists but is not a directory: %s", csv_dir_.c_str());
            return;
        }

        if (mkdir(csv_dir_.c_str(), 0755) != 0 && errno != EEXIST)
        {
            ROS_WARN("[INS TEST] failed to create csv_dir: %s", csv_dir_.c_str());
        }
    }

    // =====================================================================
    //  地面真值回调 (odom) —— 使用 header.stamp 避免时间偏差
    // =====================================================================

    void gtCallback(const nav_msgs::Odometry::ConstPtr& msg, int drone_id)
    {
        TimeSeries& gt = gt_data_[drone_id];
        Eigen::Vector3d pos(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
        if (!gt.has_pos_origin)
        {
            gt.pos_origin = pos;
            gt.has_pos_origin = true;
        }

        gt.t.push_back(msg->header.stamp);  // ✓ SITL 仿真时间
        gt.pos.push_back(pos - gt.pos_origin);
        gt.vel.emplace_back(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z);
        gt.quat.emplace_back(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
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
            // ins_estimate 与 MAVROS GT 均为 body -> ENU.
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
            auto aligned = alignData(est_data_[d], gt_data_[d]);
            printMetricsFinal(drone_names_[d], aligned);
        }
        fprintf(stdout, "============================================\n");
        fflush(stdout);
    }

    void printTimerCallback(const ros::TimerEvent&)
    {
        for (int d = 0; d < num_drones_; d++)
        {
            size_t n_est = est_data_[d].t.size();
            size_t n_gt  = gt_data_[d].t.size();
            ROS_INFO("[INS TEST] %s: 估计 %zu | 真值 %zu",
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
        ROS_INFO("[INS TEST] 真值轨迹已保存: %s (%zu 采样)", gt_file.c_str(), sorted.size());

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
        ROS_INFO("[INS TEST] 估计轨迹已保存: %s (%zu 采样)", est_file.c_str(), sorted.size());
    }

    void writeAllCSV() const
    {
        for (int d = 0; d < num_drones_; d++)
        {
            auto aligned = alignData(est_data_[d], gt_data_[d]);
            writeCSV(drone_names_[d], aligned);
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
