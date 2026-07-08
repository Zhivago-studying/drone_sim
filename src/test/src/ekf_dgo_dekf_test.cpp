/**
 * @file  ekf_dgo_dekf_test.cpp
 * @brief DEKF 相对位姿估计评估节点
 *
 * 订阅 /iris_0/dekf/iris_{1,2,3}，与 Gazebo 真值对齐后计算 RMSE，
 * 结果写入 run_data/run_X/dekf/。
 *
 * =========================== 用法 ===========================
 * roslaunch algorithm dgo_full_mission.launch run_id:=X
 *
 * 输出路径:
 *   run_data/run_X/dekf/iris_Y_relative_to_iris_0_dekf_error.csv
 *   run_data/run_X/dekf/dekf_summary.csv
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <vector>

// ── 真值历史采样 ───────────────────────────────────────
struct PoseSample
{
    ros::Time stamp;
    double x = 0.0, y = 0.0, z = 0.0;
};

// ── DEKF 评估节点 ─────────────────────────────────────
class DekfEvaluator
{
public:
    DekfEvaluator()
        : nh_(), pnh_("~")
    {
        pnh_.param("reference_name", reference_name_, std::string("iris_0"));
        pnh_.param("csv_dir", csv_dir_, std::string(""));
        pnh_.param("max_model_align_dt", max_model_align_dt_, 0.03);
        pnh_.param("ignore_after_mission_complete",
                   ignore_after_mission_complete_, true);
        pnh_.param("eval_stop_stage", eval_stop_stage_, 7);
        pnh_.param("count_ignored_callbacks",
                   count_ignored_callbacks_, true);
        pnh_.param("history_keep_time", history_keep_time_, 3.0);

        // 读取 target_names 列表
        XmlRpc::XmlRpcValue target_list;
        if (pnh_.getParam("target_names", target_list) &&
            target_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0; i < target_list.size(); ++i)
            {
                if (target_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                    target_names_.push_back(
                        static_cast<std::string>(target_list[i]));
                }
            }
        }
        if (target_names_.empty())
        {
            target_names_ = {"iris_1", "iris_2", "iris_3"};
        }

        const size_t nt = target_names_.size();
        de_sum_sq_.resize(nt, 0.0);
        de_sample_count_.resize(nt, 0);
        de_callbacks_.resize(nt, 0);
        de_ignored_.resize(nt, 0);
        de_reject_model_.resize(nt, 0);
        de_error_norms_.resize(nt);

        // Gazebo 真值
        gt_history_[reference_name_] = std::deque<PoseSample>();
        for (const auto &name : target_names_)
            gt_history_[name] = std::deque<PoseSample>();

        model_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 100,
            &DekfEvaluator::modelStatesCb, this);

        stage_sub_ = nh_.subscribe<std_msgs::UInt8>(
            "/formation/stage", 1,
            &DekfEvaluator::stageCb, this);

        // DEKF 订阅: /iris_0/dekf/iris_{target}
        for (size_t i = 0; i < nt; ++i)
        {
            const std::string topic =
                "/" + reference_name_ + "/dekf/" + target_names_[i];
            dekf_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic, 10,
                    boost::bind(&DekfEvaluator::dekfCb, this, _1, i)));
        }

	        ensureCsvDir();
	        openErrorCsv();

        ROS_INFO("[DEKF TEST] reference=%s targets=[%s%s%s] "
                 "max_model_align_dt=%.3f "
                 "ignore_after_mission_complete=%d eval_stop_stage=%d",
                 reference_name_.c_str(),
                 target_names_.size() > 0 ? target_names_[0].c_str() : "",
                 target_names_.size() > 1 ? (", " + target_names_[1]).c_str() : "",
                 target_names_.size() > 2 ? (", " + target_names_[2]).c_str() : "",
                 max_model_align_dt_,
                 ignore_after_mission_complete_ ? 1 : 0,
                 eval_stop_stage_);
    }

    ~DekfEvaluator()
    {
        shutdownAndReport();
    }

    // ── 订阅回调 ────────────────────────────────────────

    void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        const ros::Time now = ros::Time::now();
        for (size_t j = 0; j < msg->name.size(); ++j)
        {
            const std::string &name = msg->name[j];
            auto it = gt_history_.find(name);
            if (it == gt_history_.end())
                continue;

            PoseSample s;
            s.stamp = now;
            s.x = msg->pose[j].position.x;
            s.y = msg->pose[j].position.y;
            s.z = msg->pose[j].position.z;

            it->second.push_back(s);

            // 按时间裁剪旧记录
            while (it->second.size() > 2 &&
                   (now - it->second.front().stamp).toSec() > history_keep_time_)
                it->second.pop_front();
        }
    }

    void stageCb(const std_msgs::UInt8::ConstPtr &msg)
    {
        mission_stage_ = msg->data;

        if (ignore_after_mission_complete_ && recording_enabled_ &&
            static_cast<int>(mission_stage_) >= eval_stop_stage_)
        {
            recording_enabled_ = false;
            stopped_by_stage_ = true;
            eval_stop_time_ = ros::Time::now();
            ROS_INFO("[DEKF TEST] mission stage %d >= eval_stop_stage %d; "
                     "recording disabled",
                     static_cast<int>(mission_stage_), eval_stop_stage_);
        }
    }

	    void dekfCb(const nav_msgs::Odometry::ConstPtr &msg, size_t idx)
	    {
	        if (idx >= target_names_.size())
	            return;
	        de_callbacks_[idx]++;

        if (!recording_enabled_)
        {
            if (count_ignored_callbacks_)
                de_ignored_[idx]++;
            return;
        }

        const ros::Time stamp = msg->header.stamp;

        // 找 reference GT
        double rx = 0.0, ry = 0.0, rz = 0.0;
        double dt_ref = 0.0;
        if (!findNearestGt(reference_name_, stamp, max_model_align_dt_,
                           rx, ry, rz, dt_ref))
        {
            de_reject_model_[idx]++;
            return;
        }

        // 找 target GT
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double dt_tgt = 0.0;
        if (!findNearestGt(target_names_[idx], stamp, max_model_align_dt_,
                           tx, ty, tz, dt_tgt))
        {
            de_reject_model_[idx]++;
            return;
        }

        // GT 相对位置 = target - reference
        const double gt_x = tx - rx;
        const double gt_y = ty - ry;
        const double gt_z = tz - rz;

        // DEKF 估计
        const double est_x = msg->pose.pose.position.x;
        const double est_y = msg->pose.pose.position.y;
        const double est_z = msg->pose.pose.position.z;

        const double err_x = est_x - gt_x;
        const double err_y = est_y - gt_y;
        const double err_z = est_z - gt_z;
        const double err_norm = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);

        // 累积
        de_sum_sq_[idx] += err_norm * err_norm;
        de_sample_count_[idx]++;
        de_error_norms_[idx].push_back(err_norm);

        // 读协方差
        const double Pxx = msg->pose.covariance[0];
        const double Pyy = msg->pose.covariance[7];
        const double Pzz = msg->pose.covariance[14];
        const double Pvxvx = msg->twist.covariance[0];
        const double Pvyvy = msg->twist.covariance[7];
        const double Pvzvz = msg->twist.covariance[14];

        // DEKF 速度
        const double vx_est = msg->twist.twist.linear.x;
        const double vy_est = msg->twist.twist.linear.y;
        const double vz_est = msg->twist.twist.linear.z;

	        // 写 CSV
	        if (idx < de_csv_.size() && de_csv_[idx].is_open())
	        {
        const double cumulative_rmse =
            de_sample_count_[idx] > 0
                ? std::sqrt(de_sum_sq_[idx] / de_sample_count_[idx])
                : 0.0;

        const double stamp_f = stamp.toSec();
        de_csv_[idx] << stamp_f << ','
                     << static_cast<int>(mission_stage_) << ','
                     << target_names_[idx] << ','
                     << reference_name_ << ','
                     << gt_x << ',' << gt_y << ',' << gt_z << ','
                     << est_x << ',' << est_y << ',' << est_z << ','
                     << err_x << ',' << err_y << ',' << err_z << ','
                     << err_norm << ','
                     << dt_ref << ',' << dt_tgt << ','
                     << vx_est << ',' << vy_est << ',' << vz_est << ','
                     << Pxx << ',' << Pyy << ',' << Pzz << ','
	                     << Pvxvx << ',' << Pvyvy << ',' << Pvzvz << ','
	                     << cumulative_rmse << '\n';
	        }
	    }

    // ── GT 对齐 ─────────────────────────────────────────

    bool findNearestGt(const std::string &name,
                       const ros::Time &stamp,
                       double max_dt,
                       double &x, double &y, double &z,
                       double &out_dt) const
    {
        auto it = gt_history_.find(name);
        if (it == gt_history_.end() || it->second.empty())
            return false;

        double best_dt = std::numeric_limits<double>::max();
        const PoseSample *best = nullptr;

        for (const auto &s : it->second)
        {
            double dt = std::fabs((s.stamp - stamp).toSec());
            if (dt < best_dt)
            {
                best_dt = dt;
                best = &s;
            }
        }

        if (!best || best_dt > max_dt)
            return false;

        x = best->x;
        y = best->y;
        z = best->z;
        out_dt = best_dt;
        return true;
    }

    // ── 输出 ────────────────────────────────────────────

    void ensureCsvDir()
    {
        if (csv_dir_.empty())
        {
            csv_dir_ = "/tmp/dekf_test";
        }
        boost::filesystem::create_directories(csv_dir_);
    }

    void openErrorCsv()
    {
        de_csv_.resize(target_names_.size());
        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            const std::string path = csv_dir_ + "/" +
                                     target_names_[i] + "_relative_to_" +
                                     reference_name_ + "_dekf_error.csv";
            de_csv_[i].open(path.c_str(), std::ios::out | std::ios::trunc);
            if (!de_csv_[i].is_open())
            {
                ROS_WARN("[DEKF TEST] cannot open CSV: %s", path.c_str());
                continue;
            }
            de_csv_[i] << std::fixed << std::setprecision(9);
            de_csv_[i] << "timestamp,mission_stage,"
                       << "target,reference,"
                       << "x_gt,y_gt,z_gt,"
                       << "x_est,y_est,z_est,"
                       << "err_x,err_y,err_z,error_norm_m,"
                       << "gt_align_dt_ref,gt_align_dt_target,"
                       << "vx_est,vy_est,vz_est,"
                       << "Pxx,Pyy,Pzz,"
                       << "Pvxvx,Pvyvy,Pvzvz,"
                       << "cumulative_rmse_m\n";
        }
    }

    void shutdownAndReport()
    {
        if (reported_)
            return;
        reported_ = true;

        writeSummaryCsv();

        fprintf(stdout,
                "\n=============== DEKF 相对 %s 评估 ===============\n",
                reference_name_.c_str());

        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            const size_t n = de_sample_count_[i];
            const double rmse = n > 0 ? std::sqrt(de_sum_sq_[i] / n) : 0.0;
            const double mean = n > 0
                ? std::accumulate(de_error_norms_[i].begin(),
                                  de_error_norms_[i].end(), 0.0) / n
                : 0.0;
            double p95 = 0.0;
            if (!de_error_norms_[i].empty())
            {
                std::vector<double> sorted = de_error_norms_[i];
                std::sort(sorted.begin(), sorted.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * sorted.size())) - 1;
                p95 = sorted[std::min(idx, sorted.size() - 1)];
            }
            const double mx = n > 0
                ? *std::max_element(de_error_norms_[i].begin(),
                                    de_error_norms_[i].end())
                : 0.0;

            fprintf(stdout,
                    "  %s relative to %s\n"
                    "    samples=%lu\n"
                    "    RMSE=%.3f m  mean=%.3f m  p95=%.3f m  max=%.3f m\n"
                    "    callbacks=%lu  reject_model=%lu  ignored_after_stop=%lu\n",
                    target_names_[i].c_str(), reference_name_.c_str(),
                    static_cast<unsigned long>(n),
                    rmse, mean, p95, mx,
                    static_cast<unsigned long>(de_callbacks_[i]),
                    static_cast<unsigned long>(de_reject_model_[i]),
                    static_cast<unsigned long>(de_ignored_[i]));
        }
        fprintf(stdout,
                "===================================================\n\n");
    }

    void writeSummaryCsv()
    {
        const std::string path = csv_dir_ + "/dekf_summary.csv";
        std::ofstream csv(path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv.is_open())
        {
            ROS_WARN("[DEKF TEST] cannot open summary CSV: %s", path.c_str());
            return;
        }
        csv << std::fixed << std::setprecision(9);
        csv << "target,reference,samples,rmse_m,mean_error_m,"
            << "p95_error_m,max_error_m,"
            << "callbacks,reject_model,ignored_after_stop,"
            << "stopped_by_stage,eval_stop_time\n";

        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            const size_t n = de_sample_count_[i];
            const double rmse = n > 0 ? std::sqrt(de_sum_sq_[i] / n) : 0.0;
            const double mean = n > 0
                ? std::accumulate(de_error_norms_[i].begin(),
                                  de_error_norms_[i].end(), 0.0) / n
                : 0.0;
            double p95 = 0.0;
            if (!de_error_norms_[i].empty())
            {
                std::vector<double> sorted = de_error_norms_[i];
                std::sort(sorted.begin(), sorted.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * sorted.size())) - 1;
                p95 = sorted[std::min(idx, sorted.size() - 1)];
            }
            const double mx = n > 0
                ? *std::max_element(de_error_norms_[i].begin(),
                                    de_error_norms_[i].end())
                : 0.0;

            csv << target_names_[i] << ','
                << reference_name_ << ','
                << n << ','
                << rmse << ','
                << mean << ','
                << p95 << ','
                << mx << ','
                << de_callbacks_[i] << ','
                << de_reject_model_[i] << ','
                << de_ignored_[i] << ','
                << (stopped_by_stage_ ? 1 : 0) << ','
                << (eval_stop_time_.isZero() ? 0.0 : eval_stop_time_.toSec())
                << '\n';
        }
        ROS_INFO("[DEKF TEST] summary CSV: %s", path.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string reference_name_;
    std::vector<std::string> target_names_;
    std::string csv_dir_;

	    double max_model_align_dt_ = 0.03;
	    double history_keep_time_ = 3.0;

    bool ignore_after_mission_complete_ = true;
    bool count_ignored_callbacks_ = true;
    int eval_stop_stage_ = 7;

    // GT 缓存
    std::map<std::string, std::deque<PoseSample>> gt_history_;

    // 订阅
    ros::Subscriber model_sub_;
    ros::Subscriber stage_sub_;
    std::vector<ros::Subscriber> dekf_subs_;

    // 阶段管理
    uint8_t mission_stage_ = 0;
    bool recording_enabled_ = true;
    bool stopped_by_stage_ = false;
    ros::Time eval_stop_time_;

    // 累积统计
    std::vector<double> de_sum_sq_;
    std::vector<size_t> de_sample_count_;
    std::vector<size_t> de_callbacks_;
    std::vector<size_t> de_ignored_;
    std::vector<size_t> de_reject_model_;
    std::vector<std::vector<double>> de_error_norms_;

    // CSV
    std::vector<std::ofstream> de_csv_;
    bool reported_ = false;
};

// ── main ──────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_dgo_dekf_test");
    DekfEvaluator eval;
    ros::spin();
    return 0;
}
