/**
 * @file  ekf_dgo_dekf_test.cpp
 * @brief DEKF 相对位姿估计评估节点 (v2 — 新增双向对称评价)
 *
 * 保留:
 *   anchor-based: /iris_0/dekf/iris_{1,2,3} → iris_Y relative to iris_0
 *
 * 新增:
 *   bidirectional: 订阅全部 12 个方向 /iris_i/dekf/iris_j (i≠j)
 *   双向时间同步后计算 symmetric RMSE + consistency residual
 *
 * 输出路径:
 *   run_data/run_X/dekf/iris_Y_relative_to_iris_0_dekf_error.csv
 *   run_data/run_X/dekf/dekf_summary.csv
 *   run_data/run_X/dekf/pairwise/iris_i_iris_j_bidirectional_dekf_error.csv
 *   run_data/run_X/dekf/pairwise/dekf_pairwise_summary.csv
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

// ── 真值历史采样 ───────────────────────────────────────
struct PoseSample
{
    ros::Time stamp;
    double x = 0.0, y = 0.0, z = 0.0;
};

// ── DEKF 采样 ─────────────────────────────────────────
struct DekfSample
{
    ros::Time stamp;
    double x = 0.0, y = 0.0, z = 0.0;
    int mission_stage = 0;
    bool used = false;
};

// ── pairwise 统计 ──────────────────────────────────────
struct PairStats
{
    size_t samples = 0;
    double sum_sq_sym = 0.0;
    double sum_sq_consistency = 0.0;
    double sum_sq_a = 0.0;
    double sum_sq_b = 0.0;
    std::vector<double> err_sym_norms;
    std::vector<double> consistency_norms;
    std::vector<double> dt_pairs;
};

// ── DEKF 评估节点 ─────────────────────────────────────
class DekfEvaluator
{
public:
    DekfEvaluator()
        : nh_(), pnh_("~")
    {
        // === legacy anchor params ===
        pnh_.param("reference_name", reference_name_, std::string("iris_0"));
        pnh_.param("csv_dir", csv_dir_, std::string(""));
        pnh_.param("max_model_align_dt", max_model_align_dt_, 0.05);
        pnh_.param("ignore_after_mission_complete",
                   ignore_after_mission_complete_, true);
        pnh_.param("eval_stop_stage", eval_stop_stage_, 7);
        pnh_.param("count_ignored_callbacks",
                   count_ignored_callbacks_, true);
        pnh_.param("history_keep_time", history_keep_time_, 3.0);

        // === pairwise params ===
        pnh_.param("enable_pairwise_eval", enable_pairwise_eval_, true);
        pnh_.param("pairwise_max_sync_dt", pairwise_max_sync_dt_, 0.05);

        // 读取 target_names 列表
        XmlRpc::XmlRpcValue target_list;
        if (pnh_.getParam("target_names", target_list) &&
            target_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0; i < target_list.size(); ++i)
            {
                if (target_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
                    target_names_.push_back(
                        static_cast<std::string>(target_list[i]));
            }
        }
        if (target_names_.empty())
            target_names_ = {"iris_1", "iris_2", "iris_3"};

        // legacy: 所有被监测 UAV 名称 (ref + targets)
        all_iris_.push_back(reference_name_);
        for (const auto &t : target_names_)
            if (std::find(all_iris_.begin(), all_iris_.end(), t) == all_iris_.end())
                all_iris_.push_back(t);

        const size_t nt = target_names_.size();
        const size_t na = all_iris_.size();

        de_sum_sq_.resize(nt, 0.0);
        de_sample_count_.resize(nt, 0);
        de_callbacks_.resize(nt, 0);
        de_ignored_.resize(nt, 0);
        de_reject_model_.resize(nt, 0);
        de_error_norms_.resize(nt);

        // GT 缓存
        for (const auto &name : all_iris_)
            gt_history_[name] = std::deque<PoseSample>();

        model_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 100, &DekfEvaluator::modelStatesCb, this);
        stage_sub_ = nh_.subscribe<std_msgs::UInt8>(
            "/formation/stage", 1, &DekfEvaluator::stageCb, this);

        // === legacy DEKF 订阅: /iris_ref/dekf/iris_target ===
        for (size_t i = 0; i < nt; ++i)
        {
            const std::string topic = "/" + reference_name_ +
                                      "/dekf/" + target_names_[i];
            dekf_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    topic, 10,
                    boost::bind(&DekfEvaluator::dekfCb, this, _1, i)));
        }

	        // === pairwise DEKF 订阅: 全部 i≠j ===
	        if (enable_pairwise_eval_)
	        {
	            for (size_t i = 0; i < na; ++i)
	            {
	                for (size_t j = 0; j < na; ++j)
	                {
	                    if (i == j)
	                        continue;
	                    const std::string topic = "/" + all_iris_[i] +
	                                              "/dekf/" + all_iris_[j];
	                    auto key = std::make_pair(i, j);
	                    pairwise_subs_.push_back(
	                        nh_.subscribe<nav_msgs::Odometry>(
	                            topic, 10,
	                            boost::bind(&DekfEvaluator::pairwiseDekfCb,
	                                        this, _1, i, j)));
	                    dekf_buffers_[key] = std::deque<DekfSample>();
	                }
	            }

		            // 初始化 pairwise 统计 (只评价涉及 reference 的 pair)
		            for (size_t i = 0; i < na; ++i)
		            {
		                for (size_t j = i + 1; j < na; ++j)
		                {
		                    if (all_iris_[i] != reference_name_ &&
		                        all_iris_[j] != reference_name_)
		                        continue;
		                    pair_keys_.push_back(std::make_pair(i, j));
		                    pair_stats_[std::make_pair(i, j)] = PairStats();
		                }
		            }
	        }

	        ensureCsvDir();
	        openErrorCsv();

	        if (enable_pairwise_eval_)
	        {
	            ensurePairwiseCsvDir();
	            openPairwiseCsv();
	        }

        ROS_INFO("[DEKF TEST] legacy reference=%s targets=[%s%s%s] "
                 "pairwise=%d max_sync_dt=%.3f",
                 reference_name_.c_str(),
                 target_names_.size() > 0 ? target_names_[0].c_str() : "",
                 target_names_.size() > 1 ? (", " + target_names_[1]).c_str() : "",
                 target_names_.size() > 2 ? (", " + target_names_[2]).c_str() : "",
                 enable_pairwise_eval_ ? 1 : 0, pairwise_max_sync_dt_);
    }

    ~DekfEvaluator()
    {
        shutdownAndReport();
    }

    // ── 回调 ────────────────────────────────────────────

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

    // legacy anchor callback
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

        double rx = 0.0, ry = 0.0, rz = 0.0;
        double dt_ref = 0.0;
        if (!findNearestGt(reference_name_, stamp, max_model_align_dt_,
                           rx, ry, rz, dt_ref))
        {
            de_reject_model_[idx]++;
            return;
        }

        double tx = 0.0, ty = 0.0, tz = 0.0;
        double dt_tgt = 0.0;
        if (!findNearestGt(target_names_[idx], stamp, max_model_align_dt_,
                           tx, ty, tz, dt_tgt))
        {
            de_reject_model_[idx]++;
            return;
        }

        const double gt_x = tx - rx;
        const double gt_y = ty - ry;
        const double gt_z = tz - rz;

        const double est_x = msg->pose.pose.position.x;
        const double est_y = msg->pose.pose.position.y;
        const double est_z = msg->pose.pose.position.z;

        const double err_x = est_x - gt_x;
        const double err_y = est_y - gt_y;
        const double err_z = est_z - gt_z;
        const double err_norm = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);

        de_sum_sq_[idx] += err_norm * err_norm;
        de_sample_count_[idx]++;
        de_error_norms_[idx].push_back(err_norm);

        const double Pxx = msg->pose.covariance[0];
        const double Pyy = msg->pose.covariance[7];
        const double Pzz = msg->pose.covariance[14];
        const double Pvxvx = msg->twist.covariance[0];
        const double Pvyvy = msg->twist.covariance[7];
        const double Pvzvz = msg->twist.covariance[14];
        const double vx_est = msg->twist.twist.linear.x;
        const double vy_est = msg->twist.twist.linear.y;
        const double vz_est = msg->twist.twist.linear.z;

        if (idx < de_csv_.size() && de_csv_[idx].is_open())
        {
            const double cumulative_rmse =
                de_sample_count_[idx] > 0
                    ? std::sqrt(de_sum_sq_[idx] / de_sample_count_[idx])
                    : 0.0;
            de_csv_[idx] << stamp.toSec() << ','
                         << static_cast<int>(mission_stage_) << ','
                         << target_names_[idx] << ',' << reference_name_ << ','
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

    // pairwise DEKF 回调: 缓存所有方向
    void pairwiseDekfCb(const nav_msgs::Odometry::ConstPtr &msg, int i, int j)
    {
        if (!enable_pairwise_eval_ || !recording_enabled_)
            return;

        DekfSample s;
        s.stamp = msg->header.stamp;
        s.x = msg->pose.pose.position.x;
        s.y = msg->pose.pose.position.y;
        s.z = msg->pose.pose.position.z;
        s.mission_stage = static_cast<int>(mission_stage_);

        auto key = std::make_pair(i, j);
        auto it = dekf_buffers_.find(key);
        if (it == dekf_buffers_.end())
            return;

        it->second.push_back(s);
        while (it->second.size() > 2 &&
               (s.stamp - it->second.front().stamp).toSec() > 2.0)
            it->second.pop_front();

        // 触发双向配对评价
        tryPairwiseMatch(i, j);
    }

    // ── 双向配对 ────────────────────────────────────────

	void tryPairwiseMatch(int i, int j)
	{
	    // canonicalize: a = min(i,j), b = max(i,j)，方向固定 a→b
	    const int a = std::min(i, j);
	    const int b = std::max(i, j);

	    PairKey key_ab(a, b);
	    PairKey key_ba(b, a);

	    // 当前回调方向: (i,j)
	    PairKey key_now = std::make_pair(i, j);
	    PairKey key_rev = std::make_pair(j, i);

	    auto it_now = dekf_buffers_.find(key_now);
	    auto it_rev = dekf_buffers_.find(key_rev);
	    if (it_now == dekf_buffers_.end() || it_rev == dekf_buffers_.end())
	        return;
	    if (it_now->second.empty() || it_rev->second.empty())
	        return;

	    DekfSample &s_now = it_now->second.back();
	    if (s_now.used)
	        return;

	    // 找反方向最近邻（跳过已使用样本）
	    double best_dt = std::numeric_limits<double>::max();
	    DekfSample *best = nullptr;
	    for (auto &s : it_rev->second)
	    {
	        if (s.used)
	            continue;
	        double dt = std::fabs((s.stamp - s_now.stamp).toSec());
	        if (dt < best_dt) { best_dt = dt; best = &s; }
	    }
	    if (!best || best_dt > pairwise_max_sync_dt_)
	        return;

	    DekfSample &s_rev = *best;

	    // 提取固定方向样本: s_ab = /iris_a/dekf/iris_b, s_ba = /iris_b/dekf/iris_a
	    DekfSample *s_ab_ptr = nullptr;
	    DekfSample *s_ba_ptr = nullptr;

	    if (i == a && j == b) { s_ab_ptr = &s_now; s_ba_ptr = &s_rev; }
	    else                  { s_ab_ptr = &s_rev; s_ba_ptr = &s_now; }

	    DekfSample &s_ab = *s_ab_ptr;
	    DekfSample &s_ba = *s_ba_ptr;

	    // pair 中心时间 (用于 GT 对齐)
	    const ros::Time pair_stamp(
	        (s_ab.stamp.toSec() + s_ba.stamp.toSec()) * 0.5);

	    // 找 GT
	    double gx_a = 0, gy_a = 0, gz_a = 0;
	    double gx_b = 0, gy_b = 0, gz_b = 0;
	    double d1 = 0, d2 = 0;
	    if (!findNearestGt(all_iris_[a], pair_stamp, max_model_align_dt_,
	                       gx_a, gy_a, gz_a, d1))
	        return;
	    if (!findNearestGt(all_iris_[b], pair_stamp, max_model_align_dt_,
	                       gx_b, gy_b, gz_b, d2))
	        return;

	    // GT 相对位置: a→b
	    const double gt_x = gx_b - gx_a;
	    const double gt_y = gy_b - gy_a;
	    const double gt_z = gz_b - gz_a;

	    // r_ab = /iris_a/dekf/iris_b → p_b - p_a
	    // r_ba = /iris_b/dekf/iris_a → p_a - p_b → -r_ba = p_b - p_a
	    const double r_ab_x = s_ab.x, r_ab_y = s_ab.y, r_ab_z = s_ab.z;
	    const double r_ba_neg_x = -s_ba.x, r_ba_neg_y = -s_ba.y, r_ba_neg_z = -s_ba.z;

	    // 对称平均
	    const double rs_x = 0.5 * (r_ab_x + r_ba_neg_x);
	    const double rs_y = 0.5 * (r_ab_y + r_ba_neg_y);
	    const double rs_z = 0.5 * (r_ab_z + r_ba_neg_z);

	    // 一致性残差: c_ab = r_ab + r_ba
	    const double c_x = r_ab_x + s_ba.x;
	    const double c_y = r_ab_y + s_ba.y;
	    const double c_z = r_ab_z + s_ba.z;

	    // 单向误差 (统一 a→b 方向)
	    const double err_a_x = r_ab_x - gt_x, err_a_y = r_ab_y - gt_y, err_a_z = r_ab_z - gt_z;
	    const double err_b_x = r_ba_neg_x - gt_x, err_b_y = r_ba_neg_y - gt_y, err_b_z = r_ba_neg_z - gt_z;
	    const double err_s_x = rs_x - gt_x, err_s_y = rs_y - gt_y, err_s_z = rs_z - gt_z;

	    const double err_a_n = std::sqrt(err_a_x*err_a_x + err_a_y*err_a_y + err_a_z*err_a_z);
	    const double err_b_n = std::sqrt(err_b_x*err_b_x + err_b_y*err_b_y + err_b_z*err_b_z);
	    const double err_s_n = std::sqrt(err_s_x*err_s_x + err_s_y*err_s_y + err_s_z*err_s_z);
	    const double c_n = std::sqrt(c_x*c_x + c_y*c_y + c_z*c_z);

	    // 累计统计
	    auto &ps = pair_stats_[key_ab];
	    ps.samples++;
	    ps.sum_sq_sym += err_s_n * err_s_n;
	    ps.sum_sq_consistency += c_n * c_n;
	    ps.sum_sq_a += err_a_n * err_a_n;
	    ps.sum_sq_b += err_b_n * err_b_n;
	    ps.err_sym_norms.push_back(err_s_n);
	    ps.consistency_norms.push_back(c_n);
	    ps.dt_pairs.push_back(best_dt);

	    // 写 CSV
	    auto csv_it = pair_csv_.find(key_ab);
	    if (csv_it != pair_csv_.end() && csv_it->second.is_open())
	    {
	        csv_it->second << pair_stamp.toSec() << ','
	                      << std::max(s_ab.mission_stage, s_ba.mission_stage) << ','
	                      << best_dt << ','
	                      << gt_x << ',' << gt_y << ',' << gt_z << ','
	                      << r_ab_x << ',' << r_ab_y << ',' << r_ab_z << ','
	                      << r_ba_neg_x << ',' << r_ba_neg_y << ',' << r_ba_neg_z << ','
	                      << rs_x << ',' << rs_y << ',' << rs_z << ','
	                      << err_s_x << ',' << err_s_y << ',' << err_s_z << ','
	                      << err_s_n << ','
	                      << c_x << ',' << c_y << ',' << c_z << ','
	                      << c_n << ','
	                      << err_a_n << ',' << err_b_n << '\n';
	    }

	    // 标记双方已使用 → 严格一对一配对
	    s_ab.used = true;
	    s_ba.used = true;
	}

    // ── GT 对齐 ─────────────────────────────────────────

    bool findNearestGt(const std::string &name,
                       const ros::Time &stamp, double max_dt,
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
            if (dt < best_dt) { best_dt = dt; best = &s; }
        }
        if (!best || best_dt > max_dt)
            return false;
        x = best->x; y = best->y; z = best->z; out_dt = best_dt;
        return true;
    }

    // ── CSV 输出 ────────────────────────────────────────

    void ensureCsvDir()
    {
        if (csv_dir_.empty()) csv_dir_ = "/tmp/dekf_test";
        boost::filesystem::create_directories(csv_dir_);
    }

    void openErrorCsv()
    {
        de_csv_.resize(target_names_.size());
        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            std::string path = csv_dir_ + "/" + target_names_[i]
                              + "_relative_to_" + reference_name_
                              + "_dekf_error.csv";
            de_csv_[i].open(path.c_str(), std::ios::out | std::ios::trunc);
            if (!de_csv_[i].is_open()) { ROS_WARN("[DEKF TEST] cannot open %s", path.c_str()); continue; }
            de_csv_[i] << std::fixed << std::setprecision(9);
            de_csv_[i] << "timestamp,mission_stage,target,reference,"
                       << "x_gt,y_gt,z_gt,x_est,y_est,z_est,"
                       << "err_x,err_y,err_z,error_norm_m,"
                       << "gt_align_dt_ref,gt_align_dt_target,"
                       << "vx_est,vy_est,vz_est,"
                       << "Pxx,Pyy,Pzz,Pvxvx,Pvyvy,Pvzvz,cumulative_rmse_m\n";
        }
    }

    void ensurePairwiseCsvDir()
    {
        pairwise_dir_ = csv_dir_ + "/pairwise";
        boost::filesystem::create_directories(pairwise_dir_);
    }

    void openPairwiseCsv()
    {
        for (const auto &pk : pair_keys_)
        {
            const std::string aname = all_iris_[pk.first];
            const std::string bname = all_iris_[pk.second];
            std::string path = pairwise_dir_ + "/" + aname + "_" + bname
                              + "_bidirectional_dekf_error.csv";
            auto &csv = pair_csv_[pk];
            csv.open(path.c_str(), std::ios::out | std::ios::trunc);
            if (!csv.is_open()) { ROS_WARN("[DEKF TEST] cannot open %s", path.c_str()); continue; }
            csv << std::fixed << std::setprecision(9);
            csv << "timestamp,mission_stage,dt_pair,"
                << "gt_x,gt_y,gt_z,"
                << "r_ij_x,r_ij_y,r_ij_z,"
                << "r_ji_neg_x,r_ji_neg_y,r_ji_neg_z,"
                << "r_sym_x,r_sym_y,r_sym_z,"
                << "err_sym_x,err_sym_y,err_sym_z,"
                << "err_sym_norm,"
                << "consistency_x,consistency_y,consistency_z,"
                << "consistency_norm,"
                << "err_ij_norm,err_ji_neg_norm\n";
        }
    }

    void shutdownAndReport()
    {
        if (reported_) return;
        reported_ = true;

        writeSummaryCsv();
        writePairwiseSummaryCsv();

        // legacy 输出
        fprintf(stdout, "\n=============== DEKF 相对 %s 评估 ===============\n", reference_name_.c_str());
        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            size_t n = de_sample_count_[i];
            double rmse = n > 0 ? std::sqrt(de_sum_sq_[i] / n) : 0.0;
            double mean = n > 0
                ? std::accumulate(de_error_norms_[i].begin(), de_error_norms_[i].end(), 0.0) / n
                : 0.0;
            double p95 = 0.0, mx = 0.0;
            if (!de_error_norms_[i].empty())
            {
                auto s = de_error_norms_[i];
                std::sort(s.begin(), s.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * s.size())) - 1;
                p95 = s[std::min(idx, s.size() - 1)];
                mx = *std::max_element(s.begin(), s.end());
            }
            fprintf(stdout, "  %s relative to %s: samples=%lu RMSE=%.3f m  mean=%.3f m  p95=%.3f m  max=%.3f m\n",
                    target_names_[i].c_str(), reference_name_.c_str(),
                    static_cast<unsigned long>(n), rmse, mean, p95, mx);
        }
        fprintf(stdout, "===================================================\n\n");

        // pairwise 输出
        if (!enable_pairwise_eval_) return;
        fprintf(stdout, "=============== DEKF 双向对称评价 ===============\n");
        for (const auto &pk : pair_keys_)
        {
            const auto &ps = pair_stats_[pk];
            size_t n = ps.samples;
            if (n == 0) { fprintf(stdout, "  %s <-> %s: no samples\n", all_iris_[pk.first].c_str(), all_iris_[pk.second].c_str()); continue; }

            auto calc = [](const std::vector<double> &v) -> std::tuple<double,double,double,double> {
                size_t nn = v.size();
                double sum_sq = 0.0, sum = 0.0;
                for (auto d : v) { sum_sq += d*d; sum += d; }
                double rmse = std::sqrt(sum_sq / nn);
                double mean = sum / nn;
                auto s = v;
                std::sort(s.begin(), s.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * s.size())) - 1;
                double p95 = s[std::min(idx, s.size() - 1)];
                double mx = *std::max_element(s.begin(), s.end());
                return {rmse, mean, p95, mx};
            };

            auto [sr, sm, sp95, smx] = calc(ps.err_sym_norms);
            auto [cr, cm, cp95, cmx] = calc(ps.consistency_norms);
            double ar = std::sqrt(ps.sum_sq_a / n);
            double br = std::sqrt(ps.sum_sq_b / n);
            double mean_dt = std::accumulate(ps.dt_pairs.begin(), ps.dt_pairs.end(), 0.0) / n;

            fprintf(stdout, "  %s <-> %s\n"
                    "    samples=%lu  mean_pair_dt=%.4f s\n"
                    "    sym_RMSE=%.3f m  sym_mean=%.3f m  sym_p95=%.3f m  sym_max=%.3f m\n"
                    "    one_way %s->%s RMSE=%.3f m, %s->%s RMSE=%.3f m\n"
                    "    consistency RMSE=%.3f m  p95=%.3f m  max=%.3f m\n",
                    all_iris_[pk.first].c_str(), all_iris_[pk.second].c_str(),
                    static_cast<unsigned long>(n), mean_dt,
                    sr, sm, sp95, smx,
                    all_iris_[pk.first].c_str(), all_iris_[pk.second].c_str(), ar,
                    all_iris_[pk.second].c_str(), all_iris_[pk.first].c_str(), br,
                    cr, cp95, cmx);
        }
        fprintf(stdout, "===================================================\n\n");
    }

    void writeSummaryCsv()
    {
        std::string path = csv_dir_ + "/dekf_summary.csv";
        std::ofstream csv(path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv.is_open()) { ROS_WARN("[DEKF TEST] cannot open summary CSV: %s", path.c_str()); return; }
        csv << std::fixed << std::setprecision(9);
        csv << "target,reference,samples,rmse_m,mean_error_m,p95_error_m,max_error_m,"
            << "callbacks,reject_model,ignored_after_stop,stopped_by_stage,eval_stop_time\n";
        for (size_t i = 0; i < target_names_.size(); ++i)
        {
            size_t n = de_sample_count_[i];
            double rmse = n > 0 ? std::sqrt(de_sum_sq_[i] / n) : 0.0;
            double mean = n > 0
                ? std::accumulate(de_error_norms_[i].begin(), de_error_norms_[i].end(), 0.0) / n
                : 0.0;
            double p95 = 0.0, mx = 0.0;
            if (!de_error_norms_[i].empty())
            {
                auto s = de_error_norms_[i];
                std::sort(s.begin(), s.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * s.size())) - 1;
                p95 = s[std::min(idx, s.size() - 1)];
                mx = *std::max_element(s.begin(), s.end());
            }
            csv << target_names_[i] << ',' << reference_name_ << ','
                << n << ',' << rmse << ',' << mean << ',' << p95 << ',' << mx << ','
                << de_callbacks_[i] << ',' << de_reject_model_[i] << ',' << de_ignored_[i] << ','
                << (stopped_by_stage_ ? 1 : 0) << ','
                << (eval_stop_time_.isZero() ? 0.0 : eval_stop_time_.toSec()) << '\n';
        }
        ROS_INFO("[DEKF TEST] summary CSV: %s", path.c_str());
    }

    void writePairwiseSummaryCsv()
    {
        if (!enable_pairwise_eval_) return;
        std::string path = pairwise_dir_ + "/dekf_pairwise_summary.csv";
        std::ofstream csv(path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv.is_open()) { ROS_WARN("[DEKF TEST] cannot open pairwise summary: %s", path.c_str()); return; }
        csv << std::fixed << std::setprecision(9);
        csv << "pair_a,pair_b,samples,mean_pair_dt,"
            << "sym_rmse,sym_mean,sym_p95,sym_max,"
            << "one_way_a_rmse,one_way_b_rmse,"
            << "consistency_rmse,consistency_p95,consistency_max\n";
        for (const auto &pk : pair_keys_)
        {
            const auto &ps = pair_stats_[pk];
            size_t n = ps.samples;
            if (n == 0) continue;
            auto calc = [](const std::vector<double> &v) -> std::tuple<double,double,double,double> {
                size_t nn = v.size(); double sum_sq = 0.0, sum = 0.0;
                for (auto d : v) { sum_sq += d*d; sum += d; }
                double rmse = std::sqrt(sum_sq / nn), mean = sum / nn;
                auto s = v; std::sort(s.begin(), s.end());
                size_t idx = static_cast<size_t>(std::ceil(0.95 * s.size())) - 1;
                double p95 = s[std::min(idx, s.size() - 1)];
                double mx = *std::max_element(s.begin(), s.end());
                return {rmse, mean, p95, mx};
            };
            auto [sr, sm, sp95, smx] = calc(ps.err_sym_norms);
            auto [cr, cm, cp95, cmx] = calc(ps.consistency_norms);
            double ar = std::sqrt(ps.sum_sq_a / n);
            double br = std::sqrt(ps.sum_sq_b / n);
            double mean_dt = std::accumulate(ps.dt_pairs.begin(), ps.dt_pairs.end(), 0.0) / n;
            csv << all_iris_[pk.first] << ',' << all_iris_[pk.second] << ','
                << n << ',' << mean_dt << ','
                << sr << ',' << sm << ',' << sp95 << ',' << smx << ','
                << ar << ',' << br << ','
                << cr << ',' << cp95 << ',' << cmx << '\n';
        }
        ROS_INFO("[DEKF TEST] pairwise summary CSV: %s", path.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // legacy
    std::string reference_name_;
    std::vector<std::string> target_names_;
    std::vector<std::string> all_iris_;
    std::string csv_dir_;

    double max_model_align_dt_ = 0.05;
    double history_keep_time_ = 3.0;
    bool ignore_after_mission_complete_ = true;
    bool count_ignored_callbacks_ = true;
    int eval_stop_stage_ = 7;

    // pairwise
    bool enable_pairwise_eval_ = true;
    double pairwise_max_sync_dt_ = 0.05;
    std::string pairwise_dir_;

    using PairKey = std::pair<size_t, size_t>;
    std::vector<PairKey> pair_keys_;
    std::map<PairKey, std::deque<DekfSample>> dekf_buffers_;
    std::map<PairKey, PairStats> pair_stats_;
    std::map<PairKey, std::ofstream> pair_csv_;

    // GT 缓存
    std::map<std::string, std::deque<PoseSample>> gt_history_;

    // 订阅
    ros::Subscriber model_sub_;
    ros::Subscriber stage_sub_;
    std::vector<ros::Subscriber> dekf_subs_;
    std::vector<ros::Subscriber> pairwise_subs_;

    // 阶段
    uint8_t mission_stage_ = 0;
    bool recording_enabled_ = true;
    bool stopped_by_stage_ = false;
    ros::Time eval_stop_time_;

    // legacy 统计
    std::vector<double> de_sum_sq_;
    std::vector<size_t> de_sample_count_;
    std::vector<size_t> de_callbacks_;
    std::vector<size_t> de_ignored_;
    std::vector<size_t> de_reject_model_;
    std::vector<std::vector<double>> de_error_norms_;
    std::vector<std::ofstream> de_csv_;
    bool reported_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_dgo_dekf_test");
    DekfEvaluator eval;
    ros::spin();
    return 0;
}
