/**
 * ID 匹配节点
 *
 * 功能:
 *   根据 INS 估计的相对角度 (alpha_INS, theta_INS) 与视觉检测角度 (alpha, theta)
 *   进行最近邻匹配, 为每个检测结果打上无人机 ID 标签.
 *
 * 订阅:
 *   iris_X/ins_estimate      - 所有无人机的 INS 相对初始 ENU 位移 (Odometry)
 *   iris_X/camera_angle      — 当前无人机的视觉检测角度
 *   iris_X/image_detection   - 当前无人机的检测框中心, 用于连续帧 ID 复用
 *   /gazebo/model_states     - 仿真中捕获各机初始 ENU 位置
 *
 * 发布:
 *   iris_X/camera_angle_match — 带 ID 标签的检测结果 (CameraAngleMatch)
 *
 * 算法:
 *   对每个检测结果, 遍历所有其他无人机 j:
 *     1. 解析 Pi (本机) 和 Pj (无人机 j) 的三维位置
 *     2. 计算 ENU 世界系相对位置 Pij_world = Pj - Pi
 *     3. 直接在 ENU 世界系计算 INS 角度:
 *          alpha_INS = atan2(Pij_world.y, Pij_world.x)
 *          theta_INS = asin(Pij_world.z / |Pij_world|)
 *     4. 计算误差: error = (alpha - alpha_INS)^2 + (theta - theta_INS)^2
 *     5. 取小于阈值的最小误差对应 j 作为匹配 ID, 否则输出 -1
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensors/ImageDetection.h>
#include <data_process/CameraAngle.h>
#include <data_process/CameraAngleMatch.h>
#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include <cmath>

class IdMatch
{
public:
    IdMatch() : nh_(), pnh_("~")
    {
        ns_ = ros::this_node::getNamespace();

        // 读取无人机名称列表
        std::vector<std::string> default_names = {"iris_0", "iris_1", "iris_2", "iris_3"};
        pnh_.param("drone_names", drone_names_, default_names);
        num_drones_ = drone_names_.size();
        pnh_.param("max_match_error", max_match_error_, 0.8);
        max_match_error_sq_ = max_match_error_ * max_match_error_;
        pnh_.param("track_center_threshold", track_center_threshold_, 0.08);
        track_center_threshold_sq_ = track_center_threshold_ * track_center_threshold_;
        pnh_.param("track_max_dt", track_max_dt_, 0.2);

        // 找到本机在列表中的索引
        std::string ns_clean = ns_;
        if (!ns_clean.empty() && ns_clean[0] == '/')
            ns_clean = ns_clean.substr(1);
        self_index_ = -1;
        for (size_t i = 0; i < drone_names_.size(); i++)
        {
            if (drone_names_[i] == ns_clean)
            {
                self_index_ = static_cast<int>(i);
                break;
            }
        }

        if (self_index_ < 0)
        {
            ROS_ERROR("[ID MATCH] namespace '%s' not in drone_names list", ns_clean.c_str());
            ros::shutdown();
            return;
        }

        // 为每架无人机初始化位置/姿态缓存.
        // ins_estimate 的 position 是各机相对自身初始 ENU 的位移, 不能直接跨机相减.
        // 仿真中用 /gazebo/model_states 捕获各机初始世界位置作为共同 ENU origin.
        positions_.resize(num_drones_, Eigen::Vector3d::Zero());
        origins_.resize(num_drones_, Eigen::Vector3d::Zero());
        quaternions_.resize(num_drones_, Eigen::Quaterniond::Identity());
        rot_mats_.resize(num_drones_, Eigen::Matrix3d::Identity());
        pos_received_.resize(num_drones_, false);
        origin_received_.resize(num_drones_, false);

        model_sub_ = nh_.subscribe("/gazebo/model_states", 10,
                                   &IdMatch::modelStatesCallback, this);

        // 订阅所有无人机的 ins_estimate (全局 topic)
        for (int i = 0; i < num_drones_; i++)
        {
            std::string topic = "/" + drone_names_[i] + "/ins_estimate";
            ins_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(topic, 10,
                    boost::bind(&IdMatch::insCallback, this, _1, i)));
        }

        // 订阅本机的 camera_angle. 注意这里必须用 public NodeHandle,
        // private NodeHandle 会解析到 /iris_X/id_match/camera_angle.
        cam_angle_sub_ = nh_.subscribe("camera_angle", 10,
                                        &IdMatch::cameraAngleCallback, this);
        image_det_sub_ = nh_.subscribe("image_detection", 10,
                                        &IdMatch::imageDetectionCallback, this);

        // 发布带 ID 标签的检测结果
        match_pub_ = nh_.advertise<data_process::CameraAngleMatch>(
            "camera_angle_match", 10);

        ROS_INFO("[ID MATCH] ns=%s self_idx=%d monitor=%d max_match_error=%.3f rad",
                 ns_clean.c_str(), self_index_, num_drones_, max_match_error_);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        if (allOriginsReceived())
            return;

        for (int i = 0; i < num_drones_; ++i)
        {
            if (origin_received_[i])
                continue;

            for (size_t k = 0; k < msg->name.size(); ++k)
            {
                if (msg->name[k] != drone_names_[i])
                    continue;

                origins_[i].x() = msg->pose[k].position.x;
                origins_[i].y() = msg->pose[k].position.y;
                origins_[i].z() = msg->pose[k].position.z;
                origin_received_[i] = true;

                ROS_INFO("[ID MATCH] drone %s initial Gazebo ENU=(%.2f, %.2f, %.2f)",
                         drone_names_[i].c_str(),
                         origins_[i].x(), origins_[i].y(), origins_[i].z());
                break;
            }
        }
    }

    bool allOriginsReceived() const
    {
        for (bool received : origin_received_)
        {
            if (!received)
                return false;
        }
        return true;
    }

    // =========================================================================
    //  INS 估计回调 — 缓存各无人机三维位置 + 姿态矩阵
    // =========================================================================

    void insCallback(const nav_msgs::Odometry::ConstPtr &msg, int drone_idx)
    {
        positions_[drone_idx].x() = msg->pose.pose.position.x;
        positions_[drone_idx].y() = msg->pose.pose.position.y;
        positions_[drone_idx].z() = msg->pose.pose.position.z;

        quaternions_[drone_idx].w() = msg->pose.pose.orientation.w;
        quaternions_[drone_idx].x() = msg->pose.pose.orientation.x;
        quaternions_[drone_idx].y() = msg->pose.pose.orientation.y;
        quaternions_[drone_idx].z() = msg->pose.pose.orientation.z;
        rot_mats_[drone_idx] = quaternions_[drone_idx].normalized().toRotationMatrix();

        if (!pos_received_[drone_idx])
        {
            pos_received_[drone_idx] = true;
            // eulerAngles(2,1,0) 返回顺序为 yaw, pitch, roll.
            Eigen::Vector3d ypr = rot_mats_[drone_idx].eulerAngles(2, 1, 0);
            ROS_INFO("[ID MATCH] drone %s init YPR=(%.1f, %.1f, %.1f) deg",
                     drone_names_[drone_idx].c_str(),
                     ypr.x()*180.0/M_PI, ypr.y()*180.0/M_PI, ypr.z()*180.0/M_PI);
        }
    }

    // =========================================================================
    //  CameraAngle 回调 — 执行 ID 匹配
    // =========================================================================

    void imageDetectionCallback(const sensors::ImageDetection::ConstPtr &msg)
    {
        latest_detection_ = msg;
    }

    bool getCurrentCenters(const ros::Time &stamp,
                           size_t n_det,
                           std::vector<Eigen::Vector2d> &centers) const
    {
        centers.clear();
        if (!latest_detection_)
            return false;

        if (std::fabs((latest_detection_->header.stamp - stamp).toSec()) > track_max_dt_)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[ID MATCH] image_detection and camera_angle timestamp mismatch; skip frame tracking");
            return false;
        }

        const size_t n_box = std::min<size_t>(
            latest_detection_->count,
            std::min(latest_detection_->x.size(), latest_detection_->y.size()));
        if (n_box < n_det)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[ID MATCH] image_detection count smaller than camera_angle: boxes=%zu angles=%zu",
                              n_box, n_det);
            return false;
        }

        centers.reserve(n_det);
        for (size_t i = 0; i < n_det; ++i)
        {
            centers.emplace_back(latest_detection_->x[i], latest_detection_->y[i]);
        }
        return true;
    }

    void cameraAngleCallback(const data_process::CameraAngle::ConstPtr &msg)
    {
        if (msg->header.frame_id != "map" && msg->header.frame_id != "odom")
        {
            ROS_WARN_THROTTLE(5.0,
                              "[ID MATCH] camera_angle frame_id='%s'; matching expects alpha/theta in ENU world frame",
                              msg->header.frame_id.c_str());
        }

        const size_t n_det = std::min<size_t>(
            msg->count, std::min(msg->alpha.size(), msg->theta.size()));
        if (n_det != msg->count)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[ID MATCH] CameraAngle count mismatch: count=%u alpha=%zu theta=%zu, use %zu",
                              msg->count, msg->alpha.size(), msg->theta.size(), n_det);
        }

        // 检查本机位置/初始 ENU 是否已收到
        if (!pos_received_[self_index_] || !origin_received_[self_index_])
        {
            ROS_WARN_THROTTLE(2.0, "[ID MATCH] 本机 INS 位置或初始 ENU 未收到, 跳过匹配");
            return;
        }

        Eigen::Vector3d Pi = origins_[self_index_] + positions_[self_index_];
        std::vector<Eigen::Vector2d> current_centers;
        const bool has_current_centers =
            getCurrentCenters(msg->header.stamp, n_det, current_centers);

        data_process::CameraAngleMatch match_msg;
        match_msg.header = msg->header;
        match_msg.count = static_cast<uint8_t>(n_det);
        match_msg.alpha.reserve(n_det);
        match_msg.theta.reserve(n_det);
        match_msg.id.reserve(n_det);

        if (n_det == 0)
        {
            prev_detections_.clear();
            match_pub_.publish(match_msg);
            return;
        }

        // 第一遍: 为每个检测计算候选 best_id / best_error
        struct DetCandidate
        {
            float alpha;
            float theta;
            int8_t best_id;
            double best_error;
            Eigen::Vector2d center;
        };
        std::vector<DetCandidate> candidates(n_det);

        for (size_t d = 0; d < n_det; d++)
        {
            DetCandidate &cand = candidates[d];
            cand.alpha = msg->alpha[d];
            cand.theta = msg->theta[d];
            cand.best_id = -1;
            cand.best_error = std::numeric_limits<double>::infinity();
            cand.center = has_current_centers ? current_centers[d] : Eigen::Vector2d::Zero();

            if (!std::isfinite(cand.alpha) || !std::isfinite(cand.theta))
                continue;

            // Step 1: 帧间跟踪
            if (has_current_centers && !prev_detections_.empty())
            {
                for (size_t p = 0; p < prev_detections_.size(); ++p)
                {
                    if (prev_detections_[p].id < 0)
                        continue;

                    const double dx = cand.center.x() - prev_detections_[p].center.x();
                    const double dy = cand.center.y() - prev_detections_[p].center.y();
                    const double error = dx * dx + dy * dy;
                    if (error < cand.best_error)
                    {
                        cand.best_error = error;
                        cand.best_id = prev_detections_[p].id;
                    }
                }
                if (cand.best_error > track_center_threshold_sq_)
                {
                    cand.best_id = -1;
                }
            }

            // Step 2: 若跟踪失效, 用 INS 角度匹配
            if (cand.best_id < 0)
            {
                cand.best_error = std::numeric_limits<double>::infinity();

                for (int j = 0; j < num_drones_; j++)
                {
                    if (j == self_index_)
                        continue;
                    if (!pos_received_[j] || !origin_received_[j])
                        continue;

                    Eigen::Vector3d Pj = origins_[j] + positions_[j];
                    Eigen::Vector3d Pij_world = Pj - Pi;
                    double dist_world = Pij_world.norm();
                    if (dist_world < 1e-6)
                        continue;

                    double alpha_ins = std::atan2(Pij_world.y(), Pij_world.x());
                    double ratio = Pij_world.z() / dist_world;
                    ratio = std::max(-1.0, std::min(1.0, ratio));
                    double theta_ins = std::asin(ratio);

                    double da = normalizeAngle(cand.alpha - alpha_ins);
                    double dt = cand.theta - theta_ins;
                    double error = da * da + dt * dt;
                    if (error < cand.best_error)
                    {
                        cand.best_error = error;
                        cand.best_id = static_cast<int8_t>(j);
                    }
                }
                if (cand.best_error > max_match_error_sq_)
                    cand.best_id = -1;
            }
        }

        // 第二遍: 按误差从小到大排序, 贪心分配 ID, 冲突的置 -1
        std::vector<size_t> order(n_det);
        for (size_t i = 0; i < n_det; i++)
            order[i] = i;
        std::sort(order.begin(), order.end(),
                  [&candidates](size_t a, size_t b) {
                      return candidates[a].best_error < candidates[b].best_error;
                  });

        std::vector<bool> drone_used(num_drones_, false);
        for (size_t idx : order)
        {
            DetCandidate &cand = candidates[idx];
            if (cand.best_id < 0)
                continue;
            if (drone_used[cand.best_id])
                cand.best_id = -1;  // 该 ID 已被误差更小的检测占用
            else
                drone_used[cand.best_id] = true;
        }

        // 填充输出消息 (保持原始顺序)
        for (size_t d = 0; d < n_det; d++)
        {
            match_msg.alpha.push_back(candidates[d].alpha);
            match_msg.theta.push_back(candidates[d].theta);
            match_msg.id.push_back(candidates[d].best_id);

            if (candidates[d].best_id >= 0)
            {
                ROS_INFO_THROTTLE(1.0,
                    "[ID MATCH] det=%zu assigned drone %s (id=%d) error=%.3f",
                    d, drone_names_[candidates[d].best_id].c_str(),
                    candidates[d].best_id, std::sqrt(candidates[d].best_error));
            }
        }

        // 保存当前帧检测结果, 供下一帧帧间跟踪使用
        prev_detections_.clear();
        for (size_t d = 0; d < n_det; d++)
        {
            if (!has_current_centers || candidates[d].best_id < 0)
                continue;

            PrevDet pd;
            pd.center = candidates[d].center;
            pd.id = candidates[d].best_id;
            prev_detections_.push_back(pd);
        }

        match_pub_.publish(match_msg);
    }

    // =========================================================================
    //  成员变量
    // =========================================================================

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string ns_;
    int num_drones_;
    int self_index_;
    double max_match_error_;
    double max_match_error_sq_;
    double track_center_threshold_;
    double track_center_threshold_sq_;
    double track_max_dt_;

    // 帧间跟踪: 上一帧的检测结果
    struct PrevDet
    {
        Eigen::Vector2d center;
        int8_t id;
    };
    std::vector<PrevDet> prev_detections_;

    std::vector<std::string> drone_names_;
    ros::Subscriber model_sub_;
    std::vector<ros::Subscriber> ins_subs_;
    ros::Subscriber cam_angle_sub_;
    ros::Subscriber image_det_sub_;
    ros::Publisher match_pub_;
    sensors::ImageDetection::ConstPtr latest_detection_;

    std::vector<Eigen::Vector3d> positions_;
    std::vector<Eigen::Vector3d> origins_;
    std::vector<Eigen::Quaterniond> quaternions_;
    std::vector<Eigen::Matrix3d> rot_mats_;
    std::vector<bool> pos_received_;
    std::vector<bool> origin_received_;

    static double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "id_match");
    IdMatch node;
    ros::spin();
    return 0;
}
