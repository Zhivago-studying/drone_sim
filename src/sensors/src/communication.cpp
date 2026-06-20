/**
 * @file communication.cpp
 * @brief UAV communication broadcast simulator.
 *
 * Each UAV broadcasts its position estimate and INS velocity at a fixed rate.
 *
 * Position source (论文: 广播上一历元优化位置):
 *   - 优先 dgo_estimate (DGO 优化后的位置),新鲜时使用;
 *   - DGO 不新鲜或未就绪时回退 ins_estimate,避免启动死锁
 *     (DGO 等 communication, communication 硬等 dgo_estimate 会卡死)。
 *
 * Velocity source:
 *   - 始终来自 ins_estimate.twist (DGO 目前只修正位置,不估计速度)。
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensors/ComMsg.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

class CommunicationNode
{
public:
    CommunicationNode()
        : nh_(), pnh_("~")
    {
        const std::string ns = cleanNamespace(ros::this_node::getNamespace());
        int id_default = parseDroneId(ns);
        int id_param = id_default;
        pnh_.param("id", id_param, id_default);
        if (id_param < 0 || id_param > 255)
        {
            ROS_WARN("[communication] invalid id=%d, clamp to uint8 range", id_param);
            id_param = std::max(0, std::min(255, id_param));
        }
        id_ = static_cast<uint8_t>(id_param);

        pnh_.param("rate", publish_rate_, 10.0);
        if (publish_rate_ <= 0.0)
        {
            ROS_WARN("[communication] invalid rate %.2f, using 10Hz", publish_rate_);
            publish_rate_ = 10.0;
        }
        pnh_.param("use_dgo_position", use_dgo_position_, true);
        pnh_.param("max_dgo_age", max_dgo_age_, 0.3);
        pnh_.param("stabilize_position", stabilize_position_, true);
        pnh_.param("dgo_correction_gain", dgo_correction_gain_, 0.35);
        pnh_.param("ins_fallback_gain", ins_fallback_gain_, 0.15);
        pnh_.param("max_position_correction_rate",
                   max_position_correction_rate_, 0.8);
        pnh_.param("max_dgo_ins_disagreement",
                   max_dgo_ins_disagreement_, 0.75);
        pnh_.param("max_prediction_speed", max_prediction_speed_, 2.0);

        ins_sub_ = nh_.subscribe("ins_estimate", 10,
                                 &CommunicationNode::insCallback, this);
        dgo_sub_ = nh_.subscribe("dgo_estimate", 10,
                                 &CommunicationNode::dgoCallback, this);
        com_pub_ = nh_.advertise<sensors::ComMsg>("communication", 10);

        ROS_INFO("[communication] ns=%s id=%u rate=%.1fHz pos_source=%s "
                 "vel_source=ins_estimate max_dgo_age=%.2f stabilize=%d "
                 "gain_dgo=%.2f gain_ins=%.2f correction_rate=%.2f "
                 "max_dgo_ins_disagreement=%.2f max_prediction_speed=%.2f",
                 ns.c_str(), static_cast<unsigned int>(id_), publish_rate_,
                 use_dgo_position_ ? "dgo_estimate(fallback=ins)" : "ins_estimate",
                 max_dgo_age_, stabilize_position_ ? 1 : 0,
                 dgo_correction_gain_, ins_fallback_gain_,
                 max_position_correction_rate_, max_dgo_ins_disagreement_,
                 max_prediction_speed_);
    }

    void spin()
    {
        ros::Rate rate(publish_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            publish();
            rate.sleep();
        }
    }

private:
    static std::string cleanNamespace(const std::string &ns)
    {
        if (ns.empty() || ns == "/")
            return "";
        return ns.front() == '/' ? ns.substr(1) : ns;
    }

    static int parseDroneId(const std::string &ns)
    {
        const std::string prefix = "iris_";
        const size_t pos = ns.rfind(prefix);
        if (pos == std::string::npos)
            return 0;

        const std::string id_str = ns.substr(pos + prefix.size());
        char *end = nullptr;
        long value = std::strtol(id_str.c_str(), &end, 10);
        if (end == id_str.c_str() || *end != '\0' || value < 0 || value > 255)
            return 0;
        return static_cast<int>(value);
    }

    void insCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        latest_ins_msg_ = msg;
        has_ins_ = true;
    }

    void dgoCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        latest_dgo_msg_ = msg;
        has_dgo_ = true;
    }

    void publish()
    {
        if (!latest_ins_msg_)
        {
            ROS_WARN_THROTTLE(10.0, "[communication] waiting for ins_estimate");
            return;
        }

        const ros::Time now = ros::Time::now();

        // 位置优先取 DGO (论文: 广播上一历元优化位置),DGO 不新鲜时回退 INS。
        // 保留 INS fallback 是为避免启动死锁:
        //   DGO 等 communication, communication 若硬等 dgo_estimate 会卡死。
        bool use_dgo = false;
        if (use_dgo_position_ && latest_dgo_msg_)
        {
            const double age = std::fabs((now - latest_dgo_msg_->header.stamp).toSec());
            use_dgo = age <= max_dgo_age_;
            if (use_dgo)
            {
                const auto &dgo = latest_dgo_msg_->pose.pose.position;
                const auto &ins = latest_ins_msg_->pose.pose.position;
                const double dx = dgo.x - ins.x;
                const double dy = dgo.y - ins.y;
                const double dz = dgo.z - ins.z;
                const double disagreement =
                    std::sqrt(dx * dx + dy * dy + dz * dz);
                if (disagreement > max_dgo_ins_disagreement_)
                {
                    use_dgo = false;
                    ROS_WARN_THROTTLE(
                        5.0,
                        "[communication] reject DGO source: disagreement "
                        "to INS %.2fm > %.2fm",
                        disagreement, max_dgo_ins_disagreement_);
                }
            }
        }

        const nav_msgs::Odometry::ConstPtr &pos_src =
            use_dgo ? latest_dgo_msg_ : latest_ins_msg_;

        sensors::ComMsg msg;
        msg.header.seq = seq_++;

        // 时间戳用"位置估计对应的时间",而非发布时刻 now,
        // 保证下游 DGO 用速度外推时的 dt 是真实位置-速度时间差。
        msg.header.stamp = stabilize_position_ ? now : pos_src->header.stamp;
        msg.header.frame_id = pos_src->header.frame_id;
        msg.id = id_;

        // 速度始终来自 INS: DGO 目前只修正位置,不估计速度。
        msg.velocity.x = latest_ins_msg_->twist.twist.linear.x;
        msg.velocity.y = latest_ins_msg_->twist.twist.linear.y;
        msg.velocity.z = latest_ins_msg_->twist.twist.linear.z;

        if (stabilize_position_)
        {
            updateFilteredPosition(pos_src->pose.pose.position,
                                   msg.velocity, use_dgo, now);
            msg.position = filtered_position_;
        }
        else
        {
            msg.position = pos_src->pose.pose.position;
        }

        com_pub_.publish(msg);

        ROS_DEBUG_THROTTLE(2.0,
            "[communication] source=%s pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f)",
            use_dgo ? "DGO" : "INS",
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z);
    }

    void updateFilteredPosition(
        const geometry_msgs::Point &target,
        const geometry_msgs::Vector3 &velocity,
        bool use_dgo,
        const ros::Time &now)
    {
        if (!filtered_position_initialized_)
        {
            filtered_position_ = target;
            filtered_position_stamp_ = now;
            filtered_position_initialized_ = true;
            return;
        }

        double dt = (now - filtered_position_stamp_).toSec();
        dt = std::max(0.0, std::min(0.5, dt));

        double vx = velocity.x;
        double vy = velocity.y;
        double vz = velocity.z;
        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (speed > max_prediction_speed_ && speed > 1e-9)
        {
            const double scale = max_prediction_speed_ / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }

        geometry_msgs::Point predicted = filtered_position_;
        predicted.x += vx * dt;
        predicted.y += vy * dt;
        predicted.z += vz * dt;

        const double gain = use_dgo ? dgo_correction_gain_
                                    : ins_fallback_gain_;
        double cx = gain * (target.x - predicted.x);
        double cy = gain * (target.y - predicted.y);
        double cz = gain * (target.z - predicted.z);
        const double correction_norm =
            std::sqrt(cx * cx + cy * cy + cz * cz);
        const double max_correction =
            std::max(0.0, max_position_correction_rate_ * dt);
        if (correction_norm > max_correction &&
            correction_norm > 1e-9)
        {
            const double scale = max_correction / correction_norm;
            cx *= scale;
            cy *= scale;
            cz *= scale;
        }

        filtered_position_.x = predicted.x + cx;
        filtered_position_.y = predicted.y + cy;
        filtered_position_.z = predicted.z + cz;
        filtered_position_stamp_ = now;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber ins_sub_;
    ros::Subscriber dgo_sub_;
    ros::Publisher com_pub_;
    nav_msgs::Odometry::ConstPtr latest_ins_msg_;
    nav_msgs::Odometry::ConstPtr latest_dgo_msg_;

    uint8_t id_ = 0;
    uint32_t seq_ = 0;
    double publish_rate_ = 10.0;
    double max_dgo_age_ = 0.3;
    double dgo_correction_gain_ = 0.35;
    double ins_fallback_gain_ = 0.15;
    double max_position_correction_rate_ = 0.8;
    double max_dgo_ins_disagreement_ = 0.75;
    double max_prediction_speed_ = 2.0;
    bool use_dgo_position_ = true;
    bool stabilize_position_ = true;
    bool has_ins_ = false;
    bool has_dgo_ = false;
    bool filtered_position_initialized_ = false;
    geometry_msgs::Point filtered_position_;
    ros::Time filtered_position_stamp_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication");
    CommunicationNode node;
    node.spin();
    return 0;
}
