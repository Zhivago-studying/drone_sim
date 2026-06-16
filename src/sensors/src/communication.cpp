/**
 * @file communication.cpp
 * @brief UAV communication broadcast simulator.
 *
 * Each UAV publishes its current INS position estimate and horizontal
 * optical-flow velocity at a fixed communication rate.
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensors/ComMsg.h>

#include <algorithm>
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
        ins_sub_ = nh_.subscribe("ins_estimate", 10,
                                 &CommunicationNode::insCallback, this);
        com_pub_ = nh_.advertise<sensors::ComMsg>("communication", 10);

        ROS_INFO("[communication] ns=%s id=%u rate=%.1fHz source=ins_estimate",
                 ns.c_str(), static_cast<unsigned int>(id_), publish_rate_);
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

    void publish()
    {
        if (!latest_ins_msg_)
        {
            ROS_WARN_THROTTLE(10.0, "[communication] waiting for ins_estimate");
            return;
        }

        sensors::ComMsg msg;
        msg.header.seq = seq_++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = latest_ins_msg_->header.frame_id;
        msg.id = id_;

        msg.position.x = latest_ins_msg_->pose.pose.position.x;
        msg.position.y = latest_ins_msg_->pose.pose.position.y;
        msg.position.z = latest_ins_msg_->pose.pose.position.z;

        msg.velocity.x = latest_ins_msg_->twist.twist.linear.x;
        msg.velocity.y = latest_ins_msg_->twist.twist.linear.y;
        msg.velocity.z = latest_ins_msg_->twist.twist.linear.z;

        com_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber ins_sub_;
    ros::Publisher com_pub_;
    nav_msgs::Odometry::ConstPtr latest_ins_msg_;

    uint8_t id_ = 0;
    uint32_t seq_ = 0;
    double publish_rate_ = 10.0;
    bool has_ins_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication");
    CommunicationNode node;
    node.spin();
    return 0;
}
