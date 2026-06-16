/**
 * @file  uwb.cpp
 * @brief UWB 测距模拟节点
 *
 * =========================== 功能描述 ===========================
 * 在 Gazebo 仿真环境中模拟超宽带 (UWB) 测距传感器。每个无人机实例
 * 计算本机到其他 3 架无人机之间的欧氏距离，并添加高斯噪声以模拟
 * 真实 UWB 传感器的测量误差。
 *
 * =========================== 数据流 ===========================
 * 订阅:
 *   /gazebo/model_states  (gazebo_msgs/ModelStates)
 *     - Gazebo 发布的全局真值位置
 *     - 从中提取 iris_0 ~ iris_3 的四元组 (x, y, z) 位置缓存
 *
 * 发布:
 *   /iris_X/uwb  (sensors/UwbRange)
 *     - target_ids[]:   uint8 数组，目标无人机编号
 *     - distances[]:    float64 数组，对应距离 (米)，含 N(0, 0.08) 噪声
 *     - 每帧发布本机到其他所有无人机的测距结果
 *
 * =========================== 关键参数 ===========================
 *   - 发布频率: 25 Hz
 *   - 测距噪声: 高斯分布 σ = 0.08 m (论文典型值)
 *   - 无人机数量: 4 (iris_0 ~ iris_3)
 *
 * =========================== 启动方式 ===========================
 *   由 launch 文件按命名空间启动，例如:
 *   <group ns="iris_0"><node pkg="sensors" type="uwb" .../></group>
 *
 * @note 此节点是仿真专用，实际硬件需替换为真实 UWB 驱动
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensors/UwbRange.h>
#include <random>
#include <cmath>
#include <vector>
#include <string>

class UWB
{
public:
    UWB() : gen_(rd_()), noise_dist_(0.0, 0.08)
    {
        // 从命名空间获取无人机ID，如 /iris_0 -> 0
        std::string ns = ros::this_node::getNamespace();
        iris_id_ = ns.back() - '0';
        ROS_INFO("[UWB] ns=%s id=%d started", ns.c_str(), iris_id_);

        // UWB话题发布者，话题名为 /iris_X/uwb
        std::string uwb_topic = ns + "/uwb";
        uwb_pub_ = nh_.advertise<sensors::UwbRange>(uwb_topic, 10);

        // 订阅Gazebo真实位置
        gt_sub_ = nh_.subscribe("/gazebo/model_states", 10, &UWB::gtCallback, this);

        // 初始化位置缓存
        drones_pos_.resize(4, {0.0, 0.0, 0.0});
    }

    void spin()
    {
        ros::Rate rate(25.0);
        while (ros::ok())
        {
            ros::spinOnce();

            // 一次性发布本机到其他所有无人机的UWB测距
            sensors::UwbRange msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "uwb_link";
            for (int i = 0; i < 4; i++)
            {
                if (i != iris_id_)
                {
                    msg.target_ids.push_back(i);
                    msg.distances.push_back(calcDistance(iris_id_, i));
                }
            }
            uwb_pub_.publish(msg);

            rate.sleep();
        }
    }

private:
    void gtCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        // 查找每架无人机的位置并缓存
        for (int i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < msg->name.size(); j++)
            {
                if (msg->name[j] == drones_[i])
                {
                    drones_pos_[i][0] = msg->pose[j].position.x;
                    drones_pos_[i][1] = msg->pose[j].position.y;
                    drones_pos_[i][2] = msg->pose[j].position.z;
                    break;
                }
            }
        }
    }

    double calcDistance(int i, int j)
    {
        double dx = drones_pos_[i][0] - drones_pos_[j][0];
        double dy = drones_pos_[i][1] - drones_pos_[j][1];
        double dz = drones_pos_[i][2] - drones_pos_[j][2];
        double d = std::sqrt(dx*dx + dy*dy + dz*dz);
        d += noise_dist_(gen_);
        return std::max(d, 0.0);
    }

    ros::NodeHandle nh_;
    ros::Subscriber gt_sub_;
    ros::Publisher uwb_pub_;

    int iris_id_;
    std::vector<std::vector<double>> drones_pos_;
    const std::vector<std::string> drones_{"iris_0", "iris_1", "iris_2", "iris_3"};

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_dist_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb");
    UWB uwb;
    uwb.spin();
    return 0;
}
