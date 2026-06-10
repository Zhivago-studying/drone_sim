#include <ros/ros.h>
#include <sensors/UwbRange.h>
#include <data_process/UwbProcessed.h>
#include <deque>
#include <cmath>
#include <vector>

class UwbProcess
{
public:
    UwbProcess()
    {
        ROS_INFO("UWB zero_score processing...");
        std::string ns = ros::this_node::getNamespace();

        std::string uwb_topic = ns + "/uwb";
        uwb_sub_ = nh_.subscribe(uwb_topic, 10, &UwbProcess::sub_callback, this);

        std::string uwb_process_topic = ns + "/uwb_processed";
        uwb_process_pub_ = nh_.advertise<data_process::UwbProcessed>(uwb_process_topic, 10);
    }

    void sub_callback(const sensors::UwbRange::ConstPtr &msg)
    {
        for (int i = 0; i < 3; i++)
        {
            distances_[i] = msg->distances[i];
            target_ids_[i] = msg->target_ids[i];

            if (dq_[i].size() < QUEUE_SIZE_)
            {
                // 队列未满时直接接受，用于初始化统计量
                dq_[i].push_back(static_cast<float>(msg->distances[i]));
                is_available_[i] = true;
            }
            else
            {
                // Zero-score 检验: 先将新值排除在队列之外计算统计量
                float mean = cal_u(dq_[i]);
                float stddev = cal_stddev(dq_[i]);

                if (std::abs(static_cast<float>(msg->distances[i]) - mean) <
                    Z_SCORE_THRESHOLD_ * stddev)
                {
                    dq_[i].pop_front();
                    dq_[i].push_back(static_cast<float>(msg->distances[i]));
                    is_available_[i] = true;
                }
                else
                {
                    // 不更新队列，避免异常值污染滑动窗口统计量
                    is_available_[i] = false;
                }
            }
        }
    }

    float cal_u(const std::deque<float> &dq)
    {
        if (dq.empty()) return 0.0f;
        float sum = 0.0f;
        for (const auto &val : dq)
            sum += val;
        return sum / dq.size();
    }

    float cal_stddev(const std::deque<float> &dq)
    {
        if (dq.empty()) return 0.0f;
        float u = cal_u(dq);
        float temp = 0.0f;
        for (const auto &val : dq)
            temp += (val - u) * (val - u);
        return std::sqrt(temp / dq.size());
    }

    void spin()
    {
        ros::Rate rate(25.0);
        while (ros::ok())
        {
            ros::spinOnce();

            data_process::UwbProcessed msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "uwb_link";

            for (int i = 0; i < 3; i++)
            {
                if (is_available_[i])
                {
                    msg.target_ids.push_back(target_ids_[i]);
                    msg.distances.push_back(cal_u(dq_[i]));  // 均值滤波
                }
            }

            if (!msg.target_ids.empty())
                uwb_process_pub_.publish(msg);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber uwb_sub_;
    ros::Publisher uwb_process_pub_;

    float distances_[3] = {};
    uint8_t target_ids_[3] = {};
    bool is_available_[3] = {true, true, true};
    std::deque<float> dq_[3];

    static constexpr size_t QUEUE_SIZE_ = 10;
    static constexpr float Z_SCORE_THRESHOLD_ = 3.0f;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_zero_score");
    UwbProcess node;
    node.spin();
    return 0;
}
