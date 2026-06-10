#include <ros/ros.h>
#include <sensors/ImageDetection.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <data_process/CameraAngle.h>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
class CameraRelativeAngleCal
{
public:
    CameraRelativeAngleCal()
    {
        ros::NodeHandle pnh("~");
        std::string ns = ros::this_node::getNamespace();

        // 构建 camera_link 全局 frame 名 (eg. /iris_0 -> iris_0/camera_link)
        std::string ns_clean = ns;
        if (!ns_clean.empty() && ns_clean[0] == '/')
            ns_clean = ns_clean.substr(1);
        camera_frame_ = ns_clean + "/camera_link";

        // odom frame: PX4 使用 map 作为局部 ENU 参考系
        pnh.param<std::string>("odom_frame", odom_frame_, "map");

        ROS_INFO("[%s] camera_frame=%s, odom_frame=%s",
                 ns.c_str(), camera_frame_.c_str(), odom_frame_.c_str());

        // 订阅 image_detection (归一化检测框)
        det_sub_ = nh_.subscribe("image_detection", 10,
                                 &CameraRelativeAngleCal::detCallback, this);

        // 订阅 camera_info (获取相机内参 K)
        cam_info_sub_ = nh_.subscribe("camera/camera_info", 10,
                                       &CameraRelativeAngleCal::camInfoCallback, this);

        // 发布 polar angles (批量, 每帧一消息)
        angle_pub_ = nh_.advertise<data_process::CameraAngle>(ns + "/camera_angle", 10);
    }

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        has_cam_info_ = true;
    }

    void detCallback(const sensors::ImageDetection::ConstPtr &msg)
    {
        if (!has_cam_info_ || msg->count == 0)
            return;

        // 查询 camera_link -> odom_frame 变换 (取 3x3 旋转矩阵 R')
        tf::StampedTransform transform;
        try
        {
            tf_listener_.lookupTransform(odom_frame_, camera_frame_,
                                         ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF lookup %s -> %s failed: %s",
                              camera_frame_.c_str(), odom_frame_.c_str(), ex.what());
            return;
        }

        tf::Matrix3x3 R = transform.getBasis(); // 3x3 旋转矩阵

        // 存储当前帧所有检测结果的极坐标角度
        std::vector<float> alphas, thetas;
        alphas.reserve(msg->count);
        thetas.reserve(msg->count);

        float mean_alpha = 0.0f, mean_theta = 0.0f;
        data_process::CameraAngle msg_out;
        msg_out.header.stamp = msg->header.stamp;
        msg_out.header.frame_id = odom_frame_;
        msg_out.count = msg->count;

        for (size_t i = 0; i < msg->count; i++)
        {
            // Step 1: 归一化 [0,1] -> 像素坐标
            float u = msg->x[i] * IMG_WIDTH_;
            float v = msg->y[i] * IMG_HEIGHT_;

            // Step 2: 像素坐标 -> 相机系射线方向 (归一化像平面)
            tf::Vector3 ray_cam((u - cx_) / fx_, (v - cy_) / fy_, 1.0);

            // Step 3: 旋转到 odom 系 (ENU)
            tf::Vector3 ray_odom = R * ray_cam;

            double x_paper = ray_odom.x();  // 东
            double y_paper = ray_odom.y();  // 北
            double z_paper = ray_odom.z();  // 天
            double mag = std::sqrt(x_paper * x_paper +
                                   y_paper * y_paper +
                                   z_paper * z_paper);
            if (mag < 1e-6) continue;

            // Step 4: 极坐标角度
            double alpha = std::atan2(y_paper, x_paper);
            double theta = std::asin(z_paper / mag);

            // Step 5: zero-score 异常值剔除 + 置信度检查
            if (msg->confidence[i] <= CONF_THRESHOLD_)
                continue;

            if (!zeroScoreTest(static_cast<float>(alpha),
                               static_cast<float>(theta)))
                continue;

            // 通过检查，更新队列
            
            angle_queue_.push_back({static_cast<float>(alpha),
                                    static_cast<float>(theta),
                                    msg->confidence[i]});
            if (angle_queue_.size() > QUEUE_SIZE_)
                angle_queue_.pop_front();

            // 存入当前帧发布结果
            alphas.push_back(static_cast<float>(alpha));
            thetas.push_back(static_cast<float>(theta));


            //存入结果之后进行均值滤波
            // 均值滤波: 用队列均值平滑输出
            if (!angle_queue_.empty())
            {
                mean_alpha = 0.0f, mean_theta = 0.0f;
                for (const auto &d : angle_queue_)
                {
                    mean_alpha += d.alpha;
                    mean_theta += d.theta;
                }
                mean_alpha /= angle_queue_.size();
                mean_theta /= angle_queue_.size();
                msg_out.alpha.push_back(mean_alpha);
                msg_out.theta.push_back(mean_theta);
            }
        }
        angle_pub_.publish(msg_out);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber det_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher angle_pub_;
    tf::TransformListener tf_listener_;

    std::string camera_frame_;   // eg. "iris_0/camera_link"
    std::string odom_frame_;     // eg. "map" (PX4 local ENU)

    float fx_ = 0.0f, fy_ = 0.0f, cx_ = 0.0f, cy_ = 0.0f;
    bool has_cam_info_ = false;

    static constexpr float IMG_WIDTH_  = 320.0f;
    static constexpr float IMG_HEIGHT_ = 240.0f;

    // zero-score 异常值剔除参数
    static constexpr size_t QUEUE_SIZE_ = 10;
    static constexpr float CONF_THRESHOLD_ = 0.5f;
    static constexpr float Z_SCORE_THRESHOLD_ = 3.0f;

    struct AngleData
    {
        float alpha;
        float theta;
        float confidence;
    };
    std::deque<AngleData> angle_queue_;

    /** zero-score 检验: |val - mean| < Z_SCORE_THRESHOLD_ * std */
    bool zeroScoreTest(float alpha, float theta) const
    {
        if (angle_queue_.size() < 2)
            return true;  // 队列数据不足时不拒绝

        // 计算队列中 alpha / theta 的均值和标准差
        double sum_a = 0.0, sum_t = 0.0;
        for (const auto &d : angle_queue_)
        {
            sum_a += d.alpha;
            sum_t += d.theta;
        }
        double mean_a = sum_a / angle_queue_.size();
        double mean_t = sum_t / angle_queue_.size();

        double var_a = 0.0, var_t = 0.0;
        for (const auto &d : angle_queue_)
        {
            var_a += (d.alpha - mean_a) * (d.alpha - mean_a);
            var_t += (d.theta - mean_t) * (d.theta - mean_t);
        }
        double std_a = std::sqrt(var_a / angle_queue_.size());
        double std_t = std::sqrt(var_t / angle_queue_.size());

        return std::abs(alpha - mean_a) < Z_SCORE_THRESHOLD_ * std_a &&
               std::abs(theta - mean_t) < Z_SCORE_THRESHOLD_ * std_t;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_relative_angle_cal");
    CameraRelativeAngleCal node;
    ros::spin();
    return 0;
}
