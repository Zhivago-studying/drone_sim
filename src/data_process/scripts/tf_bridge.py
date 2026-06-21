#!/usr/bin/env python3
"""
@package tf_bridge.py
TF 桥接节点：将 MAVROS local_position/odom 的位姿发布为 TF 变换

订阅:
  mavros/local_position/odom  (nav_msgs/Odometry)
    - PX4 的局部位置/姿态估计 (map ENU 系)

发布 TF:
  map → iris_X/base_link
    - 使 TF 树完整: map → base_link → camera_link
    - 供 camera_relative_angle_cal 等节点使用

注意事项:
  - 必须在无人机命名空间下运行 (ns=iris_X)
  - 依赖 mavros 节点正常运行
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class TfBridge:
    def __init__(self):
        ns = rospy.get_namespace().strip('/')
        rospy.loginfo('[%s] TF bridge started', ns)

        self.child_frame_ = ns + '/base_link'
        self.br_ = tf2_ros.TransformBroadcaster()

        self.sub_ = rospy.Subscriber(
            'mavros/local_position/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        if rospy.is_shutdown():
            return
        try:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'map'
            t.child_frame_id = self.child_frame_
            t.transform.translation = msg.pose.pose.position
            t.transform.rotation = msg.pose.pose.orientation
            self.br_.sendTransform(t)
        except rospy.exceptions.ROSException:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('tf_bridge')
        node = TfBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
