#!/usr/bin/env python3
"""
@package angle_show.py
角度可视化节点

功能:
  同时显示 iris_0 和 iris_1 的前视相机图像，
  在检测框上标注 angle_est (来自 camera_relative_angle_cal)
  和 angle_gt (来自 /gazebo/model_states 真值)，
  以度为单位。

订阅:
  /iris_X/camera/image_raw     — 原始图像
  /iris_X/image_detection      — YOLOv7 检测框
  /iris_X/camera_angle         — 检测相对角度 (ENU, rad)
  /gazebo/model_states         — 所有无人机真值位置

显示:
  窗口 "Angle Show — Dual UAV View", 左右并排
  检测框上方:
    est: (a=XX, t=XX)  — 绿色, 估计值
    gt:  (a=XX, t=XX)  — 白色, 真值
"""

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensors.msg import ImageDetection
from data_process.msg import CameraAngle
from gazebo_msgs.msg import ModelStates


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AngleShow:
    def __init__(self):
        self.ns_list = ['iris_0', 'iris_1']
        self.bridge = CvBridge()

        self.latest_images = {ns: None for ns in self.ns_list}
        self.latest_dets = {ns: None for ns in self.ns_list}
        self.latest_angles = {ns: None for ns in self.ns_list}
        self.model_states = None

        rospy.Subscriber(
            '/gazebo/model_states', ModelStates,
            self.model_cb, queue_size=10)

        for ns in self.ns_list:
            rospy.Subscriber(
                f'/{ns}/camera/image_raw', Image,
                self.img_cb, callback_args=ns, buff_size=2**24)
            rospy.Subscriber(
                f'/{ns}/image_detection', ImageDetection,
                self.det_cb, callback_args=ns)
            rospy.Subscriber(
                f'/{ns}/camera_angle', CameraAngle,
                self.angle_cb, callback_args=ns)

        rospy.loginfo("[angle_show] ready — dual UAV angle viewer")

    def model_cb(self, msg):
        self.model_states = msg

    def img_cb(self, msg, ns):
        try:
            self.latest_images[ns] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[%s] img decode error: %s', ns, e)

    def det_cb(self, msg, ns):
        self.latest_dets[ns] = msg

    def angle_cb(self, msg, ns):
        self.latest_angles[ns] = msg

    # ------------------------------------------------------------------
    def _compute_gt_angles(self, self_name):
        """从 /gazebo/model_states 计算相对于其他无人机的真值角度"""
        if self.model_states is None:
            return {}

        try:
            si = self.model_states.name.index(self_name)
        except ValueError:
            return {}

        pi = self.model_states.pose[si].position
        gt = {}

        for target_name in self.ns_list:
            if target_name == self_name:
                continue
            try:
                ti = self.model_states.name.index(target_name)
            except ValueError:
                continue

            pj = self.model_states.pose[ti].position
            dx = pj.x - pi.x
            dy = pj.y - pi.y
            dz = pj.z - pi.z
            rng = math.sqrt(dx*dx + dy*dy + dz*dz)
            if rng < 0.05:
                continue

            alpha = math.atan2(dy, dx)        # ENU azimuth
            theta = math.asin(max(-1.0, min(1.0, dz / rng)))  # ENU elevation
            gt[target_name] = (alpha, theta)

        return gt

    # ------------------------------------------------------------------
    def _draw_frame(self, ns):
        img = self.latest_images.get(ns)
        if img is None:
            img = np.full((240, 320, 3), (50, 50, 50), dtype=np.uint8)
            cv2.putText(img, 'Waiting for image...', (60, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(img, ns, (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            return img

        img = img.copy()
        h, w = img.shape[:2]

        dets = self.latest_dets.get(ns)
        angles = self.latest_angles.get(ns)
        gt = self._compute_gt_angles(ns)

        # 构建 GT 角度列表 (方便匹配)
        gt_list = [(name, a, t) for name, (a, t) in gt.items()]

        if dets is not None and dets.count > 0:
            n_det = min(dets.count, len(dets.x), len(dets.y),
                        len(dets.width), len(dets.height), len(dets.confidence))
            n_ang = 0
            if angles is not None:
                n_ang = min(angles.count, len(angles.alpha), len(angles.theta))

            for i in range(n_det):
                # 反归一化像素坐标
                cx = int(dets.x[i] * w)
                cy = int(dets.y[i] * h)
                bw = int(dets.width[i] * w)
                bh = int(dets.height[i] * h)
                x1 = max(0, min(w-1, int(cx - bw/2)))
                y1 = max(0, min(h-1, int(cy - bh/2)))
                x2 = max(0, min(w-1, int(cx + bw/2)))
                y2 = max(0, min(h-1, int(cy + bh/2)))

                # 绿色检测框
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # angle_est: 从 camera_angle 获取 (弧度→度)
                alpha_est = None
                theta_est = None
                if i < n_ang:
                    alpha_est = math.degrees(wrap_to_pi(angles.alpha[i]))
                    theta_est = math.degrees(wrap_to_pi(angles.theta[i]))

                # 匹配最近的 GT 目标
                best_gt_name = None
                best_gt_alpha = None
                best_gt_theta = None
                if alpha_est is not None and len(gt_list) > 0:
                    best_error = float('inf')
                    for gname, ga, gt_ in gt_list:
                        da = wrap_to_pi(angles.alpha[i] - ga)
                        dt_ = wrap_to_pi(angles.theta[i] - gt_)
                        err = math.sqrt(da*da + dt_*dt_)
                        if err < best_error:
                            best_error = err
                            best_gt_name = gname
                            best_gt_alpha = math.degrees(ga)
                            best_gt_theta = math.degrees(gt_)
                elif len(gt_list) == 1:
                    # 只有单个目标，直接对应
                    best_gt_name = gt_list[0][0]
                    best_gt_alpha = math.degrees(gt_list[0][1])
                    best_gt_theta = math.degrees(gt_list[0][2])

                # 绘制角度标签，优先放在检测框上方；空间不足时放到框内顶部。
                screen_thick = 1
                labels = []
                if best_gt_name is not None:
                    labels.append((best_gt_name, (0, 0, 255), 0.45, 2))
                if alpha_est is not None:
                    labels.append((f'est: a={alpha_est:5.1f}, t={theta_est:5.1f} deg',
                                   (0, 255, 128), 0.35, screen_thick))
                if best_gt_alpha is not None:
                    labels.append((f'gt:  a={best_gt_alpha:5.1f}, t={best_gt_theta:5.1f} deg',
                                   (255, 255, 255), 0.35, screen_thick))

                font = cv2.FONT_HERSHEY_SIMPLEX
                line_h = 13
                block_h = line_h * len(labels)
                start_y = y1 - 6 - block_h + line_h
                if start_y < 12:
                    start_y = min(h - 6, y1 + 14)

                for line_idx, (text, color, scale, line_thickness) in enumerate(labels):
                    cv2.putText(img, text,
                                (x1, start_y + line_idx * line_h),
                                font, scale, color, line_thickness)

                # 置信度 (框下方)
                conf_label = f'drone {dets.confidence[i]:.2f}'
                cv2.putText(img, conf_label, (x1, y2 + 12),
                            font, 0.35, (0, 255, 0), screen_thick)

        # 左上角: 本机编号 (黄色)
        cv2.putText(img, ns, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 底部状态栏
        n_det_label = dets.count if dets is not None else 0
        n_ang_label = angles.count if angles is not None else 0
        gt_count = len(gt)
        status = f'#det={n_det_label} #est={n_ang_label} #gt={gt_count}'
        cv2.putText(img, status, (5, h - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (180, 180, 180), 1)

        return img

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(10)
        cv2.namedWindow('Angle Show — Dual UAV View', cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            frames = [self._draw_frame(ns) for ns in self.ns_list]
            side = np.hstack(frames)
            cv2.imshow('Angle Show — Dual UAV View', side)
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord('q'):
                rospy.signal_shutdown('User exit')
                break
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('angle_show')
        viewer = AngleShow()
        viewer.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
