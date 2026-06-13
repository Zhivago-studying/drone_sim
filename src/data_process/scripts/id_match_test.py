#!/usr/bin/env python3
"""
ID 匹配可视化节点 (id_match_test)

功能:
  同时显示四台无人机的前视摄像头图像 (2×2 网格)，
  在识别出的无人机框上标注匹配的 ID 标签。

订阅 (每架无人机):
  /iris_X/camera/image_raw        — 前视相机原始图像
  /iris_X/image_detection         — YOLOv7 检测框 (归一化坐标)
  /iris_X/camera_angle_match      — ID 匹配结果 (alpha, theta, id)

显示:
  窗口 "ID Match — 4 UAV View", 2×2 网格
  每格:
    - 左上角: 黄色无人机编号
    - 绿色矩形框 + "drone 0.95" 置信度
    - ★ 红色 ID 标签: "iris_2" (标注在检测框上方)
    - 未匹配 (id=-1): 显示白色 "?"

启动:
  roslaunch data_process id_match_test.launch
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensors.msg import ImageDetection
from data_process.msg import CameraAngleMatch


class IdMatchTest:
    def __init__(self):
        self.ns_list = ['iris_0', 'iris_1', 'iris_2', 'iris_3']
        self.bridge = CvBridge()

        self.latest_images = {}
        self.latest_dets = {}
        self.latest_matches = {}

        for ns in self.ns_list:
            self.latest_images[ns] = None
            self.latest_dets[ns] = None
            self.latest_matches[ns] = None

            # 原始图像
            rospy.Subscriber(
                f'/{ns}/camera/image_raw', Image,
                self.img_cb, callback_args=ns,
                buff_size=2**24)

            # 检测框
            rospy.Subscriber(
                f'/{ns}/image_detection', ImageDetection,
                self.det_cb, callback_args=ns)

            # ID 匹配结果 (camera_angle_match)
            rospy.Subscriber(
                f'/{ns}/camera_angle_match', CameraAngleMatch,
                self.match_cb, callback_args=ns)

        rospy.loginfo("ID Match Test viewer started — 2x2 grid with ID labels")

    @staticmethod
    def _stamp_diff(a, b):
        return abs((a - b).to_sec())

    def img_cb(self, msg, ns):
        try:
            self.latest_images[ns] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[%s] img decode error: %s', ns, e)

    def det_cb(self, msg, ns):
        self.latest_dets[ns] = msg

    def match_cb(self, msg, ns):
        self.latest_matches[ns] = msg

    def _draw_frame(self, ns):
        img = self.latest_images.get(ns)
        if img is None:
            img = np.full((240, 320, 3), (50, 50, 50), dtype=np.uint8)
            cv2.putText(img, 'Waiting...', (80, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(img, ns, (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            return img

        img = img.copy()
        h, w = img.shape[:2]

        dets = self.latest_dets.get(ns)
        matches = self.latest_matches.get(ns)

        if dets is not None and dets.count > 0:
            # 构建匹配 ID 列表 (与检测框按索引对应)
            match_ids = []
            match_is_current = (
                matches is not None and
                matches.count == dets.count and
                self._stamp_diff(matches.header.stamp, dets.header.stamp) < 0.2)
            if match_is_current:
                match_ids = list(matches.id)

            n_det = min(dets.count, len(dets.x), len(dets.y),
                        len(dets.width), len(dets.height), len(dets.confidence))
            for i in range(n_det):
                # 反归一化像素坐标
                cx = int(dets.x[i] * w)
                cy = int(dets.y[i] * h)
                bw = int(dets.width[i] * w)
                bh = int(dets.height[i] * h)
                x1 = max(0, min(w - 1, int(cx - bw / 2)))
                y1 = max(0, min(h - 1, int(cy - bh / 2)))
                x2 = max(0, min(w - 1, int(cx + bw / 2)))
                y2 = max(0, min(h - 1, int(cy + bh / 2)))

                # 绿色检测框
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 置信度标签 (绿色, 框下方)
                conf_label = f'drone {dets.confidence[i]:.2f}'
                cv2.putText(img, conf_label, (x1, y2 + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

                # ID 标签 (红色, 框上方)
                if i < len(match_ids):
                    mid = match_ids[i]
                    if mid >= 0 and mid < len(self.ns_list):
                        id_label = self.ns_list[mid]
                        # 用醒目颜色: 匹配成功用红色
                        cv2.putText(img, id_label, (x1, max(14, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                    else:
                        # 未匹配用白色 ?
                        cv2.putText(img, '?', (x1, max(14, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
                else:
                    # 还没有匹配结果
                    cv2.putText(img, '...', (x1, max(14, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # 左上角: 本机编号 (黄色)
        cv2.putText(img, ns, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 右下角: 匹配状态
        status = 'N/A'
        if matches is not None:
            matched = sum(1 for mid in matches.id if mid >= 0)
            status = f'match: {matched}/{matches.count}'
        cv2.putText(img, status, (w - 100, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        return img

    def _compose_grid(self):
        frames = [self._draw_frame(ns) for ns in self.ns_list]
        top = np.hstack([frames[0], frames[1]])
        bottom = np.hstack([frames[2], frames[3]])
        return np.vstack([top, bottom])

    def spin(self):
        rate = rospy.Rate(10)
        cv2.namedWindow('ID Match — 4 UAV View', cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            grid = self._compose_grid()
            cv2.imshow('ID Match — 4 UAV View', grid)
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord('q'):
                rospy.signal_shutdown('User exit')
                break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('id_match_test')
        node = IdMatchTest()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
