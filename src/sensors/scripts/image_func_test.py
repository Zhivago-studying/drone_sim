#!/usr/bin/env python3
"""
@package image_func_test.py
四机相机画面实时可视化节点 (image_func_test)

=========================== 功能描述 ===========================
同时订阅 4 架无人机的原始相机图像和视觉检测结果，在 2×2 网格
窗口中实时显示。每格绘制对应无人机的检测框、置信度标签和编号，
用于直观验证 detection pipeline 的正确性。

=========================== 数据流 ===========================
订阅 (每架无人机):
  /iris_X/camera/image_raw   (sensor_msgs/Image)
    - 前视相机原始图像 320×240, BGR, 10Hz
  /iris_X/image_detection    (sensors/ImageDetection)
    - DroneDetector 发布的归一化检测框

显示:
  窗口 "4 UAV Detection", 2×2 网格, 10Hz 刷新
  每格内容:
    - 左上角: 黄色无人机编号 (iris_0 ~ iris_3)
    - 绿色矩形框: 检测到的无人机位置
    - 绿色标签: "drone 0.xx" (置信度)
    - 若无图像: 灰色背景 + "Waiting..." 文字

交互:
    ESC / q 键 → 退出程序

=========================== 关键参数 ===========================
  - 刷新频率: 10 Hz
  - 网格尺寸: 2×2, 每格 320×240, 总计 640×480
  - 非阻塞窗口 (cv2.WINDOW_NORMAL, 可缩放)

=========================== 启动方式 ===========================
  独立节点，无命名空间:
  <node pkg="sensors" type="image_func_test.py" name="image_func_test"/>
"""
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensors.msg import ImageDetection


class ImageFuncTest:
    def __init__(self):
        self.bridge = CvBridge()
        self.ns_list = ['iris_0', 'iris_1', 'iris_2', 'iris_3']

        self.latest_images = {}
        self.latest_dets = {}

        for ns in self.ns_list:
            self.latest_images[ns] = None
            self.latest_dets[ns] = None
            rospy.Subscriber(f'/{ns}/camera/image_raw', Image,
                             self.img_cb, callback_args=ns,
                             buff_size=2**24)
            rospy.Subscriber(f'/{ns}/image_detection', ImageDetection,
                             self.det_cb, callback_args=ns)

        rospy.loginfo("ImageFuncTest started — 4 UAV camera + detection viewer")

    def img_cb(self, msg, ns):
        try:
            self.latest_images[ns] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[%s] img decode error: %s', ns, e)

    def det_cb(self, msg, ns):
        self.latest_dets[ns] = msg

    def _draw_frame(self, ns):
        """在图像上绘制检测框，返回带标注的 BGR 图"""
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
        if dets is not None and dets.count > 0:
            for i in range(dets.count):
                cx = int(dets.x[i] * w)
                cy = int(dets.y[i] * h)
                bw = int(dets.width[i] * w)
                bh = int(dets.height[i] * h)
                x1 = int(cx - bw / 2)
                y1 = int(cy - bh / 2)
                x2 = int(cx + bw / 2)
                y2 = int(cy + bh / 2)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'drone {dets.confidence[i]:.2f}'
                cv2.putText(img, label, (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.putText(img, ns, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return img

    def _compose_grid(self):
        frames = [self._draw_frame(ns) for ns in self.ns_list]
        top = np.hstack([frames[0], frames[1]])
        bottom = np.hstack([frames[2], frames[3]])
        return np.vstack([top, bottom])

    def spin(self):
        rate = rospy.Rate(10)
        cv2.namedWindow('4 UAV Detection', cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            grid = self._compose_grid()
            cv2.imshow('4 UAV Detection', grid)
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord('q'):
                rospy.signal_shutdown('User exit')
                break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node('image_func_test')
        node = ImageFuncTest()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
