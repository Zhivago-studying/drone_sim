#!/usr/bin/env python3
"""
@package image.py
无人机视觉检测节点 (drone_detector)

=========================== 功能描述 ===========================
订阅机载相机原始图像，加载 YOLOv7-tiny ONNX 模型进行实时目标检测。
对 320×240 输入图像做 letterbox 填充至 320×320，送入模型推理，
对 6300 个候选框进行阈值过滤 + NMS 非极大值抑制，最终输出归一化
检测框到 image_detection 话题。

=========================== 数据流 ===========================
订阅:
  camera/image_raw  (sensor_msgs/Image)
    - 机载前视相机图像 (320×240, BGR, 10Hz)
    - 在命名空间 /iris_X 下自动拼接为 /iris_X/camera/image_raw

发布:
  image_detection  (sensors/ImageDetection)
    - count:      检测到的无人机数量
    - x[], y[]:   归一化检测框中心坐标 [0-1] (相对于 320×240)
    - width[], height[]: 归一化检测框宽高 [0-1]
    - confidence[]: 综合置信度 (obj_conf × cls_conf)
    - 发布频率: 15 Hz (高于相机帧率，始终发布最新结果)

=========================== 内部处理流程 ===========================
  1. Image msg → cv2::Mat (cv_bridge, BGR888)
  2. Letterbox: 320×240 → 320×320 (上下各填充 40px 灰色 114)
  3. blobFromImage: 归一化 1/255, 通道交换 RGB
  4. onnxruntime 推理 → (1, 6300, 6) 输出张量
  5. 置信度过滤 (obj_conf × cls_conf > 0.5)
  6. OpenCV NMS (IoU 阈值 0.45)
  7. 坐标映射回 320×240 原始图像空间 + 归一化 [0-1]
  8. 发布 ImageDetection 消息

=========================== 可配置参数 ===========================
  ~model_path        str   ONNX 模型路径 (默认 best.onnx)
  ~conf_threshold    float 置信度阈值 (默认 0.5)
  ~nms_threshold     float NMS IoU 阈值 (默认 0.45)

=========================== 启动方式 ===========================
  运行于各无人机命名空间下:
  <group ns="iris_X"><node pkg="sensors" type="image.py" name="drone_detector"/></group>
"""

import os
import rospy
import cv2
import numpy as np
import onnxruntime as ort
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensors.msg import ImageDetection


class DroneDetector:
    def __init__(self):
        self.bridge = CvBridge()
        ns = rospy.get_namespace().strip('/')

        # ---- 加载 ONNX 模型 (onnxruntime) ----
        model_path = rospy.get_param('~model_path', os.path.join(
            os.path.expanduser('~'), 'swarm_localization', 'yolov7',
            'runs', 'train', 'exp2', 'weights', 'best.onnx'))
        rospy.loginfo("[%s] Loading model: %s", ns, model_path)
        self.sess = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name

        # ---- 参数 ----
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.45)
        self.input_size = (320, 320)  # YOLO 模型输入

        # ---- 发布器 ----
        # 只保留最新检测结果，避免下游继续消费过期帧。
        self.det_pub = rospy.Publisher(
            'image_detection', ImageDetection, queue_size=1)

        # ---- 订阅原始图像 (10Hz camera) ----
        # YOLO推理慢于相机频率时，queue_size=1会丢弃等待中的旧帧，
        # 将延迟限制在单帧推理时间，而不是累积约1秒的历史图像。
        self.sub = rospy.Subscriber(
            'camera/image_raw', Image, self.callback,
            queue_size=1, buff_size=2**24)

        rospy.loginfo("[%s] Detector ready (conf=%.2f, nms=%.2f)",
                      ns, self.conf_threshold, self.nms_threshold)

    # ------------------------------------------------------------------
    def letterbox(self, img, color=114):
        """将 320×240 上下填充为 320×320，保持宽高比"""
        h, w = img.shape[:2]
        canvas = np.full((self.input_size[1], self.input_size[0], 3),
                         color, dtype=np.uint8)
        y_offset = (self.input_size[1] - h) // 2
        canvas[y_offset:y_offset + h, :w] = img
        return canvas, y_offset

    # ------------------------------------------------------------------
    def process_output(self, predictions, y_offset):
        """
        predictions: ndarray (6300, 6)
          [cx, cy, w, h, obj_conf, cls_conf]
        cx,cy,w,h 在 320x320 像素空间 → 映射回原始 320x240 并归一化 [0,1]
        """
        # 综合置信度 (单类 drone: obj_conf * cls_conf)
        scores = predictions[:, 4] * predictions[:, 5]

        mask = scores > self.conf_threshold
        if not np.any(mask):
            return []

        boxes = predictions[mask]
        scores = scores[mask]

        # 准备 NMS 输入 (x1, y1, w, h)
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2

        nms_in = np.column_stack([x1, y1, x2 - x1, y2 - y1]).tolist()
        indices = cv2.dnn.NMSBoxes(nms_in, scores.tolist(),
                                   self.conf_threshold, self.nms_threshold)

        if indices is None or len(indices) == 0:
            return []

        # 展平索引 (兼容 OpenCV 4.x 不同版本)
        indices = np.asarray(indices).reshape(-1).tolist()

        W, H = 320.0, 240.0  # 原始图像尺寸
        results = []
        for i in indices:
            cx = boxes[i, 0]
            cy = boxes[i, 1] - y_offset   # 减去上填充，映射到原始图像
            bw = boxes[i, 2]
            bh = boxes[i, 3]

            results.append((
                float(np.clip(cx / W, 0, 1)),
                float(np.clip(cy / H, 0, 1)),
                float(np.clip(bw / W, 0, 1)),
                float(np.clip(bh / H, 0, 1)),
                float(scores[i]),
            ))
        return results

    # ------------------------------------------------------------------
    def callback(self, msg):
        try:
            # ROS Image → OpenCV BGR
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Letterbox 填充至 320×320
            input_img, y_offset = self.letterbox(cv_img)

            # ONNX 推理 (blob: 1x3x320x320)
            blob = cv2.dnn.blobFromImage(input_img, 1 / 255.0,
                                         self.input_size, swapRB=True)
            out = self.sess.run(None, {self.input_name: blob})[0]  # (1,6300,6)

            # 后处理
            dets = self.process_output(out[0], y_offset)

            # 发布检测结果
            det_msg = ImageDetection()
            # 完整保留原图像的曝光时间、seq和frame_id。
            det_msg.header = msg.header
            det_msg.count = len(dets)
            for cx, cy, bw, bh, conf in dets:
                det_msg.x.append(cx)
                det_msg.y.append(cy)
                det_msg.width.append(bw)
                det_msg.height.append(bh)
                det_msg.confidence.append(conf)

            self.det_pub.publish(det_msg)

        except Exception as e:
            rospy.logwarn_throttle(5.0, "Detection error: %s", e)

    # ------------------------------------------------------------------
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('drone_detector')
        node = DroneDetector()
        node.spin()
    except rospy.ROSInterruptException:
        pass
