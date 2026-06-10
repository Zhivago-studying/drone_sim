# drone_sim
2026年3月至7月份，复现论文：Onboard cooperative relative positioning system for Micro-UAV swarm在XTdrone上的仿真项目
## 一、YOLOv7训练数据（2026年6月7日）
### 1.训练结果
#### 1.1 最佳训练结果
| 指标 | 数值 |
| :--- | --: |
| Best Epoch | 396 / 400 |
| Precision | **0.9929** |
| Recall | **0.9813** |
| mAP@0.5 | **0.9949** |
| mAP@0.5:0.95 | **0.6321** |
| Box Loss | 0.02549 |
| Object Loss | 0.002584 |
| Classification Loss | 0 |
| GPU Memory | 0.95 GB |
#### 1.2 精度曲线
- **map_0.5**

<img src="yolov7_result_png/mAP_0.5.png" width="800">

- **map_0.5:0.95**

<img src="yolov7_result_png/mAP_0.5:0.95.png" width="800">

- **Precision**

<img src="yolov7_result_png/Precision.png" width="800">

- **Recall**

<img src="yolov7_result_png/Recall.png" width="800">

#### 1.3 损失曲线
**训练集损失**

<img src="yolov7_result_png/train_loss.png" width="800">

**验证集损失**

<img src="yolov7_result_png/val_loss.png" width="800">

#### 1.4 预测效果

| <img src="yolov7/runs/train/exp2/test_batch1_pred.jpg" width="500"> | <img src="yolov7/runs/train/exp2/test_batch2_pred.jpg" width="500"> |

#### 1.5 模型参数规模

| 指标 | 值 |
| :--- | :--- |
| 总参数量 | 6,007,596 (~6M) |
| 模型大小 (float32) | 22.92 MB |
| ONNX 导出大小 | 24 MB |
| 网络层数 | 208 层 |
| 检测头 | IDetect（3 个尺度） |
| 识别类别 | 1 类：drone |
| 输入尺寸 | 320×320 |
| 下采样倍数 | 32 |                                                      
 #### 1.6 编队飞行识别效果视频
<video controls src="yolov7_result_files/xtdrone_ImageDetect_sim1.mp4" title="Title"></video>
由于影子干扰模型的识别，去除仿真环境中的阴影之后重新仿真：
<video controls src="yolov7_result_files/xtdrone_ImageDetect_sim2.mp4" title="Title"></video>