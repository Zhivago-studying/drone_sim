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

<img src="yolov7_result_files/mAP_0.5.png" width="800">

- **map_0.5:0.95**

<img src="yolov7_result_files/mAP_0.5:0.95.png" width="800">

- **Precision**

<img src="yolov7_result_files/Precision.png" width="800">

- **Recall**

<img src="yolov7_result_files/Recall.png" width="800">

#### 1.3 损失曲线
**训练集损失**

<img src="yolov7_result_files/train_loss.png" width="800">

**验证集损失**

<img src="yolov7_result_files/val_loss.png" width="800">

#### 1.4 预测效果

| <img src="yolov7_result_files/test_batch1_pred.jpg" width="500"> | <img src="yolov7_result_files/test_batch2_pred.jpg" width="500"> |

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

## 二、EKF算法结果分析
### 1.EKF估计轨迹与真值的三维对比可视化
   指标             四机平均值
  ━━━━━━━━━━━━━━━  ━━━━━━━━━━━━
   平均采样数             4435
  ───────────────  ────────────
   平均时长            81.80 s
  ───────────────  ────────────
   ATE RMSE           1.2418 m
  ───────────────  ────────────
   漂移率           0.0152 m/s
  ───────────────  ────────────
   漂移率           0.91 m/min
  ───────────────  ────────────
   RPE @ 1s           0.4201 m
  ───────────────  ────────────
   RPE @ 5s           1.4955 m
  ───────────────  ────────────
   RPE @ 10s          1.6802 m
  ───────────────  ────────────
   姿态误差 RMSE    0.2335 deg

###


## 三、实验过程中的思路、遇到的问题以及解决思路
### 1.目的：测量相对角度计算误差
大致思路：
**相对角度计算：**
- 使用节点：image.py、camera_relative_angle_cal.cpp
- 发布话题：/iris_X/camera_angle（enu坐标系）
- 话题内容：
std_msgs/Header header
uint8 count            # number of detected objects
float32[] alpha        # horizontal bearing angle (radians), atan2(y, x)
float32[] theta        # elevation angle (radians), asin(z / mag)
**误差计算：**
test/src/angle_error_cal.cpp:
1. 订阅话题：gazebo/model_states，获取本地无人机iris_i和其他无人机iris_j真实位置Pi、Pj;
2. 订阅话题：/iris_i/camera_angle（enu坐标系）获取本地无人机检测结果 
3. 在gazebo/model_states回调函数中计算相对向量：Pij=Pj-Pi,计算相对角度alpha_gt、theta_gt
4. 在/iris_i/camera_angle回调函数中遍历检测结果，计算e=sqrt(alpha_error^2+theta_error^2)，并限制到[−π,π]，保存每一时刻的误差到.csv文件，我需要后续进行误差分析
5. 结束时统计RMSE_alpha和RMSE_theta
**环境文件：**
复制outdoor2.launch为outdoor2_copy.launch，修改outdoor2_copy.launch，只保留iris_0和iris_1
**飞行控制节点：**
test/src/two_uavs_formation.cpp:
飞机：iris_0和iris_1：
stage1: 起飞至3m
stage2: 保持静止和悬停3s
stage3: 径向扩展并收缩回原来的位置
satge4: 以两家无人机中心为原点，绕原点旋转180度，再返回
stage5: 着陆
**启动文件：**
test/launch/angle_error_test.launch:启动iris_0和iris_1，运行image.py、camera_relative_angle_cal.cpp、angle_error_cal.cpp、two_uavs_formation节点
#### （1）第一次测量角度
##### 测量结果：

<img src="test_results/angle_error_1.png" width="400">

##### 问题：
1. iris_0检测iris_1角度误差比iris_1检测iris_0的角度误差大很多
2. alpha和theta的误差也相差较大

##### 对策

1. 可视化角度测量
目的：检测相对角度计算是否合理可靠
调用image.py识别无人机信息->camera_relative_angle_cal.cpp计算相对角度->计算真实相对角度->在无人机识别框中标注angle_est/angle_gt（以度形式）  
结果：

<img src="test_results/angle_show_1.png" width="400">

可以发现相对角度测量结果完全不对，其中angle_show.py中标注的angle_est直接来自于话题iris_X/camera_angle，说明相对角度的计算有问题；
终端调用命令：watch -n 1 "rostopic echo /iris_0/camera_angle显示：
> header:  
> seq: 53  
> stamp:  
>  secs: 722  
>  nsecs:  10000000  
> frame_id: "map"  
> count: 1  
> alpha: [-1.2407687902450562]  
> theta: [1.5640960931777954]  

这里frame_id:"map"，说明相对角度计算是在map（全局坐标系下的），应该是enu坐标系，初步判断是坐标系转换出现问题，问题转交给claude code，发现代码中存在坐标系转换问题：
> detCallback 中第 92 行的 ray_cam 处于光学帧（Z=前，X=右，Y=下），但第 79 行 R_world_cam 是把camera_link FLU（X=前，Y=左，Z=上）旋转到 ENU。直接用光学帧向量乘以 FLU→ENU 矩阵，导致 Z方向分量（光学帧的前向）被误当作 FLU 的向上，使得 theta 始终接近 90°。

修改代码重新运行，最终效果：

<img src="test_results/angle_show_final.png" width="400">

可见角度计算基本正确，重新测试角度误差，结果如下：

<img src="test_results/angle_error_final.png" width="400">

角度误差在合理范围内，问题解决！
### 2.目的：提高id匹配的准确率
#### 采取的措施：
1. 添加初始时刻各个无人机之间的相对向量，因为是无人机编队飞行，初始时刻设置的无人机之间的距离和方向都是已知的，添加先验信息之后id匹配检测准确率大幅提高，准确率估计基本在90%
2. 添加连续帧检测之后检测成功率再次提高，但是会出现同一帧中的多个检测框匹配为同一无人机ID
3. 添加判断：同一 ID 只保留误差最小的那个检测框，其余置为 -1，再次测试，成功率接近100%，并且将影子也排除在外！
### 3.目的：实现通信节点模拟
每一个无人机广播的内容包括：
1. 位置估计，来源于iris_X/ins_estimate
2. 水平速度V^{flow}
通信频率：10Hz
实现思路：
sensors/msg/ComMsg.msg:
std_msgs/Header header
uint8 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
sensors/src/communication.cpp:
对于本机i：
   订阅话题iris_X/ins_estimate
   构建消息：
      写入速度和位置还有时间戳信息和id信息
以10Hz频率发布话题iris_i/communication
### 4.目的：实现DGO图优化
#### （1）数据预处理
设本机为i
for j in N:
   订阅话题iris_j/communication、iris_i/ins_estimate
   解析话题消息并分别将速度和位置打包为v_est和p_est，其中v_est的格式：(vx,vz,0,id)，p_est的格式：（px,py,pz,id）
   计算补偿之后的位置：
      pc = p_est + v_est*dt（dt为通信延迟100ms）
   将pc装进集合Pc中，Pc表示所有无人机的补偿位置信息集合
## 附：遇到的一些问题
### 1.在实现id匹配时，识别准确率很低
