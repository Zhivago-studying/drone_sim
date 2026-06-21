# drone_sim

复现论文：**Onboard cooperative relative positioning system for Micro-UAV swarm**

> 时间：2026 年 3 月 — 7 月  
> 仿真平台：XTDrone

---

## 目录

- [一、YOLOv7 训练数据（2026 年 6 月 7 日）](#一yolov7-训练数据2026-年-6-月-7-日)
- [二、EKF 算法结果分析](#二ekf-算法结果分析)
- [三、实验过程中的思路、遇到的问题以及解决思路](#三实验过程中的思路遇到的问题以及解决思路)
- [附：遇到的一些问题](#附遇到的一些问题)
- [四、UWB 数据处理分析报告](#四uwb-数据处理分析报告)
- [五、角度误差分析报告](#五角度误差分析报告)

---

## 一、YOLOv7 训练数据（2026 年 6 月 7 日）

### 1. 训练结果

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

| | |
|---|---|
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

---

## 二、EKF 算法结果分析

### 1. EKF 估计轨迹与真值的三维对比可视化

| 指标 | 四机平均值 |
|------|-----------|
| 平均采样数 | 4435 |
| 平均时长 | 81.80 s |
| ATE RMSE | 1.2418 m |
| 漂移率 | 0.0152 m/s |
| 漂移率 | 0.91 m/min |
| RPE @ 1s | 0.4201 m |
| RPE @ 5s | 1.4955 m |
| RPE @ 10s | 1.6802 m |
| 姿态误差 RMSE | 0.2335 deg |

---

## 三、实验过程中的思路、遇到的问题以及解决思路

### 1. 目的：测量相对角度计算误差

**大致思路：**

**相对角度计算：**

- 使用节点：`image.py`、`camera_relative_angle_cal.cpp`
- 发布话题：`/iris_X/camera_angle`（enu 坐标系）
- 话题内容：

```msg
std_msgs/Header header
uint8 count            # number of detected objects
float32[] alpha        # horizontal bearing angle (radians), atan2(y, x)
float32[] theta        # elevation angle (radians), asin(z / mag)
```

**误差计算：**

`test/src/angle_error_cal.cpp`:

1. 订阅话题：`gazebo/model_states`，获取本地无人机 `iris_i` 和其他无人机 `iris_j` 真实位置 Pi、Pj；
2. 订阅话题：`/iris_i/camera_angle`（enu 坐标系）获取本地无人机检测结果；
3. 在 `gazebo/model_states` 回调函数中计算相对向量：Pij = Pj - Pi，计算相对角度 alpha_gt、theta_gt；
4. 在 `/iris_i/camera_angle` 回调函数中遍历检测结果，计算 `e = sqrt(alpha_error^2 + theta_error^2)`，并限制到 `[-π, π]`，保存每一时刻的误差到 .csv 文件，后续进行误差分析；
5. 结束时统计 RMSE_alpha 和 RMSE_theta。

**环境文件：**

复制 `outdoor2.launch` 为 `outdoor2_copy.launch`，修改 `outdoor2_copy.launch`，只保留 `iris_0` 和 `iris_1`。

**飞行控制节点：**

`test/src/two_uavs_formation.cpp`：

- 飞机：`iris_0` 和 `iris_1`
- stage1：起飞至 3m
- stage2：保持静止和悬停 3s
- stage3：径向扩展并收缩回原来的位置
- stage4：以两架无人机中心为原点，绕原点旋转 180 度，再返回
- stage5：着陆

**启动文件：**

`test/launch/angle_error_test.launch`：启动 `iris_0` 和 `iris_1`，运行 `image.py`、`camera_relative_angle_cal.cpp`、`angle_error_cal.cpp`、`two_uavs_formation` 节点。

#### （1）第一次测量角度

##### 测量结果：

<img src="test_results/angle_error_1.png" width="400">

##### 问题

1. `iris_0` 检测 `iris_1` 角度误差比 `iris_1` 检测 `iris_0` 的角度误差大很多
2. alpha 和 theta 的误差也相差较大

##### 对策

**1. 可视化角度测量**

目的：检测相对角度计算是否合理可靠。

调用 `image.py` 识别无人机信息 → `camera_relative_angle_cal.cpp` 计算相对角度 → 计算真实相对角度 → 在无人机识别框中标注 `angle_est` / `angle_gt`（以度形式）。

结果：

<img src="test_results/angle_show_1.png" width="400">

可以发现相对角度测量结果完全不对，其中 `angle_show.py` 中标注的 `angle_est` 直接来自于话题 `/iris_X/camera_angle`，说明相对角度的计算有问题。

终端调用命令：

```bash
watch -n 1 "rostopic echo /iris_0/camera_angle"
```

显示：

```
header:
  seq: 53
  stamp:
    secs: 722
    nsecs:  10000000
  frame_id: "map"
count: 1
alpha: [-1.2407687902450562]
theta: [1.5640960931777954]
```

这里 `frame_id: "map"`，说明相对角度计算是在 map（全局坐标系下的），应该是 enu 坐标系，初步判断是坐标系转换出现问题。问题转交给 Claude Code，发现代码中存在坐标系转换问题：

> `detCallback` 中第 92 行的 `ray_cam` 处于光学帧（Z=前，X=右，Y=下），但第 79 行 `R_world_cam` 是把 `camera_link` FLU（X=前，Y=左，Z=上）旋转到 ENU。直接用光学帧向量乘以 FLU→ENU 矩阵，导致 Z 方向分量（光学帧的前向）被误当作 FLU 的向上，使得 theta 始终接近 90°。

修改代码重新运行，最终效果：

<img src="test_results/angle_show_final.png" width="400">

可见角度计算基本正确，重新测试角度误差，结果如下：

<img src="test_results/angle_error_final.png" width="400">

角度误差在合理范围内，问题解决！

### 2. 目的：提高 id 匹配的准确率

#### 采取的措施

1. 添加初始时刻各个无人机之间的相对向量，因为是无人机编队飞行，初始时刻设置的无人机之间的距离和方向都是已知的，添加先验信息之后 id 匹配检测准确率大幅提高，准确率估计基本在 90%；
2. 添加连续帧检测之后检测成功率再次提高，但是会出现同一帧中的多个检测框匹配为同一无人机 ID；
3. 添加判断：同一 ID 只保留误差最小的那个检测框，其余置为 -1，再次测试，成功率接近 100%，并且将影子也排除在外！

### 3. 目的：实现通信节点模拟

每一个无人机广播的内容包括：

1. 位置估计，来源于 `/iris_X/ins_estimate`
2. 水平速度 V<sup>flow</sup>

通信频率：10Hz。

**实现思路：**

`sensors/msg/ComMsg.msg`：

```msg
std_msgs/Header header
uint8 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
```

`sensors/src/communication.cpp`：对于本机 i：

- 订阅话题 `/iris_X/ins_estimate`
- 构建消息：写入速度和位置还有时间戳信息和 id 信息
- 以 10Hz 频率发布话题 `/iris_i/communication`

### 4. 目的：实现 DGO 图优化

#### （1）数据预处理

设本机为 i：

```python
for j in N:
    订阅话题 iris_j/communication、iris_i/ins_estimate
    解析话题消息并分别将速度和位置打包为 v_est 和 p_est
        v_est 格式：(vx, vz, 0, id)
        p_est 格式：(px, py, pz, id)
    计算补偿之后的位置：
        pc = p_est + v_est * dt   # dt 为通信延迟 100ms
    将 pc 装进集合 Pc 中
# Pc 表示所有无人机的补偿位置信息集合
```

---

## 附：遇到的一些问题

### 1. 在实现 id 匹配时，识别准确率很低

（后续内容可继续补充）

---

## 四、UWB 数据处理分析报告

**数据来源**: `uwb_zero_score` 节点录制的 CSV 文件  
**参数配置**: window=5, z_score_threshold=3.0, min_stddev=0.08, min_distance=0.10, warmup_stddev_threshold=0.02, max_consecutive_rejects=5  

### 1. 数据概览

| 无人机 | 原始测量数 | 处理后数量 | 接收率 | 对齐对数 |
|--------|-----------|-----------|--------|---------|
| iris_0 | 5,694 | 5,628 | 98.8% | 5,628 |
| iris_1 | 5,655 | 5,611 | 99.2% | 5,611 |
| iris_2 | 5,697 | 5,649 | 99.2% | 5,649 |
| iris_3 | 5,697 | 5,650 | 99.2% | 5,650 |

- 消息频率: 25.0 Hz（稳定）
- 时间跨度: 173.2 → 249.1 s（约 76 秒）

### 2. 滤波精度（全局）

| 无人机 | 原始 RMSE | 滤波后 RMSE | 提升幅度 | 滤波最大误差 |
|--------|----------|-----------|---------|------------|
| iris_0 | 0.0792 m | **0.0428 m** | **46.0%** | 0.2944 m |
| iris_1 | 0.0777 m | **0.0425 m** | **45.3%** | 0.2051 m |
| iris_2 | 0.0764 m | **0.0419 m** | **45.2%** | 0.2295 m |
| iris_3 | 0.0793 m | **0.0421 m** | **47.0%** | 0.1646 m |

- 四机滤波后 RMSE 集中在 **4.2–4.3 cm**，一致性极好
- 相比原始数据，Z-score 均值滤波平均提升 **45.9%**
- 滤波后 MAE 约 3.3 cm，均值误差接近零（无偏估计）

### 3. 各目标通道误差

| 无人机 | 目标 | 数据量 | 原始 RMSE | 滤波 RMSE | 提升 |
|--------|------|--------|----------|----------|------|
| iris_0 | 1 | 1,877 | 0.0790 | 0.0422 | 46.6% |
| iris_0 | 2 | 1,869 | 0.0794 | 0.0461 | 41.9% |
| iris_0 | 3 | 1,882 | 0.0793 | 0.0399 | 49.8% |
| iris_1 | 0 | 1,869 | 0.0768 | 0.0395 | 48.5% |
| iris_1 | 2 | 1,874 | 0.0765 | 0.0403 | 47.2% |
| iris_1 | 3 | 1,868 | 0.0797 | 0.0471 | 40.9% |
| iris_2 | 0 | 1,875 | 0.0750 | 0.0435 | 42.0% |
| iris_2 | 1 | 1,883 | 0.0772 | 0.0413 | 46.4% |
| iris_2 | 3 | 1,891 | 0.0770 | 0.0407 | 47.1% |
| iris_3 | 0 | 1,881 | 0.0797 | 0.0415 | 48.0% |
| iris_3 | 1 | 1,879 | 0.0781 | 0.0438 | 43.9% |
| iris_3 | 2 | 1,890 | 0.0800 | 0.0409 | 49.0% |

所有 12 个通道滤波 RMSE 在 **0.039–0.047 m** 范围内，无异常通道。

### 4. 误差分布（滤波后绝对误差）

| 百分位 | iris_0 | iris_1 | iris_2 | iris_3 |
|--------|--------|--------|--------|--------|
| P50（中位数） | 0.0265 | 0.0272 | 0.0278 | 0.0268 |
| P75 | 0.0467 | 0.0473 | 0.0470 | 0.0468 |
| P90 | 0.0689 | 0.0678 | 0.0676 | 0.0703 |
| P95 | 0.0841 | 0.0841 | 0.0820 | 0.0846 |
| P99 | 0.1166 | 0.1183 | 0.1152 | 0.1140 |
| P100（最大） | 0.2944 | 0.2051 | 0.2295 | 0.1646 |

- 50% 的滤波误差 ≤ **2.7 cm**
- 90% 的滤波误差 ≤ **6.9 cm**
- 四机 P50–P99 高度一致，滤波器行为稳定

### 5. 异常值拒绝分析

| 无人机 | 被拒绝测量 | 拒绝率 | 被拒测量均值 | 被拒测量真值均值 |
|--------|----------|--------|-------------|----------------|
| iris_0 | 66 | 1.16% | 3.18 m | 3.20 m |
| iris_1 | 44 | 0.78% | 3.18 m | 3.20 m |
| iris_2 | 48 | 0.84% | 3.33 m | 3.29 m |
| iris_3 | 47 | 0.82% | 3.23 m | 3.18 m |

- 拒绝率 0.78%–1.16%，约 **0.9%**
- 被拒绝测量均值与 GT 均值接近，Z-score 剔除的是**瞬时噪声跳变**而非系统偏置

### 6. 与上次测试的对比

| 指标 | 上次 | 本次 | 变化 |
|------|------|------|------|
| iris_3 冷启动问题 | 有（前 0.4s 输出 0 值，导致~1.2m 误差） | 无 | warm-up 机制修复 |
| 全局滤波 RMSE | ~0.073 m（raw vs filt） | ~0.042 m（filt vs GT） | 可对 GT 评估 |
| 四机一致性 | iris_3 显著劣于其他 | 四机高度一致 | 问题解决 |
| 拒绝率 | 0.68–1.24% | 0.78–1.16% | 稳定 |
| 最大误差 | 2.28 m（iris_3） | 0.29 m | **下降 87%** |

### 7. 结论

1. **滤波显著有效**: 原始 UWB 测量 RMSE ~7.7 cm，滤波后降至 ~4.2 cm，提升幅度 **45%**
2. **算法稳健**: Z-score=3σ 阈值在保持 99% 接收率的同时有效剔除了 ~1% 的瞬时跳变
3. **冷启动修复**: iris_3 前次测试的 0 值污染问题通过 warm-up 检测彻底消除
4. **四机无差异**: 所有 12 个通道滤波精度在 0.039–0.047 m 范围，系统一致性优秀

---

## 五、角度误差分析报告

**节点链**: `image.py` → `camera_relative_angle_cal` → `angle_error_cal`
**参数**: max_match_error=1.0 rad, tf_timeout=30ms, output: csv_dir=src/test/logs/

### 1. 运行测试结果 (2026-06-16)

| 指标 | iris_0 → iris_1 | iris_1 → iris_0 |
|------|-----------------|-----------------|
| 有效样本 | **499** | **23** |
| 超出阈值 | 0 | 0 |
| RMSE_alpha | **0.0175 rad (1.00°)** | **0.0333 rad (1.91°)** |
| RMSE_theta | **0.0693 rad (3.97°)** | **0.0027 rad (0.15°)** |
| 检测时间段 | 6.1–67.6s (61.5s 连续) | 48.1–50.3s (2.2s 间断) |

### 2. iris_0 → iris_1（正常通道）

- **499 个有效样本**，覆盖整个编队飞行过程（起飞、悬停、扩张收缩、旋转、降落）
- alpha RMSE 1.00°，theta RMSE 3.97°
- theta 误差大于 alpha（3.97° vs 1.00°），符合俯仰角从单目视觉投影恢复精度低于方位角的规律
- 最大误差 0.45 rad (25.7°)，中位数误差 0.64°

### 3. iris_1 → iris_0（稀疏通道）

- **仅 23 个样本**，集中在 t=48.1–50.3s（ROTATE 阶段）
- alpha RMSE 1.91°，theta RMSE 0.15°
- 检测稀疏根因：**前视相机几何限制**

  `iris_0` 位于 (0, 0)，`iris_1` 位于 (2, 0)，两机初始航向均朝 +X（东）
  - iris_0 的前视相机始终能看到前方的 iris_1 → 连续检测
  - iris_1 的前视相机中，iris_0 在后方 (−2, 0) → 大部分时间不可见
  - 仅在 ROTATE 阶段两机绕中心旋转、航向变化时 iris_1 短暂看到 iris_0

### 4. 修复记录

| 问题 | 修复 | 效果 |
|------|------|------|
| 启动期 TF 未就绪，camera_angle 输出 NaN 导致 RMSE=nan | `angle_error_cal.cpp`: 累积时 `std::isfinite()` 跳过 NaN | RMSE 恢复正常 |
| TF 失败时 camera_relative_angle_cal 发布 NaN 角度 | 改为跳过该帧不发布（等待 TF 就绪） | 下游不再接收 NaN |
| iris_1 检测极其稀疏 | **非代码 bug**，是前视相机物理限制 | 多机 swarm 中其余机可互补覆盖 |

### 5. 结论

- **alpha 角度精度 ~1–2°**，满足相对定位系统需求
- **theta 角度精度 ~4°**（俯仰角），精度低于 alpha 但可接受
- 单目前视相机的盲区导致部分通道检测稀疏，4 机 swarm 的冗余观测可缓解
- 修复后的 NaN 问题已根除，RMSE 统计不再被启动期垃圾数据污染




## 六、数据记录

所有实验数据保存于 `run_data/run_X/` 目录下,按节点类型分类(dgo/、ekf_dgo_test/、ins_eskf/、ins_eskf_test/、uwb_zero_score/)。

### run_6 ~ run_8: 相机队列深度优化

修改 `image.py`:

- `subscriber.queue_size: 10 → 1` — 只保留最新帧,旧帧丢弃。原先 queue_size=10 时,若 YOLO CPU 推理慢于 10Hz,旧图像会排队约 1 秒,导致 `camera_age` p50≈1.0s。
- 去掉 15Hz 发布节流,改为处理每一帧到达的图像。
- `publisher.queue_size: 10 → 1`。

效果:相机延迟从 ~1.0s 下降到单帧推理时间(~60ms),`camera_age` p50 从 1.0s 降至 <0.1s,相机约束使用率提升。

### run_9 ~ run_11: ToF 高度观测计算改进

修改 `ins_eskf.cpp`:

1. **重写 ESKF 参数管理**:新增 `tof_noise_std`、`tof_min_range`、`flow_relative_noise_std`、`initial_attitude_std_deg` 等 17 项参数的显式读取与合法性校验,避免运行时使用无效参数。
2. **ToF 饱和检测**:当 ToF 读数小于 `tof_min_range + 0.03m` 时判定为饱和,将高度初始值 `height0_` 置零而非使用不可靠的测距值,避免起飞时引入高度偏置。
3. **随机种子优化**:每架无人机使用命名空间哈希异或 `measurement_noise_seed` 作为随机种子,保证可复现的同时各机噪声独立。
4. **发布协方差缩放调整**:`publish_pos_cov_scale: 5.0 → 15.0`, `publish_vel_cov_scale: 1.0 → 3.0`,改善 DGO 对 INS 协方差的信任度。

### 数据可用性

| run | 修改内容 | dgo iter | 数据路径 |
|-----|---------|----------|---------|
| 3~5 | EKF 优化后基线 | ~700 | `run_data/run_3~5/` |
| 6~8 | camera queue_size=1 | ~700 | `run_data/run_6~8/` |
| 9~11 | ToF 高度观测改进 | ~700 | `run_data/run_9~11/` |

绘图:

```bash
# 激活虚拟环境后
source ~/swarm_localization/venv/bin/activate
python3 src/test/scripts/dgo_plot.py --run-id 11 --no-show
python3 src/test/scripts/dgo_ekf_plot.py --run-id 11 --no-show
```
