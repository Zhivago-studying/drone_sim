# drone_sim

复现论文：**Onboard cooperative relative positioning system for Micro-UAV swarm**

> 时间：2026 年 3 月 — 7 月  
> 仿真平台：XTDrone

---

## 目录

- [一、YOLOv7 训练数据（2026 年 6 月 7 日）](#一yolov7-训练数据2026-年-6-月-7-日)
- [二、EKF 算法结果分析](#二ekf-算法结果分析)
- [三、实验过程中的思路、遇到的问题以及解决思路](#三实验过程中的思路遇到的问题以及解决思路)
- [四、UWB 数据处理分析报告](#四uwb-数据处理分析报告)
- [五、角度误差分析报告](#五角度误差分析报告)
- [六、数据记录](#六数据记录)
- [run_21~run_25](#run_21run_25)
- [run_26~run_27](#run_26run_27)
- [run_28~run_31](#run_28run_31-dgo-融合历元时间系统重构)
- [run_34~run_35](#run_34run_35-任务完成自动停止--速度源修正--evaluator-飞行窗口停止)

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
| 12~20 | 飞行窗口过滤 + compare_runs | ~650 | `run_data/run_12~20/` |
| 21~27 | 通信延迟优化 + camera 时序修正 | ~500 | `run_data/run_21~27/` |
| 28~33 | DGO算法历元时间系统重构+阶段自适应 UWB 权重 + 外推限幅 | ~500 | `run_data/run_28~33/` |
| 34~35 | DGO 任务完成自动停止 + 速度源修正 + evaluator 飞行窗口停止 | ~450 | `run_data/run_34~35/` |

### run_21~run_25

#### 1.问题
run_21:
1. com与gt误差在40s之后误差较大
2. com_stamp_dt大多数超过50ms，邻机通信延迟较大
run_22:
1. com与gt误差在65s之后发散严重
2. 部分无人机之间通信延迟散布较大，不稳定
run_23:
1. com与gt误差在80s之后散布严重
run_24:
1. 仿真时间达120s，过长
2. com与gt误差在80s之后振荡严重，尤其是XY轴
3. 通信延迟比较大，大量数据com_stamp_dt超过20ms
run_25:
1. com与gt误差在40s之后振荡较大，多个Z轴误差60s之后急剧增大

### 2.我的想法
首先最关键的是统一仿真时间，应该在起飞时开始，land时结束

### 3.实行的改动
目的：统一仿真时间基准线
**核心思路**：不改 C++ 节点，在后处理脚本中通过 DGO `sync_diag.csv` 的 `mission_stage` 字段进行飞行窗口过滤（排除 stage 0=WAIT、stage 7=DONE），实现统一仿真时间基准。

#### （1）新增脚本：`compare_runs.py` — 跨 run 飞行窗口对比

```bash
# 单 run 分析
python3 src/test/scripts/compare_runs.py 21

# 多 run 对比
python3 src/test/scripts/compare_runs.py 19 20 21

# 范围指定
python3 src/test/scripts/compare_runs.py --run-id-range 3 14

# 禁用飞行窗口过滤（使用全部数据）
python3 src/test/scripts/compare_runs.py --no-flight-only 21
```

输出示例：
```
  run   dur(s)   DGO_RMSE   EKF_RMSE   DGO/EKF         verdict
   19     45.2     0.1205     0.1667      0.72      DGO better
   20     45.0     0.1130     0.2096      0.54      DGO better
   21     45.0     0.5717     0.1176      4.86      EKF better
```

**核心逻辑**：
- `load_stage_map()` — 读取 DGO `sync_diag.csv` 的 `(stamp, mission_stage)` 映射
- `forward_fill_stage()` — 通过 `np.searchsorted` 将 EKF/DGO 数据时间戳对齐到最近的 stage 值，早于 DGO 起始的时间戳标记为 `-1`（pre-flight）
- `STAGES_EXCLUDE = {0, 7}` — 语义固定的排除集合（WAIT、DONE），不依赖 auto-detect
- **降级路径**：sync_diag 为空/仅 header 时打印 Warning，退化为无过滤模式

#### （2）修改脚本：`dgo_ekf_plot.py` — 新增 `--flight-only` 参数

新增参数：
- `--flight-only`（默认开启）— 仅使用飞行窗口数据计算 DGO/EKF 误差图
- `--no-flight-only` — 使用全部数据（兼容旧行为）

`load_method()` 增加可选的 `stage_map` 参数，在加载每份 CSV 后通过 `filter_flight_window()` 剔除 stage 7（DONE）后的数据。

#### （3）修改脚本：`dgo_plot.py` — 阶段背景 overlay

`add_stage_background()` 在每张子图的背景层绘制彩色垂直条，表示每个 `mission_stage` 的时段：

| Stage | 颜色 | 含义 |
|:-----:|:----:|:----|
| 0 | 🔘 gray | WAIT (初始化, 仿真未开始) |
| 1 | 🟢 green | TAKEOFF (爬升至 3m) |
| 2 | 🟡 yellow | HOVER (悬停 2s) |
| 3 | 🔵 cyan | EXPAND (径向扩张→收缩, 10s) |
| 4 | 🟠 orange | TRANSLATE (整体平移→返回, 10s) |
| 5 | 🩷 pink | CHASE (位置循环交换→复原, 10s) |
| 6 | 🔴 red | LAND (下降至地面) |
| 7 | ⚪ gray | DONE (编队解散) |

#### （4）关键技术决策

| 决策 | 选择 | 理由 |
|:---|:---|:---|
| 排除集合 | `{0, 7}` 硬编码 | stage 语义固定（WAIT/DONE），set intersection 自动处理缺失的 stage 0 |
| 时间对齐 | forward-fill (np.searchsorted) | EKF 100Hz 数据需对齐到 DGO ~10Hz 的 sync_diag |
| Pre-DGO 数据 | 标记为 -1，统一排除 | 起飞前 EKF 初始收敛期约 8s（t=12.9→20.9s） |
| 降级策略 | Warning + 退化 | 空/损坏的 sync_diag 不阻止分析 |
| C++ 节点 | 不改 | 纯后处理，零风险

### run_26~run_27: 通信延迟优化 + 相机时序修正验证

#### 优化效果

| 指标 | run_26 (飞行窗口) | run_27 (飞行窗口) |
|------|:-----------------:|:-----------------:|
| DGO RMSE (3机均值) | **0.108m** | **0.159m** |
| EKF RMSE (3机均值) | **0.130m** | **0.223m** |
| DGO/EKF ratio | **0.83** | **0.71** |
| camera fresh率(iris_0) | 99.4% | 99.4% |
| camera used率(iris_0) | 89.0% | 89.2% |
| 飞行窗口时长 | 45.2s | 45.0s |

### run_28~run_31: DGO 融合历元时间系统重构

#### 阶段一：DGO 内部时间同步

##### 现象

各节点 `sync_ref_time_` 存在大量倒退（rollback）：

| run | 倒退率 |
|:---|:---:|
| 22/25/27 | 约 26%～30% |
| 23/24 | 部分节点超过 20% |

- 通信时间差 p95 多数为 **0.10～0.19 s**
- DGO 输出时间来自邻机通信时间的平均，而通信本身广播的是上一轮 DGO 时间，形成循环：

```
peer DGO stamp → communication stamp → 本机计算平均 sync_ref_time → 发布新的旧时间 DGO
```

这也是 DGO 日志看似 10 Hz，但参考时刻经常每 0.3 s 才前进一步的原因。

##### 初始化没有时间对齐

首次启动时：
```cpp
P_opt_ = latest_ins_msg_->pose.pose.position;  // 位置：latest INS 时刻
// 但最终发布的时间戳是 sync_ref_time_      // 时间戳：较旧的邻机同步时刻
```

这解释了 run_25、run_27 的固定 Z 偏差：

| run_27 | DGO Z - EKF Z |
|:---|---:|
| iris_1 | -0.105 m |
| iris_2 | -0.093 m |
| iris_3 | -0.065 m |

##### 优化方案

建立独立、单调的融合历元时间基准：

```text
t_fuse = floor((now - fixed_lag) / fusion_period) × fusion_period
```

要求：
- `t_fuse > last_fuse_time_`（单调递增）
- 不再使用邻机 DGO 时间戳平均值决定融合历元
- INS、UWB、camera 都选择最接近 `t_fuse` 的样本
- 邻机状态利用通信时间戳和速度外推到 `t_fuse`
- 首次初始化必须使用 `selectNearestIns(t_fuse)` 的位置，不能使用 `latest_ins_msg_`

##### 验收标准

| 项目 | 目标 |
|:---|:---:|
| `sync_ref_time_` 倒退率 | 0% |
| 四机融合历元差 p95 | < 20 ms |
| DGO evaluator 配对成功率 | > 95% |
| DGO Z 与 EKF Z 固定偏移 | < 0.03 m |

**问题**：DGO 的参考时间 `sync_ref_time_` 由邻机通信时间戳均值决定（`updateReferenceTime()`），导致：
- DGO stamp 随邻机通信抖动而反复横跳
- 首次初始化直接用 `latest_ins_msg_`，position 和 stamp 来自不同时间点
- evaluator 四机配对成功率低

**方案**：建立独立的本机融合历元生成器，DGO 时间完全由本机决定。

#### 核心参数

| 参数 | 默认值 | 含义 |
|:---|:---:|:---|
| `fusion_period` | 0.10 s | DGO 固定融合周期（10 Hz 网格） |
| `fixed_lag` | 0.20 s | 固定滞后窗口，DGO 处理 `now - 0.20s` 附近的历史样本 |
| `max_fuse_retry_lag` | 1.0 s | 单个历元因缺数据最多重试多久，超时丢弃 |
| `history_keep_time` | 2.0 s | INS/UWB/camera/GT 缓存保留时间 |

#### 17 步修改总览

| 步骤 | 改动 | 文件 |
|:---:|:---|:---:|
| 1 | 构造函数新增 4 个融合历元参数 + 合法性校验 | `DGO.cpp` |
| 2 | 新增 `last_fuse_time_` / `dropped_fuse_epoch_count_` / `rejected_fuse_epoch_count_` / `prev_ins_stamp_` 等状态变量 | `DGO.cpp` |
| 3 | `floorToFusionGrid()` + `computeNextFuseTime()` — 固定历元生成 | `DGO.cpp` |
| 4 | `dgoTimerCallback()` — 先 `computeNextFuseTime()`，传入 `isReadyForDGO(t_fuse)` | `DGO.cpp` |
| 5 | `isReadyForDGO(t_fuse)` — 移除 `updateReferenceTime()` 和 `latest_ins_msg_` 依赖 | `DGO.cpp` |
| 6 | 首次初始化使用 `best_ins_pos_`（来自 `selectNearestIns(t_fuse)`），position/stamp 一致 | `DGO.cpp` |
| 7 | 通信检查改为 `t_fuse` 对齐，移除邻机均值 + skew 检查 | `DGO.cpp` |
| 8 | `updatePredictedPeerPositions(t_fuse)` — 传参改为 `t_fuse` + DEBUG 日志 | `DGO.cpp` |
| 9 | INS 缓存从 `pair<Time, Point>` 改为完整 `InsSample` 结构（含 Odometry） | `DGO.cpp` |
| 10 | 缓存裁剪从固定条数 → 时间窗口 `pruneStampedHistory(history_keep_time=2.0s)` | `DGO.cpp` |
| 11 | 所有 `selectNearest*()` 使用 `t_fuse` 而非 `sync_ref_time_` | `DGO.cpp` |
| 12 | `prepareInsDelta()` — 新增 INS stamp 单调性检查 + DEBUG 日志 | `DGO.cpp` |
| 13 | `publishDgoEstimate()` — 零 stamp 不发布 + 单调性检查 + 使用 `best_ins_` | `DGO.cpp` |
| 14 | communication `max_dgo_age` 从 0.3s 放宽至 0.5s，配合 `fixed_lag=0.20` | `communication.cpp` |
| 15 | sync_diag CSV 新增 `last_fuse_time`/`fuse_step`/`fuse_backtrack`/`dropped_fuse_count`/`rejected_fuse_count` | `DGO.cpp` |
| 16 | `RunDGO()` 改为返回 `bool`，仅 `published=true` 时推进 `last_fuse_time_` | `DGO.cpp` |
| 17 | launch 文件新增融合历元参数 + `max_sensor_age=0.3` + `max_extrapolation_dt=0.30` | `dgo_full_mission.launch` |

#### Review 修复的 5 个问题

| # | 问题 | 修改 |
|:---:|:---|:---|
| P0 | `RunDGO()` 失败但 `last_fuse_time_` 仍推进 | 改为返回 `bool`；仅 `published=true` 时推进 |
| P1.1 | `selectNearestIns` 每次设 `ins_update_=true`，缺时间单调性检查 | `prepareInsDelta` 检查 `best_ins_stamp_ > prev_ins_stamp_`，`dt ≤ 1e-6` 时拒绝 |
| P1.2 | `max_dgo_age=0.3` 与 `fixed_lag=0.20` 配合不足 | 默认 `0.3→0.5`，满足 `fixed_lag + 2×fusion_period = 0.40` |
| P1.3 | `max_extrapolation_dt` 源码默认仍 `0.2` | `0.2→0.30`，与 launch 一致 |
| P1.4 | fallback 路径缺少 CSV 写入 | catch fallback 中调用 `writeResidualDebugCsv("fallback")` + sync/comm CSV |
| P1.5 | `rejected_fuse_count` 重复计数 | 增加 `last_rejected_fuse_time_` 去重 |

#### 数据流变化

```
旧:
  邻机 communication stamp → 求均值 → sync_ref_time → selectNearest*(sync_ref_time) → DGO stamp

新:
  computeNextFuseTime() → t_fuse → sync_ref_time = t_fuse
  → selectNearestIns(t_fuse) → 初始化 P_opt_ (position/stamp 对齐)
  → checkPeerCommunication(t_fuse)   (t_fuse - com_stamp 对齐检查)
  → selectNearestGt(t_fuse)
  → updatePredictedPeerPositions(t_fuse)
  → selectNearestUwb(t_fuse)
  → selectNearestCamera(t_fuse)
  → RunDGO() → publish stamp = t_fuse
```

run_28/29 运行时的终端输出(精选):

<details>
<summary>📄 run_28 终端输出 (点击展开)</summary>

```
roslaunch test ekf_dgo_test.launch run_id:=28 oracle_mode:=false
# DGO 初始化
[iris_0] residual debug CSV: run_data/run_28/dgo/iris_0_dgo_residual_debug.csv
[formation] configuring PX4 OFFBOARD/failsafe parameters
[formation] priming OFFBOARD setpoint stream for 2 seconds
[formation] requesting OFFBOARD mode and arming

# DGO 优化结果 (抽样)
[iris_0] cost: 9.77 -> 2.68 (72.5%), iter=4 | uwb=6.70/2.15 angle=0.81/0.38 xy=2.26/0.15 ins=0.00/0.00
  camera: fresh=1 raw=1 valid=1 angle_cstr=1 xy_cstr=1 used=1
[iris_2] cost: 0.69 -> 0.11 (83.9%), iter=6 | uwb=0.69/0.11 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
  camera: fresh=1 raw=0 valid=0 (几何盲区) used=0
[iris_1] cost: 5.37 -> 0.96 (82.1%), iter=6 | uwb=5.37/0.96 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
  camera: fresh=1 raw=0 valid=0 (几何盲区) used=0
[iris_3] cost: 1.12 -> 0.46 (58.7%), iter=4 | uwb=0.60/0.36 angle=0.26/0.07 xy=0.26/0.03 ins=0.00/0.00
  camera: fresh=1 raw=2 valid=1 angle_cstr=1 xy_cstr=1 used=1

# 关键性能指标
[EKF DGO TEST] iris_1 relative to iris_0
  samples: 510 | RMSE: 0.0998 m
[EKF DGO TEST] iris_2 relative to iris_0
  samples: 510 | RMSE: 0.1262 m
[EKF DGO TEST] iris_3 relative to iris_0
  samples: 509 | RMSE: 0.0945 m
DGO mean RMSE: 0.108m | EKF mean RMSE: 0.130m | ratio: 0.83
```

</details>

<details>
<summary>📄 run_29 终端输出 (点击展开)</summary>

```
roslaunch test ekf_dgo_test.launch run_id:=29 oracle_mode:=false
# DGO 初始化
[iris_0] residual debug CSV: run_data/run_29/dgo/iris_0_dgo_residual_debug.csv
[formation] configuring PX4 OFFBOARD/failsafe parameters
[formation] priming OFFBOARD setpoint stream for 2 seconds
[formation] requesting OFFBOARD mode and arming

# DGO 优化结果 (抽样)
[iris_3] cost: 1.79 -> 1.54 (13.9%), iter=5 | uwb=1.71/1.23 angle=0.06/0.16 xy=0.02/0.15 ins=0.00/0.00
  camera: fresh=1 raw=2 valid=1 angle_cstr=1 xy_cstr=1 used=1
[iris_2] cost: 4.09 -> 0.07 (98.2%), iter=5 | uwb=4.09/0.07 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
  camera: fresh=1 raw=0 valid=0 (几何盲区) used=0

# 关键性能指标
[EKF DGO TEST] iris_1 relative to iris_0
  samples: 367 | RMSE: 0.1761 m
[EKF DGO TEST] iris_2 relative to iris_0
  samples: 366 | RMSE: 0.1566 m
[EKF DGO TEST] iris_3 relative to iris_0
  samples: 366 | RMSE: 0.1420 m
DGO mean RMSE: 0.159m | EKF mean RMSE: 0.223m | ratio: 0.71
```

</details>

### 阶段一验收报告：run_28 / run_29

**结论**：run_28 达到阶段一主要验收要求；run_29 因初始化副作用未通过。修复后需重新验证。

#### 验收标准逐项结果

| 项目 | 阶段一目标 | run_28 | run_29 | 结论 |
|:---|:---:|:---:|:---:|:---:|
| DGO stamp 倒退率 | 0% | 0% | 0% | ✅ 通过 |
| 单机融合步长 | 0.1 s | 全部 0.1 s | 全部 0.1 s | ✅ 通过 |
| 四机融合历元差 p95 | < 20 ms | 共同窗口完全对齐 | 共同窗口完全对齐 | ✅ 通过 |
| 四机首个有效 DGO stamp 差 | < 0.2 s | 0.1 s | **13.4 s** | ❌ run_29 不通过 |
| raw evaluator 配对率 | > 95% | 99.6%~99.8% | iris_2/3 ~73% | ❌ run_29 不通过 |
| large INS delta | 不应出现 | 未出现 | iris_0/1 出现 13 s delta | ❌ run_29 不通过 |
| DGO RMSE | 明显改善 | 0.095~0.126 m | 0.142~0.176 m | ⚠️ run_29 偏差 |
| DGO/EKF Z 偏移差 p95 | < 0.03 m | iris_1/2 超过 | **全部 < 0.01 m** | ⚠️ run_28 未满足 |

#### run_28 DGO 同步

| UAV | DGO 记录数 | 首个 stamp | 最后 stamp | stamp 倒退 | fuse_step |
|:---|:---:|:---:|:---:|:---:|:---:|
| iris_0 | 512 | 14.8s | 65.9s | 0 | 0.1s |
| iris_1 | 511 | 14.9s | 65.9s | 0 | 0.1s |
| iris_2 | 511 | 14.9s | 65.9s | 0 | 0.1s |
| iris_3 | 509 | 14.9s | 65.7s | 0 | 0.1s |

四机共同窗口：14.9s ~ 65.7s，共 509 个共同历元，覆盖率 **100%** ✅

#### run_28 evaluator 配对率

| 相对关系 | self DGO callbacks | valid samples | 配对率 |
|:---|:---:|:---:|:---:|
| iris_1 - iris_0 | 511 | 510 | **99.8%** |
| iris_2 - iris_0 | 511 | 510 | **99.8%** |
| iris_3 - iris_0 | 509 | 509 | **100%** |

#### run_28 DGO 精度

| 相对关系 | DGO RMSE | XY RMSE | Z RMSE | \|Z error\| p95 |
|:---|:---:|:---:|:---:|:---:|
| iris_1 - iris_0 | 0.100 m | 0.090 m | 0.044 m | 0.085 m |
| iris_2 - iris_0 | 0.126 m | 0.119 m | 0.042 m | 0.090 m |
| iris_3 - iris_0 | 0.095 m | 0.089 m | 0.032 m | 0.058 m |

#### run_29 问题根因

**根因 1：`isReadyForDGO()` 初始化位置太早**

旧结构为：`selectNearestIns(t_fuse) → if (!dgo_started_) { 初始化; dgo_started_ = true; } → 再检查 peer communication / UWB / camera`。只要 INS/origin/airborne 满足就会初始化，但 peer communication 可能随后失败。`dgo_started_` 已置 true，`prev_ins_stamp_` 已锁存，但未发布 DGO。十几秒后第一次成功 RunDGO 时，INS delta = 27.8 - 14.7 ≈ 13.1 s。

**根因 2：`max_extrapolation_dt` 实际仍是 0.20 s**

源码成员默认值 `double max_extrapolation_dt_ = 0.2;` 未同步更新。通信 stamp 与 `t_fuse` 的差经常在 0.20 s 附近，导致启动阶段频繁拒绝历元。

#### 修复措施（已实施）

| 修改 | 说明 |
|:---|:---|
| `isReadyForDGO()` 重构 | 初始化条件检查与状态写入分离，所有 readiness 检查通过后最后才 `initializeDgoFromAlignedIns()` |
| 新增 4 个子函数 | `checkPeerCommunicationReady()`、`prepareAlignedGt()`、`prepareOptionalCamera()`、`initializeDgoFromAlignedIns()` |
| `max_extrapolation_dt_` 默认值 | 0.2 → 0.30，与 launch 一致 |
| 启动日志 | 新增 `timing gates` 日志，终端明确打印各门限实际值 |
| `updateReferenceTime()` | 禁用为 `ROS_ERROR` 返回 false |
| `communication.cpp` velocity | 改为与位置源同源 |

#### run_30 / run_31 检查与修复

| 问题 | 验证 | 修复 |
|:---|:---|:---|
| `max_extrapolation_dt` 运行时仍为 0.20 | 终端打印 `max_extrapolation_dt=0.200` | 4 个 DGO 节点改为硬编码 `value="0.30"` |
| communication vel 日志显示 `ins_estimate` | 终端 `vel_source=ins_estimate` | communication.cpp 日志改为 `same_as_position_source` |

**结论**：核心同步逻辑通过；`max_extrapolation_dt` 硬化后预期 run_32/33 全部达标。





#### run_30~run_31

<details>
<summary>📄 run_30 终端输出 (点击展开)</summary>

```
roslaunch test ekf_dgo_test.launch run_id:=30 oracle_mode:=false

# 关键运行日志 (精选)
[formation] requesting OFFBOARD mode and arming
[iris_0] DGO initial offsets locked from Gazebo after 1.00s
[iris_1] DGO initialized from fully-ready aligned INS: t_fuse=154.500 ins_stamp=154.509 stage=1 altitude=0.365
  cost: 0.57 -> 0.01 (98.7%), iter=4 | uwb=0.57/0.01 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
[iris_3] DGO initialized from fully-ready aligned INS: t_fuse=154.600 ins_stamp=154.599 stage=1 altitude=0.390
  cost: 1.60 -> 0.99 (38.5%), iter=4 | uwb=0.90/0.78 angle=0.06/0.08 xy=0.65/0.13 ins=0.00/0.00
[iris_2] DGO initialized from fully-ready aligned INS: t_fuse=155.000 ins_stamp=154.997 stage=1 altitude=0.546
  cost: 0.96 -> 0.72 (24.8%), iter=5 | uwb=0.96/0.72 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
[iris_0] DGO initialized from fully-ready aligned INS: t_fuse=155.100 ins_stamp=155.101 stage=1 altitude=0.711
  cost: 2.33 -> 1.81 (22.3%), iter=4 | uwb=1.53/0.92 angle=0.78/0.78 xy=0.01/0.11 ins=0.00/0.00

[formation] Stage TAKEOFF: climb from 0.05 m to 3.0 m at 0.45 m/s
[formation] Stage 1 Hovering: 2 seconds
[formation] Stage 2 Expanding & Shrinking: 2.0 m outward and back
[formation] Stage 3 Translating: +3.0 m ENU-Y and back

# 最终 RMSE (Ctrl-C)
[EKF DGO TEST] iris_1 relative to iris_0: samples=??? | RMSE: ??? m
[EKF DGO TEST] iris_2 relative to iris_0: samples=??? | RMSE: ??? m
[EKF DGO TEST] iris_3 relative to iris_0: samples=??? | RMSE: ??? m
```

</details>

<details>
<summary>📄 run_31 终端输出 (点击展开)</summary>

```
roslaunch test ekf_dgo_test.launch run_id:=31 oracle_mode:=false
# 最终 RMSE (Ctrl-C)
[EKF DGO TEST] iris_1 relative to iris_0: RMSE: ??? m
[EKF DGO TEST] iris_2 relative to iris_0: RMSE: ??? m
[EKF DGO TEST] iris_3 relative to iris_0: RMSE: ??? m
```

</details>

<details>
<summary>📄 run_32 终端输出 (点击展开)</summary>

```
[DGO] fusion timing: period=0.100 fixed_lag=0.200 retry_lag=1.000 history_keep=2.000
[DGO] timing gates: max_sensor_age=0.500 max_communication_age=0.500 max_extrapolation_dt=0.300 max_gt_age=0.100
[communication] vel_source=same_as_position_source max_dgo_age=0.50 stabilize=0

[iris_2] DGO initialized from fully-ready aligned INS: t_fuse=16.700 stage=1 altitude=0.372
  cost: 2.10 -> 0.35 (83.5%), iter=5 | uwb=2.10/0.35 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
[iris_0] DGO initialized from fully-ready aligned INS: t_fuse=16.900 stage=1 altitude=0.410
  cost: 6.26 -> 0.60 (90.3%), iter=5 | uwb=4.61/0.37 angle=0.03/0.12 xy=1.63/0.12 ins=0.00/0.00
[iris_3] DGO initialized from fully-ready aligned INS: t_fuse=16.900 stage=1 altitude=0.389
  cost: 4.62 -> 2.43 (47.4%), iter=4 | uwb=2.66/2.22 angle=0.06/0.01 xy=1.90/0.20 ins=0.00/0.00
[iris_1] DGO initialized from fully-ready aligned INS: t_fuse=16.900 stage=1 altitude=0.367
  cost: 0.96 -> 0.04 (96.4%), iter=5 | uwb=0.96/0.04 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00

[formation] Stage TAKEOFF
[formation] Stage 1 Hovering: 2 seconds
[formation] Stage 2 Expanding & Shrinking
[formation] Stage 3 Translating
[formation] Stage 4 Chase/Restore
[formation] Landing
[formation] mission complete, all UAVs landed and disarmed

[EKF DGO TEST] iris_1 relative to iris_0: samples=495 RMSE=0.089555 m
[EKF DGO TEST] iris_2 relative to iris_0: samples=495 RMSE=0.109907 m
[EKF DGO TEST] iris_3 relative to iris_0: samples=495 RMSE=0.093557 m
```

</details>

<details>
<summary>📄 run_33 终端输出 (点击展开)</summary>

```
[DGO] timing gates: max_extrapolation_dt=0.300
[communication] vel_source=same_as_position_source max_dgo_age=0.50 stabilize=0

[iris_1] DGO initialized from fully-ready aligned INS: t_fuse=17.000 stage=1 altitude=0.386
  cost: 1.61 -> 0.92 (43.0%), iter=5 | uwb=1.61/0.92 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
[iris_2] DGO initialized from fully-ready aligned INS: t_fuse=17.000 stage=1 altitude=0.384
  cost: 1.24 -> 1.14 (8.3%), iter=5 | uwb=1.24/1.14 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
[iris_0] DGO initialized from fully-ready aligned INS: t_fuse=17.200 stage=1 altitude=0.406
  cost: 9.97 -> 1.01 (89.9%), iter=5 | uwb=4.45/0.59 angle=1.06/0.18 xy=4.46/0.24 ins=0.00/0.00
[iris_3] DGO initialized from fully-ready aligned INS: t_fuse=17.300 stage=1 altitude=0.475
  cost: 3.82 -> 1.57 (59.0%), iter=4 | uwb=1.20/0.87 angle=1.12/0.40 xy=1.50/0.30 ins=0.00/0.00

[formation] Stage TAKEOFF
[formation] mission complete, all UAVs landed and disarmed

# mission complete 后 DGO 继续运行，误差爆炸
[iris_1] cost: 258.56 -> 255.60 (1.1%), iter=5 | uwb=258.56/255.41
DGO XY correction limited (multiple)

[EKF DGO TEST] iris_1 relative to iris_0: samples=497 RMSE=0.320632 m  # ← 被尾段污染
[EKF DGO TEST] iris_2 relative to iris_0: samples=497 RMSE=0.312623 m
[EKF DGO TEST] iris_3 relative to iris_0: samples=496 RMSE=0.269119 m
```

</details>

### run_34~run_35: 任务完成自动停止 + 速度源修正 + evaluator 飞行窗口停止

#### 修改内容

本次修改主要解决 run_33 落地后 DGO 发散污染结果的问题，同时对 communication 速度源进行修正：

| 修改 | 说明 | 文件 |
|:---|:---|:---:|
| DGO 任务完成自动停止 | 新增 `stop_after_mission_complete=true`, `dgo_stop_stage=7`，stage 到达 7 后 DGO 停止迭代和发布 | `DGO.cpp` |
| `RunDGO()` 返回 bool | 仅 `published=true` 时推进 `last_fuse_time_`，失败时增加 `rejected_fuse_epoch_count` | `DGO.cpp` |
| fallback 路径 CSV 写入 | L-BFGS 失败时写入 residual debug / sync diag / comm debug CSV | `DGO.cpp` |
| INS 缓存改为完整 Odometry | `deque<pair<Time, Point>>` → `InsSample`，`pruneStampedHistory()` 时间窗口裁剪 | `DGO.cpp` |
| `max_extrapolation_dt` 硬化 | 默认 0.2→0.30，4 个 DGO 节点 launch 硬编码 0.30 | `DGO.cpp`, `dgo_full_mission.launch` |
| Communication 速度源修正 | 速度使用与位置源同源的 twist（之前始终用 INS）；日志改为 `same_as_position_source` | `communication.cpp` |
| `max_dgo_age` 放宽 | 0.3→0.5，配合 `fixed_lag=0.20` | `communication.cpp` |
| Evaluator 任务完成停止 | 订阅 `/formation/stage`，`eval_stop_stage=7` 停止记录；CSV 新增 `mission_stage`/`recording_active` 列 | `ekf_dgo_test.cpp` |
| 融合历元参数化 | 新增 `fusion_period`/`fixed_lag`/`max_fuse_retry_lag`/`history_keep_time` 及合法性校验 | `DGO.cpp` |

#### run_34（首次运行失败）

代码改动后首次运行，DGO 节点未产生有效输出（sync_diag CSV 仅有表头，evaluator CSV 仅有表头）。可能原因：DGO 初始化条件过于严格或参数冲突。

#### run_35（成功运行）

**关键性能指标：**

| 相对关系 | DGO RMSE | EKF RMSE | 样本数 |
|:---|:---:|:---:|:---:|
| iris₁ - iris₀ | **0.092 m** | 0.141 m | 451 |
| iris₂ - iris₀ | **0.124 m** | 0.155 m | 451 |
| iris₃ - iris₀ | **0.102 m** | 0.142 m | 451 |
| **均值** | **0.106 m** | **0.146 m** | — |

- DGO/EKF ratio: **~0.73**（优于 run_32 的 0.80）
- 飞行窗口: ~24s→~78s（起飞→落地），DGO 在 stage 7 自动停止
- 落地后 DGO 不再发散污染结果（run_33 问题修复）
- iris₁/iris₃ 角度约束持续使用（前视可见），iris₂ 大部分时间几何盲区

<details>
<summary>📄 run_35 终端输出 (点击展开)</summary>

```
[INFO] [INS ESKF] state initialized from odom: p0=(-0.01, -0.00, -0.02) RPY(deg)=(179.4, 180.0, 180.0)
[INFO] [communication] ns=iris_3 id=3 rate=10.0Hz pos_source=dgo_estimate(fallback=ins) vel_source=same_as_position_source max_dgo_age=0.50 stabilize=0
[WARN] [communication] waiting for ins_estimate
[INFO] [ID MATCH] ns=iris_1 self_idx=1 monitor=4 max_match_error=0.800 rad max_ins_align_dt=0.080
[INFO] [iris_1] residual debug CSV: run_data/run_35/dgo/iris_1_dgo_residual_debug.csv
[INFO] [iris_1] comm debug CSV: run_data/run_35/dgo/iris_1_comm_debug.csv
[INFO] [iris_1] sync diag CSV: run_data/run_35/dgo/sensor_sync_logs/iris_1_dgo_sync_diag.csv
[DGO] ns=/iris_1 uav_id=1 uav_num=4 initial_spacing=2.00 ... oracle_mode=0
[DGO] fusion timing: period=0.100 fixed_lag=0.200 retry_lag=1.000 history_keep=2.000
[DGO] timing gates: max_sensor_age=0.500 max_communication_age=0.500 max_extrapolation_dt=0.300 max_gt_age=0.100
[DGO] mission stop: stop_after_complete=1 dgo_stop_stage=7 publish_hold_after_stop=0

# 各节点初始化 (iris_0~3 residual/comm/sync CSV, 参数确认)
[iris_0] DGO initial offsets locked from Gazebo after 1.00s
[iris_1] DGO initial offsets locked from Gazebo after 1.02s
[iris_2] DGO initial offsets locked from Gazebo after 1.00s
[iris_3] DGO initial offsets locked from Gazebo after 1.00s

# DGO waiting for airborne (altitude < 0.350m)
[iris_1] DGO waiting for airborne initialization: stage=0 t_fuse=25.100 altitude=-0.000 required=0.350
[iris_0] DGO waiting for airborne initialization: stage=0 t_fuse=25.200 altitude=-0.000 required=0.350
[iris_2] DGO waiting for airborne initialization: stage=0 t_fuse=25.200 altitude=0.016 required=0.350
[iris_3] DGO waiting for airborne initialization: stage=0 t_fuse=25.200 altitude=0.024 required=0.350

[formation] requesting OFFBOARD mode and arming
[formation] Stage TAKEOFF: climb from -0.00 m to 3.0 m at 0.45 m/s

# DGO 初始化 (t_fuse ~32.6s, stage=1, altitude>0.35m)
[iris_1] DGO initialized from fully-ready aligned INS: t_fuse=32.600 ins_stamp=32.592 stage=1 altitude=0.354
[iris_1] cost: 3.77 -> 3.38 (10.3%), iter=4 | uwb=3.77/3.38 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
  camera: fresh=1 raw=0 valid=0 (几何盲区) used=0

[iris_3] DGO initialized from fully-ready aligned INS: t_fuse=32.600 stage=1 altitude=0.406
[iris_3] cost: 2.93 -> 0.33 (88.6%), iter=5 | uwb=1.74/0.15 angle=0.28/0.17 xy=0.90/0.01 ins=0.00/0.00
  camera: fresh=1 raw=1 valid=1 angle_cstr=1 xy_cstr=1 used=1 age=0.372

[iris_2] DGO initialized from fully-ready aligned INS: t_fuse=32.700 stage=1 altitude=0.353
[iris_2] cost: 1.36 -> 0.28 (79.4%), iter=4 | uwb=1.36/0.28 angle=0.00/0.00 xy=0.00/0.00 ins=0.00/0.00
  camera: stale, use UWB+INS only, age=0.502s > 0.500s

[iris_0] DGO initialized from fully-ready aligned INS: t_fuse=32.800 stage=1 altitude=0.428
[iris_0] cost: 11.78 -> 3.01 (74.4%), iter=4 | uwb=1.15/1.54 angle=5.20/0.75 xy=5.44/0.72 ins=0.00/0.00
  camera: fresh=1 raw=2 valid=1 angle_cstr=1 xy_cstr=1 used=1 age=0.216

# 飞行阶段切换
[formation] Stage 1 Hovering: 2 seconds
[formation] Stage 2 Expanding & Shrinking: 2.0 m outward and back
[formation] Stage 3 Translating: +3.0 m ENU-Y and back
[formation] Stage 4 Chase/Restore: cyclic position swap and return
[formation] Landing: descend from 3.01 m at 0.45 m/s

# DGO 自动停止 (stage=7)
[WARN] [EKF DGO TEST] stop recording: mission_stage=7 >= eval_stop_stage=7, samples=451, rmse_so_far=0.102089
[WARN] [EKF DGO TEST] stop recording: mission_stage=7 >= eval_stop_stage=7, samples=451, rmse_so_far=0.123647
[WARN] [EKF DGO TEST] stop recording: mission_stage=7 >= eval_stop_stage=7, samples=451, rmse_so_far=0.092426
[WARN] [iris_0] DGO stopped after mission complete: stage=7 >= dgo_stop_stage=7, last_fuse_time=77.800
[WARN] [iris_1] DGO stopped after mission complete: stage=7 >= dgo_stop_stage=7, last_fuse_time=77.800
[WARN] [iris_2] DGO stopped after mission complete: stage=7 >= dgo_stop_stage=7, last_fuse_time=77.800
[WARN] [iris_3] DGO stopped after mission complete: stage=7 >= dgo_stop_stage=7, last_fuse_time=77.800

[formation] mission complete, all UAVs landed and disarmed

# Ctrl-C, 计算最终 RMSE
=== Ctrl-C received, computing EKF+DGO relative RMSE ===

============================================
[EKF DGO TEST] iris_3 relative to iris_0
  samples: 451 | RMSE: 0.102089 m
============================================
[EKF DGO TEST] iris_1 relative to iris_0
  samples: 451 | RMSE: 0.092426 m
============================================
[EKF DGO TEST] iris_2 relative to iris_0
  samples: 451 | RMSE: 0.123647 m
============================================

# INS 评估 (ESKF vs PX4 EKF2)
============================================
  iris_0  ATE (RMSE): 0.2070 m | drift: 0.0036 m/s  (0.21 m/min)
============================================
  iris_1  ATE (RMSE): 0.1518 m | drift: 0.0026 m/s  (0.16 m/min)
============================================
  iris_2  ATE (RMSE): 0.1309 m | drift: 0.0022 m/s  (0.13 m/min)
============================================
  iris_3  ATE (RMSE): 0.1962 m | drift: 0.0034 m/s  (0.20 m/min)
============================================

=============== EKF 相对 iris_0 评估 ===============
  iris_1 relative to iris_0: samples=3101, RMSE=0.1406 m
  iris_2 relative to iris_0: samples=3176, RMSE=0.1548 m
  iris_3 relative to iris_0: samples=3075, RMSE=0.1424 m
============================================
```
</details>
