# run_9 ~ run_11 代码恢复记录

恢复日期：2026-06-21

## 目标时间

- run_9：2026-06-20 20:48 ~ 20:50
- run_10：2026-06-20 20:59 ~ 21:00
- run_11：2026-06-20 21:02 ~ 21:03

## 恢复依据

1. Git基线提交：`c7b7399`（2026-06-20 16:24:44）。
2. 实验数据：`run_data/run_9/`、`run_data/run_10/`、`run_data/run_11/`。
3. 对应ROS运行日志中的源码行号、启动参数和运行消息。
4. `README.md`中“run_9 ~ run_11: ToF高度观测计算改进”的记录。

## 核对结果

当前Git基线已经包含run_9 ~ run_11使用的主要功能：

- DGO有效相机约束计数和同步诊断CSV。
- camera队列深度为1。
- ESKF显式噪声参数和每机独立随机种子。
- 发布协方差缩放：位置15.0、速度3.0。
- run_id实验目录机制。
- IMU模型噪声参数修改。

确认丢失并根据实验记录重建的部分：

- `tof_min_range`，默认值0.20m。
- 初次ToF读数低于`tof_min_range + 0.03m`时按饱和处理。
- 饱和时令`height0_=0`，避免引入约0.20m固定高度偏置。

ROS日志中的运行证据：

```text
[INS ESKF] ToF saturated at 0.200 m (... 0.20 m),
set height0=0 to avoid bias
```

## 回退保障

恢复前的`src/data_process/src/ins_eskf.cpp`与Git对象
`c7b7399:src/data_process/src/ins_eskf.cpp`完全一致，可随时从该Git对象取回。

本次没有覆盖或删除实验数据、README、SDF参数和其他未跟踪文件。
