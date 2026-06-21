# 数据记录窗口改动回退记录

日期：2026-06-19

## 回退目标

撤销“数据记录和指标计算只在起飞后开始、进入着陆时结束”的整组改动，恢复回退前行为：

- CSV 和诊断记录由节点启动后持续执行。
- INS、DGO、角度误差指标在节点退出或 Ctrl-C 时计算。
- 飞行控制器不再发布统一记录窗口信号。

此前已完成的 ESKF、传感器参数、OFFBOARD/ARM、PX4 failsafe 和编队起飞修复不在本次回退范围内，全部保留。

## 回退前确认

通过全文检索确认上一轮改动包括：

- `/experiment/recording_active` latched 话题。
- `recording_active_`、`recording_started_`、`recording_start_time_` 状态。
- 飞行控制器中的 `publishRecordingState()`。
- DGO、UWB、ESKF 光流诊断的 CSV 门控与文件提前关闭。
- INS、EKF+DGO、角度误差评估的飞行窗口门控和着陆自动结算。
- `test` 包新增的 `std_msgs` 依赖。
- `ins_eskf_test.launch` 中对应的行为说明。

## 已回退文件

飞行窗口发布端：

- `src/sensors/src/xtdrone_four_uav_formation.cpp`
- `src/test/src/two_uavs_formation.cpp`
- `src/sensors/src/control.cpp`

数据记录端：

- `src/algorithm/src/DGO.cpp`
- `src/data_process/src/uwb_zero_score.cpp`
- `src/data_process/src/ins_eskf.cpp`

指标评估端：

- `src/data_process/src/ins_eskf_test.cpp`
- `src/test/src/ekf_dgo_test.cpp`
- `src/test/src/angle_error_cal.cpp`

依赖与说明：

- `src/test/CMakeLists.txt`
- `src/test/package.xml`
- `src/data_process/launch/ins_eskf_test.launch`

## 回退方法

采用逐段反向补丁，只删除上一轮新增代码，没有使用 `git reset`、`git checkout` 或覆盖整个文件，以避免破坏工作区内原有未提交修改。

回退顺序：

1. 删除三个飞行控制器的记录状态发布器和阶段切换逻辑。
2. 删除 DGO、UWB、ESKF 诊断记录的订阅、状态和时间戳门控。
3. 恢复三个评估节点原有的持续采样及退出时结算逻辑。
4. 移除 `test` 包新增的 `std_msgs` 依赖。
5. 恢复 launch 文件原有 Ctrl-C 指标说明。
6. 恢复两个文件因补丁产生的空行差异，使原本干净的文件回到零差异。

## 验证结果

残留检查：

```text
experiment/recording_active
recording_active_
recording_started_
recording_start_time_
recording_pub_
recording_sub_
publishRecordingState
```

上述关键字在相关项目源码中检索结果为空。

差异检查：

```text
git diff --check
```

结果：通过，无空白错误。

编译命令：

```bash
catkin build sensors data_process algorithm test --no-status
```

结果：

- `sensors`：成功
- `data_process`：成功
- `algorithm`：成功
- `test`：成功
- 失败包：0

构建期间只有 Gazebo Classic `gazebo_msgs` 已弃用提示，不是本次代码导致的编译问题。

## 最终状态

上一轮记录窗口改动已完全撤销。回退前已经存在的工作区修改和实验数据均未删除或重置。

---

# 2026-06-21：取消 DGO 通信历史插值

## 回退目标

撤销 DGO 为 camera 时间对齐新增的邻机 communication 历史缓存和双侧插值，
恢复使用最新通信位置及其 INS 速度进行有限时间外推。

本次仅取消通信插值，以下修复继续保留：

- 原始图像时间戳和序号在视觉链路中的传递。
- 本机 INS 在 camera_stamp 和 sync_ref_time 的历史插值。
- camera_stamp 附近 UWB 独立选择及时间差门限。
- 相机目标逐目标有效性检查。
- ID Match 按图像序号从短历史中匹配检测消息。
- Oracle 模式使用 Gazebo GT 历史对齐 camera_stamp。

## 修改内容

- 删除 `ComSample` 和 `com_history_`。
- 删除 communication 回调中的历史样本缓存。
- 删除 `interpolateComPosition()`。
- 删除 `max_camera_com_dt` 参数及 launch 透传。
- 正常模式改为：

```text
peer(camera_stamp) =
    latest_communication.position
    + latest_communication.velocity * (camera_stamp - communication_stamp)
```

- 当实际时间差超过 `max_extrapolation_dt` 时，该目标的相机约束直接失效。
- 不对时间差做截断后继续使用，避免生成并非 camera_stamp 时刻的伪状态。

## 保留文件修改

- `src/algorithm/src/DGO.cpp`
- `src/algorithm/launch/dgo_full_mission.launch`
- `src/test/launch/ekf_dgo_test.launch`

没有使用 `git reset`、`git checkout` 或覆盖整个文件。

## 验证结果

残留检索确认以下符号已经从 DGO 和相关 launch 中移除：

```text
ComSample
com_history_
interpolateComPosition
max_camera_com_dt
```

检查命令：

```bash
git diff --check -- \
  src/algorithm/src/DGO.cpp \
  src/algorithm/launch/dgo_full_mission.launch \
  src/test/launch/ekf_dgo_test.launch \
  ROLLBACK_RECORD.md

xmllint --noout \
  src/algorithm/launch/dgo_full_mission.launch \
  src/test/launch/ekf_dgo_test.launch

catkin build algorithm test --no-status
```

结果：

- 差异格式检查通过。
- 两个 launch 文件 XML 检查通过。
- `sensors`、`data_process`、`algorithm`、`test` 全部编译成功。
- 编译警告和失败包均为 0。
