# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Micro-UAV swarm onboard cooperative relative positioning system, implementing the paper "Onboard cooperative relative positioning system for Micro-UAV swarm". Uses PX4 SITL + Gazebo simulation with XTDrone on a 4-drone (iris_0~iris_3) formation.

**Key algorithm chain:** IMU → ESKF (Error-State Kalman Filter) → Communication (10Hz broadcast) → UWB z-score filter → Camera angle detection → DGO (Distributed Graph Optimization via L-BFGS) → Relative position estimate.

## Build System

ROS catkin workspace with 4 packages. Build with standard ROS tooling:

```bash
cd /home/scott/swarm_localization
catkin_make  # or catkin build
source devel/setup.bash
```

ROS distro: Melodic (Python 3.8 in virtualenv, ROS Python libs from system).

## ROS Packages & Architecture

### `sensors` — Sensor simulation and low-level nodes
- **Messages:** `UwbRange`, `InsEstimate`, `ImageDetection`, `ComMsg`
- **C++ nodes:** `uwb` (UWB simulator with GT), `communication` (10Hz INS estimate broadcast), `control`, `xtdrone_four_uav_formation` (7-stage formation controller)
- **Python nodes:** `image.py` (YOLOv7 object detection via ROS)

### `data_process` — Sensor fusion and state estimation
- **Messages:** `UwbProcessed`, `CameraAngle`, `CameraAngleMatch`
- **Core node:** `ins_eskf.cpp` — 15-state Error-State EKF (p/v/q/bg/ba). Prediction: IMU ~100Hz. Update: optical flow body velocity + ToF height ~30-50Hz. Publishes `ins_estimate` (nav_msgs/Odometry).
- **Supporting nodes:** `camera_relative_angle_cal` (bearing/elevation from YOLO detections), `id_match` (persistent UAV ID tracking), `uwb_zero_score` (sliding-window z-score outlier rejection), `tf_bridge.py`

### `algorithm` — Distributed Graph Optimization
- **Messages:** (depends on sensors, data_process)
- **Core node:** `DGO.cpp` (~1200 lines) — L-BFGS optimization fusing UWB ranging + camera angles + INS constraints. Uses the `LBFGSpp` library (header-only, included in repo at `src/LBFGSpp/`).
- Key DGO features: `relativeToTarget()` computes offset-corrected relative positions, `updatePredictedPeerPositions()` extrapolates communication positions by velocity×dt, writes `{drone}_dgo_residual_debug.csv` and `{drone}_comm_debug.csv`.

### `test` — Evaluation and analysis
- **C++ evaluators:** `ins_eskf_test.cpp` (ATE/RPE/NEES metrics, GT comparison), `ekf_dgo_test.cpp` (combined EKF + DGO relative position error to reference), `angle_error_cal.cpp` (camera angle RMSE), `two_uavs_formation.cpp` (2-drone test script)
- **Python plotting scripts:** `ekf_plot.py`, `ekf_relative_plot.py`, `dgo_plot.py`, `dgo_ekf_plot.py`, `uwb_plot.py`, `angle_show.py`
- **Launch files in `src/test/launch/`:** `uwb_test.launch`, `angle_error_test.launch`, `ekf_dgo_test.launch`
- **CSV output:** all CSVs saved to `src/test/logs/` (data_process nodes) or `src/data_process/logs/` (EKF test)

## Message Data Flow

```
Gazebo (model_states)
  ├─→ uwb.cpp → sensors/UwbRange (with gt_distances)
  │               └─→ uwb_zero_score.cpp → data_process/UwbProcessed
  │                                             └─→ DGO (UWB cost)
  ├─→ image.py → ImageDetection
  │               └─→ camera_relative_angle_cal → CameraAngle
  │                     └─→ id_match → CameraAngleMatch
  │                           └─→ DGO (angle cost)
  ├─→ PX4 EKF2 → /mavros/local_position/odom (NOT Gazebo GT, but PX4's estimate!)
  └─→ ins_eskf.cpp → ins_estimate (nav_msgs/Odometry)
                        ├─→ communication → ComMsg (10Hz) → DGO Pc
                        └─→ ins_eskf_test.cpp (evaluation against GT)
```

## SITL Simulation Setup

- PX4 Firmware at `/home/scott/PX4_Firmware/`
- Custom drone model SDF: `sitl_config/models/iris_uwb_flow_tof_frontcam/`
- SITL launch: PX4's `multi_uav_mavros_sitl_sdf.launch` (modified for custom SDF)
- Formation: 4 iris drones in 2×2 grid (initial_spacing=2.0m):
  - iris_0 (0,0), iris_1 (2,0), iris_2 (2,2), iris_3 (0,2) — clockwise arrangement
- Formation flight controller: `xtdrone_four_uav_formation` (7 stages: takeoff → hover → expand/contract → translate → rotate → land → done)

## Key Launch Files

| Launch File | Purpose |
|---|---|
| `src/sensors/launch/uwb.launch` | Start 4 UWB simulators |
| `src/data_process/launch/uwb_zero_score.launch` | UWB z-score filter |
| `src/data_process/launch/ins_eskf_multi.launch` | Start 4 EKF nodes (no GT) |
| `src/data_process/launch/ins_eskf_test.launch` | EKF evaluation (writes CSVs) |
| `src/algorithm/launch/dgo_full_mission.launch` | Complete DGO pipeline |
| `src/data_process/launch/full_mission.launch` | Full pipeline without DGO |
| `src/test/launch/angle_error_test.launch` | 2-drone angle accuracy test |
| `src/test/launch/uwb_test.launch` | 4-drone UWB accuracy test |

## Known Issues & Evaluation Gotchas

1. **`/mavros/local_position/odom` is PX4's EKF2 estimate, NOT Gazebo ground truth.** The `ins_eskf_test.cpp` uses it as GT (see comment on line 76), but it's actually PX4's internal estimator output. For true GT, use `/gazebo/model_states`. The relative position error CSV correctly uses model_states, but the absolute ATE/RPE metrics compare our EKF vs PX4's EKF.

2. **Optical flow formula bug (confirmed):** `ins_eskf.cpp` lines 364-365 currently have:
   ```cpp
   vx_body = msg->distance * (flow_y + gyro_y);  // WRONG — should be (flow_x - gyro_x)
   vy_body = msg->distance * (flow_x - gyro_x);  // WRONG — should be (flow_y + gyro_y)
   ```
   Offline verification shows swapping reduces meas-minus-GT error by ~60%. NOT YET APPLIED to code.

3. **EKF `disable_attitude_update=true`:** The launch files set this, meaning yaw from IMU integration drift is NOT corrected by observations. This is intentional for isolating error sources, but affects long-term accuracy.

4. **UWB filter:** Sliding window z-score (window=5, threshold=3σ), with warm-up detection and consecutive-reject reset. Filtered RMSE ~4.2cm vs ~7.7cm raw.

## Common Commands

```bash
# Build
catkin_make -C /home/scott/swarm_localization
source /home/scott/swarm_localization/devel/setup.bash

# Start SITL simulation (in separate terminals, follow README sequence)
# Typically: Gazebo → PX4 SITL → MAVROS → ROS nodes

# Run DGO full mission
roslaunch algorithm dgo_full_mission.launch

# Run EKF test only
roslaunch data_process ins_eskf_test.launch

# Plot results (after CSVs generated)
python3 src/test/scripts/ekf_plot.py --no-show
python3 src/test/scripts/ekf_relative_plot.py --no-show
python3 src/test/scripts/dgo_plot.py --no-show
python3 src/test/scripts/dgo_ekf_plot.py --no-show
python3 src/test/scripts/uwb_plot.py --no-show
```

## Key Parameters

- **EKF:** `disable_attitude_update` (bool), `imu_frame` (auto/frd/flu), `min_flow_quality` (0-255), `publish_pos_cov_scale`, `publish_vel_cov_scale`
- **UWB filter:** `window_size` (5), `z_score_threshold` (3.0), `min_stddev` (0.08), `max_consecutive_rejects` (5)
- **DGO:** `uav_id` (0-3), `initial_spacing` (2.0), `max_sensor_age` (0.5s)
- **INS test:** `drone_names`, `reference_name` (iris_0), `max_ekf_align_dt` (0.12s), `max_model_align_dt` (0.03s)
