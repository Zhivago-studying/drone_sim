# Swarm Localization — Project Snapshot (2026-06-18)

Micro-UAV swarm (4× iris) onboard cooperative relative positioning. PX4 SITL + Gazebo + ROS Melodic.
Implements: "Onboard cooperative relative positioning system for Micro-UAV swarm".

## Directory Map

```
src/
├── sensors/         Sensor sim + low-level: uwb, communication(10Hz broadcast), xtdrone_four_uav_formation, image.py(YOLOv7)
├── data_process/    State estimation: ins_eskf (core 15-state ESKF), uwb_zero_score, camera_relative_angle_cal, id_match, tf_bridge
├── algorithm/       DGO (Distributed Graph Optimization, L-BFGS), uses header-only LBFGSpp in src/LBFGSpp/
├── test/            Evaluators: ins_eskf_test, ekf_dgo_test, angle_error_cal + plotting scripts
├── LBFGSpp/         Header-only L-BFGS library
sitl_config/models/  Custom iris SDF: iris_uwb_flow_tof_frontcam (IMU noise, UWB, ToF, optical flow, front camera)
```

## Algorithm Chain

```
IMU(100Hz) → ESKF(15-state, p/v/q/ba/bg) → ins_estimate → communication(10Hz) → DGO L-BFGS optimization
Optical flow + ToF(30-50Hz) → ESKF update                          ↑ UWB z-score filter ──┤
PX4 EKF2 → /mavros/local_position/odom (odom init, not GT)         ↑ Camera angle + ID match ─┤
/gazebo/model_states (true GT for evaluation)                      ↑ ComMsg position broadcast ─┤
```

## Core File: ins_eskf.cpp (~960 lines)

**State:** nominal 16-dim (p,v,q,ba,bg) + error 15-dim (δp,δv,δθ,δba,δbg)
**Prediction:** IMU-driven, ~100Hz. `a_world = R*(acc-ba) + g`, `v += a_world*dt`, `q = q⊗Exp((gyro-bg)*dt)`
**Update:** Optical flow body velocity (vx/vy FLU) + ToF height. 3-dim observation.
**Key params in YAML** (`src/data_process/config/ins_eskf_noise.yaml`):
  - `sigma_acc: 6.867e-4`  `sigma_gyro: 4.8869e-5`  `sigma_ba: 0.006`  `sigma_bg: 3.8785e-5`
  - `tau_ba: 300.0`  `tau_bg: 1000.0`  (Gauss-Markov bias correlation times)
  - `flow_relative_noise_std: 0.1`  `flow_base_noise_std: 0.02`  `tof_noise_std: 0.05`
**Noise model:** `σ_v = hypot(0.1*|V_raw|, 0.02)` → dynamic R per frame → LLT solve for Kalman gain
**Flow formula (corrected):** `vx_flu = h*(flow_x - gyro_x)`  `vy_flu = h*(flow_y + gyro_y)`
**Key flags:** `disable_attitude_update=true` — blocks δθ update, but allows δbg estimation. `imu_frame=auto|flu|frd`

## DGO (src/algorithm/src/DGO.cpp, ~1200 lines)

Distributed per-drone L-BFGS. Cost = UWB ranging + camera angle + INS prior constraints.
- `Pc[]` = peer comm positions (INS-based), extrapolated by velocity×dt
- `relativeToTarget()` returns `offset + (Pc_target - P_opt_self)` with initial formation offset
- IMU process noise params synchronized with SDF: `src/data_process/config/ins_eskf_noise.yaml`
- Evaluation: `ekf_dgo_test.cpp` computes relative position error (self vs iris_0) using `/gazebo/model_states` GT
- Outputs: `{drone}_dgo_residual_debug.csv`, `{drone}_comm_debug.csv`, `{drone}_relative_to_iris_0_dgo_error.csv`

## Evaluation Pipeline

| Evaluator | GT Source | Metrics |
|-----------|-----------|---------|
| `ins_eskf_test.cpp` | `/mavros/local_position/odom` (PX4 EKF2, NOT true GT!) | ATE, RPE(1/5/10s), attitude/velocity error, NEES |
| `ins_eskf_test.cpp` (relative) | `/gazebo/model_states` (true GT) | Relative position error vs iris_0 |
| `ekf_dgo_test.cpp` | `/gazebo/model_states` (true GT) | DGO relative position error vs iris_0 |

**Known gotcha:** Absolute ATE/RPE compares our EKF vs PX4's EKF2 — both process same IMU data, so common-mode drift cancels out. True absolute error is higher. Relative error uses model_states and is correct.

## Build & Run

```bash
catkin build -C /home/scott/swarm_localization
source devel/setup.bash
roslaunch algorithm dgo_full_mission.launch         # Full DGO
roslaunch data_process ins_eskf_test.launch          # EKF eval only
roslaunch test ekf_dgo_test.launch                   # DGO+EKF eval
python3 src/test/scripts/dgo_ekf_plot.py --no-show   # DGO vs EKF comparison plot
```

## Key Parameters & Files

- SDF model: `sitl_config/models/iris_uwb_flow_tof_frontcam/iris_uwb_flow_tof_frontcam.sdf`
  (IMU noise: accelerometerNoiseDensity=6.867e-4, gyroscopeNoiseDensity=4.8869e-5, randomWalk params, Gauss-Markov correlation times)
- Launch: `src/algorithm/launch/dgo_full_mission.launch`, `src/data_process/launch/ins_eskf_test.launch`
- UWB filter: sliding window z-score (window=5, threshold=3σ), RMSE ~4.2cm filtered vs ~7.7cm raw
- Camera angle: YOLOv7 (320×320, mAP@0.5=0.995) → bearing/elevation ~1-4° accuracy
- Data flow timing: `max_ekf_align_dt=0.12s`, `max_model_align_dt=0.03s`

## Recent Changes (EKF optimization)

1. Optical flow formula corrected to standard derivation
2. Sensor noise injection: composite flow noise `σ=hypot(0.1·|v|, 0.02)` + ToF noise `σ=0.05m`
3. Dynamic Kalman R matrix (vs old hardcoded 0.35²)
4. Bias Gauss-Markov decay moved before prediction (consistency fix)
5. `disable_attitude_update` only blocks δθ, now allows δbg estimation
6. LLT decomposition replaces determinant check (numerical stability)
7. Centralized IMU noise config in `ins_eskf_noise.yaml` matching SDF parameters
8. Flow diagnostic CSV extended with vx_noise_std, vy_noise_std, tof_noise columns

## Outstanding Issues

- `/mavros/local_position/odom` is PX4's estimate, not Gazebo GT — inflates absolute accuracy in `ins_eskf_test`
- `disable_attitude_update=true` means yaw drift uncorrected (intentional for experiments)
- No GPS, no magnetometer, no barometer — pure IMU + optical flow + ToF only
