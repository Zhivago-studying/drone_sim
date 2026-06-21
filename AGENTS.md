# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS catkin workspace for XTDrone swarm localization experiments. Core ROS packages live under `src/`: `sensors` publishes simulated UWB, INS, camera detections, and communication messages; `data_process` converts and fuses measurements; `algorithm` contains localization logic using `LBFGSpp`; `test` contains experiment nodes, plotting scripts, launch files, and CSV logs. Custom ROS messages are in each package's `msg/`, launch files in `launch/`, C++ nodes in `src/`, and Python scripts in `scripts/`.

YOLOv7 code is vendored in `yolov7/`; its dataset is under `yolov7/datas/`, and result media is in `yolov7_result_files/`. Simulation assets are in `sitl_config/`, while notes and diagrams are in `paper_and_structure/`.

## Build, Test, and Development Commands

Run catkin commands from the workspace root:

```bash
catkin build
source devel/setup.bash
roslaunch sensors xtdrone_mission.launch
roslaunch data_process full_mission.launch
roslaunch test angle_error_test.launch
```

`catkin build` builds ROS packages and generated messages. Source `devel/setup.bash` before running nodes. Most launch tests require ROS master, Gazebo/XTDrone, and matching MAVROS topics.

For YOLOv7 work:

```bash
pip install -r yolov7/requirements.txt
python yolov7/train.py --data yolov7/datas/data.yaml
python yolov7/detect.py --weights path/to/weights.pt --source path/to/images
```

## Coding Style & Naming Conventions

C++ targets use C++11 and ROS conventions. Keep node filenames lowercase with underscores, matching `camera_relative_angle_cal.cpp` and `uwb_zero_score.cpp`. Use package-prefixed CMake target names, then set concise ROS output names. Python scripts should be executable ROS scripts with lowercase underscore names. Message types use PascalCase, for example `CameraAngle.msg`.

## Testing Guidelines

There is no standalone unit-test suite; validation is launch-driven. Add experiment nodes under `src/test/src/`, scripts under `src/test/scripts/`, and launch files under `src/test/launch/`. Keep generated CSVs and plots in package-specific `logs/` or top-level result folders, and document expected topics or vehicles in the launch file name or README notes.

## Commit & Pull Request Guidelines

Recent history uses short Chinese commit summaries such as `实验日志更新`, `测试结果提交`, and `更新图片`. Keep commits focused and describe the artifact or experiment changed. Pull requests should include a brief objective, affected ROS packages or YOLO files, commands/launch files run, key metrics or screenshots for experiment changes, and any required simulator or model-weight setup.

## Security & Configuration Tips

Do not commit private credentials, absolute machine-specific paths, or large new model weights unless they are intentional experiment artifacts. Keep ROS topic names, frame IDs, and coordinate-frame assumptions explicit when changing localization or perception code.
