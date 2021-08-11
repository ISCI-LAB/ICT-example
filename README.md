# ICT-example
Example code of Design and Implementation of Robotic Systems and Applications

## Prerequisite

```bash
sudo apt install python3-catkin-tools
```

## Install

```bash
source /opt/ros/<ros-distro>/setup.bash
git clone --recursive https://github.com/ISCI-LAB/ICT-example.git
cd ~/ICP-example/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Quickstart

1. Launch locobot gazebo simulation program.

    ```bash
    roslaunch ict_gazebo locobot_sim.launch world:=placing use_camera:=true
    ```

2. Launch apriltag detector

    ```bash
    roslaunch apriltag_ros continuous_detection.launch camera_frame:=<frame_name> camera_name:=<camera_name> image_topic:=<topic_name>
    ```

3. Transform tag frame to locobot goal pose.

    ```bash
    roslaunch baseline_navi apriltags_to_goalpoint.launch
    ```

4. Launch locobot navigation controller

    ```bash
    roslaunch baseline_navi navigation.launch
    ```
