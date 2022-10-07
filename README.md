# ICT-example
Example code of Design and Implementation of Robotic Systems and Applications

## Prerequisite

```bash
sudo apt install python-catkin-tools
```

## Install

```bash
cd ~/
source /opt/ros/<ros-distro>/setup.bash
git clone --recursive https://github.com/ISCI-LAB/ICT-example.git
cd ~/ICT-example/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Docker
Build docker container
```bash
cd ~/
git clone --recursive https://github.com/ISCI-LAB/ICT-example.git
cd ~/ICT-example/Docker/<device>/
./build.sh  # Build docker image for locobot environment
./docker_run.sh  # Run docker container with mounted folder
source ~/low_cost_ws/devel/setup.bash
cd ~/ICT-example/catkin_ws/
catkin build
```

Start container or open another terminal in container
```bash
./docker_join.sh
```

To evaluate if gpu is available for Pytorch in Docker container, run test script
```bash
cd ~/ICT-example/
python2 ./third_party/grasp_samplers/test/test_grasping.py
```

## Quickstart

1. Launch locobot gazebo simulation program

    ```bash
    roslaunch locobot_control main.launch use_sim:=true ict:=true
    ```

2. Launch locobot navigation controller

    ```bash
    roslaunch baseline_navi navigation.launch
    ```

3. Launch grasp estimation and stage switch

    ```bash
    roslaunch baseline_navi bringup.launch use_sim:=true
    ```

4. Publish topic to start picking demo

    ```bash
    rostopic pub /locobot_motion/grasp_start std_msgs/Int32 "data: 1" --once 
    ```