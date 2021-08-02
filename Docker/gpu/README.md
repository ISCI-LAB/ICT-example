# Making GPU docker image
*You should run this code on a GPU computer*

dependencies: 
- CUDA 10.1
- Python 3.6
- Python 2.7
- Pytorch 1.7.0
- ROS(Melodic)
- Pyrobot
- Smach

Usage:

You can skip build docker images, just source docker_run.sh, then it will automatically pull docker image from Docker Hub. The building docker image is used for developer.

**Building docker image**
```
    $ source build.sh
```

**How to run**
```
    $ source docker_run.sh
    Docker $ source catkin_make.sh
    Docker $ source environment.sh
```
**If you want to enter same container**
```
    $ source docker_join.sh
    Docker $ source environment.sh
```