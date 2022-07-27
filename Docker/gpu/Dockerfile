FROM nvidia/cuda:10.2-devel-ubuntu18.04


ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=isci \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:iscilab" | chpasswd

###################################### basic tools #####################################

RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    cmake \
    make \
    vim \
    build-essential \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    tzdata \
    python-pip \
    python3-pip \
    wget


############################# LoCoBot and PyRobot ############################
RUN sudo wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
RUN sudo python get-pip.py
RUN python -m pip install -U setuptools


RUN cd ${HOME}/

RUN curl 'https://raw.githubusercontent.com/jack6099boy/pyrobot/master/robots/LoCoBot/install/locobot_install_all.sh' > locobot_install_all.sh && chmod +x locobot_install_all.sh

RUN ./locobot_install_all.sh -t sim_only -p 2 -l interbotix

RUN rm locobot_install_all.sh

# install ros-melodic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install ros-melodic-desktop -y
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN source ~/.bashrc

RUN apt-get update && apt-get install -y \
    ros-melodic-smach \
    ros-melodic-smach-msgs \
    ros-melodic-smach-ros \
    ros-melodic-executive-smach \
    ros-melodic-smach-viewer


##################################### Pytorch ######################################
RUN python3 -m pip install -U pip
RUN sudo wget https://bootstrap.pypa.io/pip/3.5/get-pip.py
RUN sudo python3 get-pip.py


RUN pip3 install empy \
    catkin-tools \
    opencv-python \ 
    rospkg \
    matplotlib 


# RUN python3 -m pip install \
#     torch==1.2.0 torchvision==0.4.0 torchaudio==0.3.0 -f https://download.pytorch.org/whl/cu102/torch_stable.html


############################# custom gazebo ###############################
# pre download model 
RUN mkdir -p $HOME/.gazebo \
    && cd $HOME/.gazebo \
    && git clone https://github.com/osrf/gazebo_models \
    && mv gazebo_models models \
    && cd


RUN python -m pip install future open3d IPython python-gflags
# RUN apt install -y ros-melodic-ros-numpy python-catkin-tools
RUN python -m pip install pyrsistent==0.13 open3d_ros_helper
#install torch, torchvision and gdown
RUN pip install torch torchvision gdown
RUN pip install notebook
# RUN python -m pip install torch==1.10.1 torchvision==0.11.2 -f https://download.pytorch.org/whl/cu102/torch_stable.html

##################################### setting #####################################
RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL) ALL" > /etc/sudoers

RUN echo "root ALL=(ALL)  ALL" > /etc/sudoers

RUN echo "cd ~/ICT-example" >> ${HOME}/.bashrc

# GPU support
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

USER ${USER}
WORKDIR ${HOME}

RUN ls
