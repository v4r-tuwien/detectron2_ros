FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04
MAINTAINER Christian Eder (eder@acin.tuwien.ac.at)

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated curl lsb-release wget ca-certificates gnupg2

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# setup keys
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -

# update
RUN apt-get update 

RUN apt-get install --no-install-recommends -y --allow-unauthenticated ros-noetic-desktop-full

# install python3
RUN apt-get install --no-install-recommends -y --allow-unauthenticated software-properties-common build-essential
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
     python3-dev \
     python3-numpy \
     python3-pip \
     && rm -rf /var/lib/apt/lists/*

# catkin tools
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
     python3-catkin-tools \
     && rm -rf /var/lib/apt/lists/*

# for ros environments
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Prepare catkin build
RUN mkdir -p ~/catkin_ws/src

# install git and tmux
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated git tmux

ENV FORCE_CUDA="1"
ENV TORCH_CUDA_ARCH_LIST="Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"

RUN pip3 install torch torchvision --pre --extra-index-url https://download.pytorch.org/whl/nightly/cu116

RUN git clone https://github.com/facebookresearch/detectron2 /detectron2_repo
RUN pip3 install -e /detectron2_repo

RUN /bin/bash -c  'cd ~/catkin_ws/src; git clone https://github.com/v4r-tuwien/object_detector_msgs.git'
RUN /bin/bash -c  'cd ~/catkin_ws/src; git clone https://github.com/v4r-tuwien/detectron2_ros.git'

# Run catkin build
RUN /bin/bash -c  '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws/; catkin build'

# source the catkin workspace
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Make share folder
RUN mkdir -p ~/share

WORKDIR /root

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
