FROM nvidia/cuda:10.0-cudnn7-devel
MAINTAINER Markus Suchi (suchi@acin.tuwien.ac.at)

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Linux package install
RUN sed 's/main$/main universe/' -i /etc/apt/sources.list
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated --fix-missing \
      build-essential \
      autoconf \
      automake \
      git \
      wget \
      usbutils \
      unzip \
      vim \
      software-properties-common \
      libxext-dev \
      libxrender-dev \
      libxslt1.1 \
      libxtst-dev \
      libgtk2.0-0 \
      libcanberra-gtk-module \
      tmux \
      && rm -rf /var/lib/apt/lists/*

# Fixes shared memory error in docker
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

# Python 3
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
     python3-dev \
     python3-numpy \
     python3-pip \
     && rm -rf /var/lib/apt/lists/*

# Install detectron2 package dependencies
RUN apt-get update && apt-get install -y \
      libpng-dev libjpeg-dev python3-opencv ca-certificates \
      python3-dev build-essential pkg-config git curl wget automake libtool && \
  rm -rf /var/lib/apt/lists/*

RUN curl -fSsL -O https://bootstrap.pypa.io/get-pip.py && \
                   python3 get-pip.py && \
                   rm get-pip.py

# Install detectron2 python dependencies
# See https://pytorch.org/ for other options if you use a different version of CUDA
RUN pip3 install torch torchvision -f https://download.pytorch.org/whl/torch_stable.html cython \
	'git+https://github.com/facebookresearch/fvcore'
RUN pip install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'

# Install detectron2
RUN git clone https://github.com/facebookresearch/detectron2 /detectron2_repo
ENV FORCE_CUDA="1"
ENV TORCH_CUDA_ARCH_LIST="Maxwell;Maxwell+Tegra;Pascal;Volta;Turing"
RUN pip install -e /detectron2_repo

# ROS
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
      python-rosinstall \
      python-rosinstall-generator \
      python-wstool \
      python-rosdep \
      python-vcstools \
      && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update

# catkin tools
RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
     python-catkin-tools \
     && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y --allow-unauthenticated \
      ros-melodic-ros-core \
      ros-melodic-libuvc-camera \
      ros-melodic-image-view \
      ros-melodic-cv-bridge \
      ros-melodic-cv-camera \
      ros-melodic-actionlib \
      && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip install --upgrade pip setuptools
RUN pip3 install --upgrade rospkg catkin_pkg opencv-contrib-python empy
RUN pip install pillow==6.1

# for ros environments
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Prepare catkin build
RUN mkdir -p ~/catkin_build_ws/src

# ROS-opencv
RUN /bin/bash -c 'cd ~/catkin_build_ws/src; git clone -b melodic https://github.com/ros-perception/vision_opencv.git'

# Detector ROS messages
RUN /bin/bash -c 'cd ~/catkin_build_ws/src; git clone https://github.com/v4r-tuwien/object_detector_msgs.git'

# Detectron ROS wrapper
COPY . /root/catkin_build_ws/src/detectron2_ros/
# Run catkin build
RUN /bin/bash -c  '. /opt/ros/melodic/setup.bash; cd ~/catkin_build_ws; catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so; catkin build'

# source the catkin workspace
RUN echo "source ~/catkin_build_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
