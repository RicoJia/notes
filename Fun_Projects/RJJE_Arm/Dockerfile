# We don't need ubuntu:focal as it's already a layer in ros.
FROM ros:noetic-ros-core-focal

LABEL maintainer="Rico Jia"

# As of the creation of this file, the gpg key of ros docker for apt has expired. Therefore, we cannot run apt update and install at all without providing the key to apt-key
COPY gpg_key gpg_key 
RUN cat gpg_key | apt-key add -

RUN apt update && \
    apt install -yq \
    && apt-get install -yq --no-install-recommends \
      build-essential \
      curl \
      dbus \
      doxygen \
      doxygen-latex \
      git \
      gcc-arm-none-eabi \
      gdb-multiarch \
      libarmadillo-dev \
      liblapack-dev \
      libopenblas-dev \
      libnewlib-arm-none-eabi \
      libv4l-dev \
      libstdc++-arm-none-eabi-newlib \
      minicom \
      openocd \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
    ros-noetic-camera-info-manager \
      ros-noetic-catch-ros \
      ros-noetic-diagnostic-updater \
      ros-noetic-joint-state-controller \
      ros-noetic-joint-state-publisher \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-robot-state-publisher \
      ros-noetic-rqt \
      ros-noetic-rqt-common-plugins \
      ros-noetic-rqt-console \
      ros-noetic-rviz \
      ros-noetic-self-test \
      ros-noetic-usb-cam \
      ros-noetic-xacro \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-moveit \
      v4l-utils \
      vim && \
    apt clean \
    ; exit 0


#install joint_state_publisher_gui
RUN apt update \
    && apt upgrade -y \
    && apt install ros-noetic-joint-state-publisher-gui ; exit 0

# need /bin/bash to make the container long running
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && /bin/bash"


