# We don't need ubuntu:focal as it's already a layer in ros.
FROM ros:noetic-ros-core

LABEL maintainer="Rico Jia"

RUN apt update && \
    apt install -yq \
      build-essential \
      curl \
      dbus \
      doxygen \
      doxygen-latex \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      vim && \
    apt clean

ENTRYPOINT ["/bin/bash"]

