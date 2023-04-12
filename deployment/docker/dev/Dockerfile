FROM ros:humble

RUN apt update && \
  \
  # Make sure all necessary ROS2 Humble packages are installed \
  apt install -y \
  ros-humble-desktop-full python3-colcon-common-extensions \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-camera-calibration-parsers v4l2loopback-utils \
  ros-humble-topic-tools ros-humble-robot-localization \
  ros-humble-ros-testing && \
  \
  # Prepare Arduino tools \
  apt install -y arduino-mk && \
  cd /usr/local && \
  (curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh) && \
  /usr/local/bin/arduino-cli lib install Servo && \
  \
  # Prepare X11 environment for remote access \
  apt install -y x11vnc xvfb fluxbox && \
  \
  # The following packages help to avoid unnecessary suffering while using docker \
  apt install -y net-tools vim less && \
  \
  # The following OS packages are required for development and testing purposes \
  apt install -y socat xterm && \
  apt install -y ansible && \
  ansible-galaxy collection install community.general && \
  \
  # Prepare places to put OpenVMP files to \
  mkdir -p /root/.openvmp && \
  echo DOCK > /root/.openvmp/id

COPY deployment/shared/files/config/fastdds_client.xml /root/.openvmp/fastdds_client.xml
COPY deployment/shared/files/env.sh /root/.openvmp/env.sh

RUN \
  GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" \
  git clone --recursive https://github.com/openvmp/openvmp.git /root/openvmp

RUN \
  source /opt/ros/humble/setup.bash &&  \
  cd /root/openvmp/platform && \
  COLCON_HOME=$(pwd) colcon build || true

ENV HOSTNAME=openvmp-docker-dev
ENV DISPLAY=:0
EXPOSE 5900/tcp

SHELL ["/bin/bash", "-c"]
RUN \
  source /root/.openvmp/env.sh && \
  \
  (/usr/bin/Xvfb $DISPLAY -screen 0 1024x768x16 &) && \
  (sleep 1 && /usr/bin/fluxbox &) && \
  sleep 2 && /usr/bin/x11vnc -display $DISPLAY -bg -forever -nopw -quiet -listen 0.0.0.0 -xkb

ENTRYPOINT \
  cd /root/openvmp/platform && \
  . /root/.openvmp/env.sh && \
  bash
