FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rviz \
    ros-noetic-catkin \
    python3-catkin-tools \
    python3-pip \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    ros-noetic-xacro \
    ros-noetic-industrial-robot-client \
    ros-noetic-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=0

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["bash"]