FROM ros:galactic-ros-base-focal


RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y terminator
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y nano iputils-ping 



RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-rmw-cyclonedds-cpp




# Build the Simulation application
#RUN cd /home/fry/ros_ws
#RUN /bin/bash -c " colcon build --symlink-install"
#RUN apt-get clean
RUN apt-get update && apt-get install -y \
    ros-galactic-rviz2 \
    ros-galactic-rqt \
    ros-galactic-rqt-common-plugins \
    ros-galactic-ros-core \
    ros-galactic-geometry2 \
    ros-galactic-joint-state-publisher-gui \
    ros-galactic-slam-toolbox \
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-turtlebot3* \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-gazebo-ros \
    ros-galactic-rviz2 \
    ros-galactic-xacro \
    ros-galactic-robot-localization \
    ros-galactic-launch-testing \
    && rm -rf /var/lib/apt/lists/*
    
  

# Set environment variables
ENV DISPLAY=host.docker.internal:0  

# Create user
RUN useradd klein -m -p klein -s /bin/bash
RUN echo "klein:klein" | chpasswd
RUN adduser klein sudo
RUN /bin/bash -c "adduser klein dialout"
USER klein
RUN mkdir /home/klein/xdg
RUN /bin/bash -c "echo source /opt/ros/galactic/setup.bash >> /home/klein/.bashrc"
RUN /bin/bash -c "echo source /home/klein/ros_ws/install/setup.bash >> /home/klein/.bashrc"

WORKDIR /home/klein/ros_ws

# USER root

ENTRYPOINT ["/bin/bash"]

