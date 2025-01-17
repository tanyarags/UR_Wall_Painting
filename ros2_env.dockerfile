# Use the official ROS2 Humble base image
FROM ros:humble

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies and required packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins \
    ros-humble-nav2* \
    ros-humble-moveit* \
    ros-humble-moveit-msgs \
    ros-humble-moveit-resources \
    ros-humble-moveit-visual-tools \
    ros-humble-ur \
    ros-humble-ur-msgs \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros \
    ros-humble-perception-pcl \
    ros-humble-octomap* \
    ros-humble-ompl \
    ros-humble-eigenpy \
    ros-humble-ros-testing \
    ros-humble-ros-workspace \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-core \
    ros-humble-ament-cmake-export-dependencies \
    ros-humble-ament-cmake-libraries \
    ros-humble-ament-cmake-test \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-launch-testing-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-sklearn \
    libeigen3-dev \
    libcgal-dev \
    libpcl-dev \
    libompl-dev \
    build-essential \
    cmake \
    git \
    gazebo \
    tmux \
    vim \ 
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-capabilities \
    ros-humble-moveit-task-constructor-visualization \
    ros-humble-pose-broadcaster   
    #&& rm -rf /var/lib/apt/lists/*

# Install additional Python packages for custom planning
RUN pip3 install \
    numpy-quaternion \
    trimesh \
    rtree \
    networkx \
    python-fcl

RUN apt-get install -y ros-humble-moveit-py 


# Source ROS environment and build workspace
# RUN /bin/bash -c '. /opt/ros/humble/setup.bash; \
#     colcon build \
#         --cmake-args \
#             -DCMAKE_BUILD_TYPE=Release \
#             -DBUILD_TESTING=ON \
#             -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
# '

# # Source the workspace in .bashrc
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
#     echo "source /install/setup.bash" >> ~/.bashrc && \
#     echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
    
# Set up entry point
CMD ["/bin/bash"]
ENTRYPOINT /bin/bash
# COPY ./entrypoint.sh /
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
