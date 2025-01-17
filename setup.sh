# Clone additional repositories
cd src && \
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git && \
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git && \
git clone -b humble https://github.com/ros-planning/moveit2.git
# Or if you're building from source:
git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select moveit_task_constructor_core moveit_task_constructor_capabilities moveit_task_constructor_visualization
source install/setup.bas
cd ../
