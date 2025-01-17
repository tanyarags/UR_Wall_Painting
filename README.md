# UR_Wall_Painting

This is a mini project using UR10e robotic arm to paint a wall leaving windows in between.

Currently, you can compile and run launch file:
  colcon build  
  ros2 launch ur_painting_planner ur10e_gazebo.launch.py 

- It will launch the gazebo world, with wall, arm and planner.
- You can play around by giving navigation goals on rviz.


  Things to be done:
- add github submodules - Universal Robot description and driver repos, moveit (including moveit task constructor).
- compile them
