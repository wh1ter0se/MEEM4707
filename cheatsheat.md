## startup
roscore
roslaunch turtlebot3_gazebo_lab lab{lab_number}.launch

## run
rosrun turtlebot3_gazebo_lab {python script}

## extras
rosrun turtlebot3_gazebo_lab scripts/record.py
rosrun turtlebot3_gazebo_lab scripts/record_path.py
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
rosrun gmapping slam_gmapping
