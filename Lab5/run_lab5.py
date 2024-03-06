import os
import time
import subprocess

filename = os.path.abspath("perception_sickomode.py")

CMD_ROS = '/k "C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Community\\Common7\\Tools\\VsDevCmd.bat" \
    -arch=amd64 -host_arch=amd64&& set ChocolateyInstall=c:\\opt\\chocolatey&& c:\\opt\\ros\\melodic\\x64\\setup.bat'

CMD_ROSCORE = CMD_ROS + "&& roscore"
CMD_ROSLAUNCH = CMD_ROS + "&& roslaunch turtlebot3_gazebo_lab lab6.launch"
CMD_RVIZ = CMD_ROS + "&& "
CMD_ROSRUN = CMD_ROS + '&& rosrun turtlebot3_gazebo_lab "' + filename + '"'

for cmd in [CMD_ROSCORE, CMD_ROSLAUNCH, CMD_ROSRUN]:
    p = subprocess.Popen("cmd.exe " + cmd, creationflags=subprocess.CREATE_NEW_CONSOLE)
    time.sleep(15)
