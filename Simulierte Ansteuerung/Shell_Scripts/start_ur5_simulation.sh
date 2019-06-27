source /srv-0/rstimmer/catkin_ws/devel/setup.bash
sleep 1
roslaunch ./Shell_Scripts/start_gazebo.launch &
sleep 5
roslaunch ./Shell_Scripts/start_ur5_simulation.launch &
