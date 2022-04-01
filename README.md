# testrobot
TAB 1:
export TURTLEBOT3_MODEL=waffle_pi

roslaunch turtlebot3_gazebo turtlebot3_house_me_human.launch

TAB 2:

export TURTLEBOT3_MODEL=waffle_pi

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/apramani/catkin_ws/src/turtlebot3/turtlebot3_navigation/maps/building4.yaml


TAB 3:

roslaunch rtamt4ros ros_stl_monitor.launch




