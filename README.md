# ucity_EEM404
ROS command, codes and links for EEM404

## creating hello_world package

mkdir -p catkin_ws/src && cd catkin_ws/src

catkin_create_pkg hello_world roscpp rospy std_msgs

cd ../..

tree

catkin_make

source devel/setup.bash



## jackal robot in Gazebo  

Jackal is a small, fast, entry-level field robotics research platform. It has an onboard computer, GPS and IMU fully integrated with ROS for out-of-the-box autonomous capability. As with all Clearpath robots, Jackal is plug-and-play compatible with a huge list of robot accessories to quickly expand your research and development.

- To install the package from the source 

sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation ros-melodic-jackal-viz ros-melodic-jackal-tutorials ros-melodic-jackal-gazebo ros-melodic-jackal-control ros-melodic-jackal-msgs ros-melodic-jackal-description ros-melodic-jackal-cartographer-navigation

After installation you may need to restart your machine

You may check this directory to see the launch file

- cd /opt/ros/melodic/share/jackal_gazebo/launch

- roslaunch jackal_gazebo jackal_world.launch

To control the robot, you can use: 

1- rviz arrows: 

- roslaunch jackal_viz view_robot.launch

2- rqt gui

- rosrun rqt_robot_steering rqt_robot_steering

or 

- rosrun rqt_gui rqt_gui

3- command 

- rostopic list: 

/cmd_vel

/geometry_msgs/Twist

- rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.20
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.30" -r 3

or

- rostopic pub /cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'







## Publish the stereo images in Jackal Gazebo

- roslaunch jackal_gazebo jackal_world.launch config:=front_bumblebee2

http://wiki.ros.org/jackal_description

base --> Base Jackal, includes IMU and GPS.

front_laser --> Include front-facing LMS1xx LIDAR.

front_bumblebee2 --> Includes front-facing Pointgrey Bumblebee2.

front_flea3 --> Includes front-facing Pointgrey Flea3.


# image view 
- rostopic list
- rqt_image_view
- rosrun rviz rviz
- roslaunch jackal_viz view_robot.launch


# stereo image rectification 

- roslaunch jackal_gazebo jackal_world.launch config:=front_bumblebee2

http://wiki.ros.org/stereo_image_proc
- ROS_NAMESPACE=front rosrun stereo_image_proc stereo_image_proc
- rqt_image_view
- rosrun image_view stereo_view stereo:=/front image:=image_rect
- rosrun rqt_robot_steering rqt_robot_steering 

http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters
- rosrun rqt_reconfigure rqt_reconfigure
- rosrun rviz rviz
- roslaunch jackal_viz view_robot.launch


