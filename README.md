# ucity_EEM404
ROS command, codes and links for EEM404


## Publish the stereo images in Jackal Gazebo

roslaunch jackal_gazebo jackal_world.launch config:=front_bumblebee2

http://wiki.ros.org/jackal_description

base --> Base Jackal, includes IMU and GPS.

front_laser --> Include front-facing LMS1xx LIDAR.

front_bumblebee2 --> Includes front-facing Pointgrey Bumblebee2.

front_flea3 --> Includes front-facing Pointgrey Flea3.


# image view 
rostopic list
rqt_image_view
rosrun rviz rviz
roslaunch jackal_viz view_robot.launch


# stereo image rectification 

roslaunch jackal_gazebo jackal_world.launch config:=front_bumblebee2

http://wiki.ros.org/stereo_image_proc

ROS_NAMESPACE=front rosrun stereo_image_proc stereo_image_proc

rqt_image_view

rosrun image_view stereo_view stereo:=/front image:=image_rect

rosrun rqt_robot_steering rqt_robot_steering 

http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters
rosrun rqt_reconfigure rqt_reconfigure

rosrun rviz rviz
roslaunch jackal_viz view_robot.launch


