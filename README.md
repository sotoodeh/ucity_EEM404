# ucity_EEM404
ROS command, codes and links for EEM404

## creating hello_world package

- mkdir -p catkin_ws_city/src && cd my_catkin_ws_city/src

src folder --> is the place where you can create, or clone, new packages from repositories. ROS packages only build and create an executable when it is in the src folder. When we execute the catkin_make command from the workspace folder, it checks inside the src folder and build each packages.

- catkin_create_pkg hello_world roscpp rospy std_msgs  # http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage
- cd ..
- tree
- rosdep install --from-paths src --ignore-src --rosdistro melodic
- catkin_make       # to build the nodes http://wiki.ros.org/catkin/commands/catkin_make 
- optional: catkin_make -DCMAKE_BUILD_TYPE=Release # or not specifying the type --> Debug

build folder --> the catkin tool creates some build files and intermediate cache CMake files inside the build folder. These cache files help prevent from rebuilding all the packages when running the catkin_make command; for example, if you build five packages, and then add a new package to the src folder, only the new package builds during the next catkin_make command. This is because of those cache files inside the build folder. If you delete the build folder, all the packages build again.

devel Folder --> When we run the catkin_make command, each package is built, and if the build process is successful, the target executable is created. The executable
is stored inside the devel folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation. 

The .bashrc file is a script file that’s executed when a user logs in. The file itself contains a series of configurations for the terminal session. This includes setting up or enabling: coloring, completion, shell history, command aliases, and more. 

Before continuing source your new setup.*sh file. 

- source devel/setup.bash

talker and listener codes in python: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

to make python codes executable:

- chmod +x scripts/talker_py.py scripts/listener_py.py

run the nodes in two new terminals:

- rosrun hello_world talker_py.py
- rosrun hello_world listener_py.py


Visualizing a Computing Graph

- roslaunch hello_world talker_listener_cpp.launch   # for cpp
- roslaunch hello_world talker_listener_py.launch   # for python
- rqt_graph

rosout is a handy tool for debugging. http://wiki.ros.org/rosout rosout subscribes to the standard /rosout topic, records these messages in a textual log file, and rebroadcasts the messages on /rosout_agg

you can pull up messages using rqt_console: http://wiki.ros.org/rqt_console




## installing Visual Studio Code (VSCode)

https://code.visualstudio.com/Download

- code .
- extensions: c/c++ , python , ROS , Cmake

To fix the issue with rospkg:

- sudo apt install python3-pip -y
- pip3 --version
- sudo apt update
- sudo apt install python-pip -y
- pip --version
- sudo pip3 install rospkg
- sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy


## jackal robot in Gazebo  

Jackal is a small, fast, entry-level field robotics research platform. It has an onboard computer, GPS and IMU fully integrated with ROS for out-of-the-box autonomous capability. As with all Clearpath robots, Jackal is plug-and-play compatible with a huge list of robot accessories to quickly expand your research and development.

To install the package from the source 

- sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation ros-melodic-jackal-viz ros-melodic-jackal-tutorials ros-melodic-jackal-gazebo ros-melodic-jackal-control ros-melodic-jackal-msgs ros-melodic-jackal-description ros-melodic-jackal-cartographer-navigation

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


## image view 
- rostopic list
- rqt_image_view
- rosrun rviz rviz
- roslaunch jackal_viz view_robot.launch


## stereo image rectification 

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

## running libviso2 on jackal in Gazebo

clone and catkin_make libviso2 from https://github.com/srv/viso2

- ROS_NAMESPACE=front rosrun stereo_image_pc stereo_image_proc
- rosrun viso2_ros stereo_odometer stereo:=front image:=image_rect
- roslaunch jackal_gazebo jackal_world.launch config:=front_bumblebee2
- rosrun rqt_robot_steering rqt_robot_steering
- rostopic echo /stereo_odometer/pose

## running rtabmap on jackal in Gazebo

## rosbag 

- rosbag --help # http://wiki.ros.org/rosbag/Commandline
- rosbag info 

To play a rosbag and remap the topics

- rosbag play --queue=1 filtered_2020-10-23-15-07-52.bag /zed/zed_node/left_raw/camera_info:=/zed/zed_node/left/camera_info /zed/zed_node/right_raw/camera_info:=/zed/zed_node/right/camera_info /zed/zed_node/left_raw/image_raw_gray:=/zed/zed_node/left/image_raw /zed/zed_node/right_raw/image_raw_gray:=/zed/zed_node/right/image_raw
- ROS_NAMESPACE=zed/zed_node rosrun stereo_image_proc stereo_image_proc
- rqt_image_view

to convert kitti dataset to rosbag: https://github.com/tomas789/kitti2bag


## install opencv

1. Open Terminal and run this command to update Apt:

- sudo apt-get update

2. Now, run these commands to install NumPy, SciPy, and OpenCV with Python bindings:

- sudo apt-get install python-numpy
- sudo apt-get install python-scipy
- sudo apt-get install libopencv-*
- sudo apt-get install python-opencv

Enter Y whenever prompted about package installation.

for python:

- pip install opencv-python==3.3.0.10 opencv-contrib-python==3.3.0.10

A; B    # Run A and then B, regardless of success of A

A && B  # Run B if and only if A succeeded

A || B  # Run B if and only if A failed

A &     # Run A in background.

Tutorial: Stereo 3D reconstruction with OpenCV and python (https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-iii-95460d3eddf0)

python graphslam: https://pypi.org/project/graphslam/

python pose graph optimisation: https://github.com/shinsumicco/pose-graph-optimization

g2opy: https://github.com/uoip/g2opy

gtsam python: https://github.com/borglab/gtsam/blob/develop/python/gtsam/examples/Pose3SLAMExample_g2o.py

to use tf in python: http://wiki.ros.org/tf/TfUsingPython


## libviso2

in "odometer_base.h" change the "/odom" and "/base_link"

- local_nh.param("odom_frame_id", odom_frame_id_, std::string("/US_origin_ground"));
- local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("/US_origin"));

- rosbag play --pause 

- rosrun tf view_frames
- evince frames.pdf

- ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc


- cd ~/Documents/ucity_lcd_mantis
- source devel/setup.bash 
- rosrun ibow-lcd lcd_mantis /stereo/left/image_rect

- cd ~/Documents/libviso2_ws
- source devel/setup.bash 
- rosrun viso2_ros stereo_odometer image:=image_rect
- rostopic echo /stereo_odometer/pose


## rectify cpp

- tree

Veles.

  ├── cam0
  
  ├── cam1
  
  └── stereo
  

convert the bagfile using lunch file /home/ros/Documents/3D-2D_Stereo_VO/stereo_vo_catkin/src/ucity_kinect/scripts

- ./lab_stereo_calibration /path/to/raw camX/%05d.png /path/to/navcam-calibration.yml -s2208x1242


## Jackal robot

remotely connection:

- ssh administrator@192.168.131.1
- The default password is clearpath

CONNECTING TO WIFI ACCESS POINT

- wicd-curses

find the city-guest, then config it and enter the password, then connect 

To record a rosbag:

- rosbag record /topic1 /topic2 /topic3
- /cmd_vel /odometry/filtered /tf /tf_static /zed/zed_node/left_raw/camera_info /zed/zed_node/left_raw/image_raw_gray /zed/zed_node/right_raw/camera_info /zed/zed_node/right_raw/image_raw_gray

Copy a Remote File to a Local System using the scp Command

- scp administrator@192.168.131.1:/remote/file.txt /local/directory

instruction to connect to real robot via wifi

https://answers.ros.org/question/297000/cannot-connect-to-jackal-clear-path-from-my-laptop/

## Kinect V1

- sudo apt-get install ros-melodic-openni-camera
- sudo apt-get install ros-melodic-openni-launch
- roslaunch openni_launch openni.launch

## Kinect V2

follow this instruction: https://www.programmersought.com/article/8067544172/

you can also follow this instruction to install libfreenect2 for ubuntu: https://github.com/OpenKinect/libfreenect2


## VO python - real-time working

- roscore
- rosbag play --pause -r 1 ../Data\ Set/Jackal-CG41/filtered_2020-10-23-15-07-52.bag /zed/zed_node/left_raw/image_raw_gray:=/zed/zed_node/left/image_raw /zed/zed_node/right_raw/image_raw_gray:=/zed/zed_node/right/image_raw /zed/zed_node/left_raw/camera_info:=/zed/zed_node/left/camera_info /zed/zed_node/right_raw/camera_info:=/zed/zed_node/right/camera_info
- ROS_NAMESPACE=zed/node rosrun stereo_image_proc stereo_image_proc
- rosrun ucity_vo_py ros_stereo_vo_work.py
