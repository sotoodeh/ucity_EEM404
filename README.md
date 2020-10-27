# ucity_EEM404
ROS command, codes and links for EEM404

## creating hello_world package

- mkdir -p catkin_ws/src && cd catkin_ws/src

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

- chmod +x scripts/talker.py scripts/listener.py

run the nodes in two new terminals:

- rosrun hello_world talker.py
- rosrun hello_world listener.py


Visualizing a Computing Graph

- roslaunch hello_world talker_listener.launch
- rqt_graph

rosout is a handy tool for debugging. http://wiki.ros.org/rosout rosout subscribes to the standard /rosout topic, records these messages in a textual log file, and rebroadcasts the messages on /rosout_agg

you can pull up messages using rqt_console: http://wiki.ros.org/rqt_console




## installing Visual Studio Code (VSCode)

https://code.visualstudio.com/Download


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






A; B    # Run A and then B, regardless of success of A

A && B  # Run B if and only if A succeeded

A || B  # Run B if and only if A failed

A &     # Run A in background.
