#!/usr/bin/env python

# Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script.

import rospy                       # rospy has all the important ROS functions
from std_msgs.msg import String    # importing a string message type in Python
                                   # We have to use package_name.msg and import the required message type


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)   # To publish a topic called chatter with a std_msgs/String message type
                                                              # and a queue_size of 10.
    rospy.init_node('talker', anonymous=True)                 # The first argument is the name of the node ('talker'), 
                                                              # and the second argument is anonymous=True, which means the node can run on multiple instances.
    rate = rospy.Rate(10)                                     # 10Hz After creating the instance, we have to call the sleep() function inside it 
                                                              # to get the rate in effect. 
    # With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)
    
    # creating a loop to publish a msg
    while not rospy.is_shutdown():                            
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()                                         # to get the rate in effect.


if __name__ == '__main__':
    try:
        talker()                                             # talker function 
    except rospy.ROSInterruptException:
        pass
