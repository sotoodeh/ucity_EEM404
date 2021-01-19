#!/usr/bin/env python

__author__ = "Masoud S. Bahraini"
__copyright__ = "Copyright 2021, City, University of London"
__credits__ = ["Masoud S. Bahraini"]
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Masoud S. Bahraini"
__email__ = "sotoodeh.bahraini@city.ac.uk"
__status__ = "Production"

# The first section of the Python code is given in the following code fragment. It mainly
# involves importing rospy , sys , cv2 , sensor_msgs , cv_bridge, and the numpy module.
# The sensor_msgs dependency imports the ROS data type of both image and camera
# information type. The cv_bridge module imports the CvBridge class for converting the
# ROS image data type to the OpenCV data type and vice versa.
# NumPy is a python library used for working with arrays. It also has functions for
# working in domain of linear algebra, fourier transform, and matrices.

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np


# The following section of code is a class definition in Python that we will use to demonstrate
# CvBridge functions. The class is called cvBridgeDemo :

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "ucity_stereo"
        # Initialize the ros node
        rospy.init_node(self.node_name)
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        # Create the cv_bridge object
        self.bridge = CvBridge()
        # subscribe to stereo images and set the appropriate callbacks
        rospy.loginfo("Create subscribers for each topic")
        self.left_image = message_filters.Subscriber(
            "/stereo/left/image_rect", Image)
        self.right_image = message_filters.Subscriber(
            "/stereo/right/image_rect", Image)
        # self.info_sub = message_filters.Subscriber('/front/camera_info', CameraInfo)
        rospy.loginfo(
            "subscribed to /zed/zed_node/left_raw/image_raw_gray and /zed/zed_node/right_raw/image_raw_gray")
        rospy.loginfo(
            "Create sync filter. Use exact or approximate as appropriate.")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_image, self.right_image], queue_size=5, slop=0.1)
        rospy.loginfo("Registering callback")
        self.ts.registerCallback(self.image_callback)
        # Callback executed when the timer timeout
        rospy.Timer(rospy.Duration(0.03), self.show_img_cb)
        rospy.loginfo("Waiting for image topics to show ...")

    # Here is the callback to visualize the actual stereo images and processed stereo image:

    def show_img_cb(self, event=None):
        try:
            # Left and Right images
            cv2.namedWindow("Left_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("Left_Image", 25, 75)
            cv2.namedWindow("Right_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("Right_Image", 500, 75)
            cv2.imshow("Left_Image", self.cv_rgb_left)
            cv2.imshow("Right_Image", self.cv_rgb_right)

            # And one for the depth image
            cv2.moveWindow("Depth_Image", 950, 75)
            cv2.namedWindow("Depth_Image", cv2.WINDOW_NORMAL)
            cv2.imshow("Depth_Image", self.depth_disprity_image)
            # And one for the canny edges image
            cv2.moveWindow("edges_Image", 1400, 75)
            cv2.namedWindow("edges_Image", cv2.WINDOW_NORMAL)
            cv2.imshow("edges_Image", self.edges)
            cv2.resizeWindow("track_Image", 640, 480)
            cv2.waitKey(3)
        except:
            pass

    # The following code gives a callback function of the color image from stereo camera. When a sync stereo color
    # image is received on the "/front/left/image_raw" and "/front/right/image_raw" topics, it will call this function. This
    # function will process the color frame for edge detection. You can show the edge detected, feature points and the
    # raw color image:
    def image_callback(self, ros_image_left, ros_image_right):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Reading Left and Right Stereo Pair
            self.cv_rgb_left = self.bridge.imgmsg_to_cv2(
                ros_image_left, "rgb8")
            self.cv_rgb_right = self.bridge.imgmsg_to_cv2(
                ros_image_right, "rgb8")
            # convert the image to grayscale, if it is needed
            self.cv_gray_left = cv2.cvtColor(
                self.cv_rgb_left, cv2.COLOR_BGR2GRAY)
            self.cv_gray_right = cv2.cvtColor(
                self.cv_rgb_right, cv2.COLOR_BGR2GRAY)
            # compute disparity
            stereo = cv2.StereoBM_create(numDisparities=16*10, blockSize=11)
            self.depth_disprity_image = stereo.compute(
                self.cv_gray_left, self.cv_gray_right).astype(np.float32) / 16.0
            # find the edges in the left image
            self.edges = self.process_image(self.cv_rgb_left)
        except CvBridgeError as e:
            print(e)
            pass

    # The following function is called process_image(), and will convert the color image to
    # grayscale, then blur the image, and find the edges using the canny edge filter:
    def process_image(self, frame):
        # Convert to grayscale
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 40, 80)
        return edges

    # The following function will close the image window when the node shuts down:
    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


def main(args):
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
