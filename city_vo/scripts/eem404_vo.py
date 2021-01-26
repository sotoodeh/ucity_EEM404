#!/usr/bin/env python

__author__ = "Masoud S. Bahraini"
__copyright__ = "Copyright 2021, City, University of London"
__credits__ = ["Masoud S. Bahraini"]
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Masoud S. Bahraini"
__email__ = "sotoodeh.bahraini@city.ac.uk"
__status__ = "Production"

# VO code for cg41 rosbag - my functions

'''
    catkin_create_pkg city_vo rospy sensor_msgs cv_bridge message_filters
    catkin_make (or catkin build)
    source devel/setup.bash
    chmod +x src/ucity_vo/scripts/eem404.py
    rosrun ucity_vo eem404_vo.py 
    rosbag play --pause -r .02 eem404_bag_cg41.bag /zed/zed_node/left_raw/camera_info:=/stereo/left/camera_info /zed/zed_node/left_raw/image_raw_gray:=/stereo/left/image_raw /zed/zed_node/right_raw/camera_info:=/stereo/right/camera_info /zed/zed_node/right_raw/image_raw_gray:=/stereo/right/image_raw
    ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
'''

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
import math as m
import csv


# The following section of code is a class definition in Python that we will use to demonstrate
# VO functions. The class is called VO_Demo :


class VO_Demo():
    def __init__(self):
        self.node_name = "ucity_surf_VO"
        # Initialize the ros node
        rospy.init_node(self.node_name, anonymous=True)
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        # Create the cv_bridge object
        self.bridge = CvBridge()
        # initial pose
        self.Position = self.initialize_pose()
        # read calibration file
        self.read_calibration()
        # setting parameters
        self.show_images = True             # True or False
        self.plot_tracking_arrows = True
        # saving the poses
        self.save_txt = True                # True or False
        if self.save_txt:
            with open('vo_poses.csv', 'w') as csvfile:
                headers = ["pose_x", "pose_y", "pose_z"]
                self.writer = csv.DictWriter(csvfile, fieldnames=headers)
                self.writer.writeheader()
        self.feature_type = "SURF"   # "SURF", "SIFT", "ORB"
        self.feature_extractor = cv2.xfeatures2d.SURF_create()
        self.disparity_type = "BM"   # "BM+WLS","BM"
        self.process_frame = False    #
        rospy.loginfo("Parameters have been set ...")
        # disparity parameter setting
        self.disparity_setting()
        # subscribe to stereo images and set the appropriate callbacks
        rospy.loginfo("Create subscribers for each topic")
        self.left_image = message_filters.Subscriber(
            "/stereo/left/image_rect", Image)  # front stereo
        self.right_image = message_filters.Subscriber(
            "/stereo/right/image_rect", Image)
        # self.left_image = message_filters.Subscriber(
        #     "/zed/zed_node/left_raw/image_raw_gray", Image)
        # self.right_image = message_filters.Subscriber(
        #     "/zed/zed_node/right_raw/image_raw_gray", Image)
        rospy.loginfo(
            "subscribed to:  /stereo/left/image_rect  and  /stereo/right/image_rect")
        rospy.loginfo(
            "Create sync filter. Use exact or approximate as appropriate.")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_image, self.right_image], queue_size=2, slop=0.1)
        rospy.loginfo("Registering callback")
        self.ts.registerCallback(self.image_callback)
        # Callback executed when the timer timeout
        if self.show_images:
            rospy.Timer(rospy.Duration(0.1), self.show_stereo_keypints_cb)
        rospy.loginfo("Waiting for image topics to show ...")

    # Here is the callback to visualize the actual stereo images and processed stereo image:
    def show_stereo_keypints_cb(self, event=None):
        try:
            cv2.namedWindow("track_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("track_Image", 25, 75)
            cv2.imshow('track_Image', self.tracking_match_img)
            cv2.resizeWindow("track_Image", 1800, 540)
            cv2.waitKey(2)
        except:
            pass

    # The following code gives a callback function of the color image from stereo camera. When a sync stereo color
    # image is received on the "/stereo/left/image_rect" and "/stereo/right/image_rect" topics, it will call this function. This
    # function will process the color frame for edge detection. You can show the edge detected, feature points and the
    # raw color image:

    def image_callback(self, left_image, right_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Reading Left and Right Stereo Pair
            self.cv_rgb_left = self.bridge.imgmsg_to_cv2(
                left_image, "rgb8")
            self.cv_rgb_right = self.bridge.imgmsg_to_cv2(
                right_image, "rgb8")
            # Get height and width.
            # Note: It assumes that both pictures are the same size. They HAVE to be same size
            # self.height, self.width = self.cv_rgb_left.shape[:2]
            # convert the image to grayscale, if it is needed
            self.cv_gray_left_cur = cv2.cvtColor(
                self.cv_rgb_left, cv2.COLOR_BGR2GRAY)
            self.cv_gray_right_cur = cv2.cvtColor(
                self.cv_rgb_right, cv2.COLOR_BGR2GRAY)
            # get next pose from VO
            self.visual_odometry()
        except CvBridgeError as e:
            print(e)
            pass

    # The following function will close the image window when the node shuts down:

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()

    # The following function will read the calibration parameters
    def read_calibration(self):
        try:
            # Setting the camera parameters
            # Focal length in pixels:
            self.f = 700.2050170898438
            # Optical center (the principal point), in pixels:
            self.cx = 636.64501953125
            self.cy = 367.1289978027344
            # The camera intrinsic matrix:
            self.K = np.array(
                [[self.f, 0, self.cx, 0, self.f, self.cy, 0, 0, 1]], dtype="double").reshape(3, 3)
            self.K_inv = np.linalg.inv(self.K)
            # The radial and tangential distortion coefficients:
            # (zeros) if we subscribe to rectified and undistorted images
            self.d = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, 5)
            # projection matrix
            # the left camera
            Proj1 = np.zeros((3, 4))
            Proj1[0, 0] = 700.2050170898438
            Proj1[0, 2] = 636.64501953125
            Proj1[1, 1] = 700.2050170898438
            Proj1[1, 2] = 367.1289978027344
            Proj1[2, 2] = 1.0
            self.P1_roi = Proj1
            # the right camera
            Proj2 = np.zeros((3, 4))
            Proj2[0, 0] = 700.7449951171875
            Proj2[0, 2] = 640.8800048828125
            Proj2[0, 3] = -84.02250671386719
            Proj2[1, 1] = 700.7449951171875
            Proj2[1, 2] = 361.7510070800781
            Proj2[2, 2] = 1.0
            self.P2_roi = Proj2
            # the baseline:
            self.base = -self.P2_roi[0, 3]/self.P2_roi[0, 0]
            rospy.loginfo("calibration data has been loaded.")
            print("base= " + str(self.base))
            print("Proj1= " + str(self.P1_roi))
            print("Proj2= " + str(self.P2_roi))
            print("K= " + str(self.K))
            print("D= " + str(self.d))
        except:
            rospy.loginfo("failed to read the calibration parameters")
            pass

    def initialize_pose(self):
        # Setting the initial pose of the vehicle
        Position = [[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]
        theta = 0
        IniRot = [[m.cos(theta), 0, m.sin(theta), 0],
                  [0, 1, 0, 0],
                  [-m.sin(theta), 0, m.cos(theta), 0],
                  [0, 0, 0, 1]]
        Position = np.dot(Position, IniRot)
        print("initial pose: " + str(Position))
        return Position

    def disparity_setting(self):
        # Setting parameters based on disparity type selected
        try:
            self.left_matcher = cv2.StereoBM_create(
                numDisparities=16*10, blockSize=15)
        except:
            print("Incorrect Disparity Method Selected")
            sys.exit()
        rospy.loginfo("disparity type has been set to " + self.disparity_type)

    def get_descriptors_and_points(self, gray_image):
        # extract keypoints and corresponding features from the input image
        keypoints, descriptors = self.feature_extractor.detectAndCompute(
            gray_image, None)
        return keypoints, descriptors

    def stereo_and_tracking_2d_3d(self):
        kps_left, des_left = self.get_descriptors_and_points(
            self.cv_gray_left_pre)
        kps_right, des_right = self.get_descriptors_and_points(
            self.cv_gray_right_pre)
        kps_left_cur, des_left_cur = self.get_descriptors_and_points(
            self.cv_gray_left_cur)
        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        # Get the matches based on the descriptors
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches_stereo = flann.knnMatch(des_left, des_right, k=2)
        matches_tracking = flann.knnMatch(des_left, des_left_cur, k=2)

        # Need to draw only good matches for stereo matching, so create a mask
        matchesMask_all = [[0, 0] for i in range(len(matches_stereo))]
        pts_left_image_cur = []
        pts_left_image = []
        pts_right_image = []
        # ratio test as per Lowe's paper
        for i, (m, n) in enumerate(matches_stereo):
            mm, nn = matches_tracking[i]
            if m.distance < 0.3*n.distance and mm.distance < 0.3*nn.distance:
                matchesMask_all[i] = [1, 0]
                pts_left_image.append(kps_left[m.queryIdx].pt)
                pts_right_image.append(kps_right[m.trainIdx].pt)
                pts_left_image_cur.append(kps_left_cur[mm.trainIdx].pt)

        if self.show_images:
            draw_params = dict(matchColor=(0, 255, 0),
                               singlePointColor=(205, 0, 0),
                               matchesMask=matchesMask_all,
                               flags=cv2.DrawMatchesFlags_DEFAULT)
            self.stereo_match_img = cv2.drawMatchesKnn(
                self.cv_gray_left_pre, kps_left, self.cv_gray_right_pre, kps_right, matches_stereo, None, **draw_params)

        # Now we have the list of best matches from both the stereo images.
        # Lets find the Fundamental Matrix.
        pts1 = np.int32(pts_left_image)
        pts2 = np.int32(pts_right_image)
        pts1_cur = np.int32(pts_left_image_cur)
        # F, mask = cv2.findFundamentalMat(pts1, pts1_cur, cv2.FM_LMEDS)
        # # We select only inlier points
        # pts1 = pts1[mask.ravel() == 1]
        # pts2 = pts2[mask.ravel() == 1]
        # pts1_cur = pts1_cur[mask.ravel() == 1]

        self.pts_left_stereo = pts1
        self.pts_right_stereo = pts2
        self.pts_left_tracking_cur = pts1_cur
        # to maintain the feature points in a list
        first_match_points = np.zeros(
            (len(pts_left_image), 2), dtype=np.float32)
        second_match_points = np.zeros_like(first_match_points)
        for i in range(len(pts_left_image)):
            first_match_points[i] = pts_left_image[i]
            second_match_points[i] = pts_left_image_cur[i]

        if self.plot_tracking_arrows:
            img = self.stereo_match_img
            for i in range(len(first_match_points)):
                cv2.line(img, tuple(first_match_points[i]), tuple(
                    second_match_points[i]), color=(0, 0, 255))
            self.tracking_match_img = img
        # return pts_left_image, pts_right_image

    def motion_estimation_pnp(self):
        # triangulation to optain 3d position of the correspondance points in the stereo image
        points_3d1 = self.generate3DPoints(
            self.pts_left_stereo, self.pts_right_stereo)
        # print('points3d_py= ' + str(points_3d_py))
        image_points1 = np.array(self.pts_left_tracking_cur, dtype="float64")
        # points 3D and points 2D must have same number of vertices
        assert points_3d1.shape[0] == image_points1.shape[0]

        # solve pnp problem
        _, rotation_vector, translation_vector, _ = cv2.solvePnPRansac(
            points_3d1, image_points1, self.K, self.d, flags=cv2.SOLVEPNP_ITERATIVE, confidence=.99)  # , iterationsCount=1000)
        # print("Rotation Vector:\n {0}".format(rotation_vector))
        # print("Translation Vector:\n {0}".format(translation_vector))
        # Finding Rotation matrix and translation vector
        # initialisation the matrix and vector
        Rmat = np.zeros((4, 4), np.float32)
        Rmat[3, 3] = 1.0

        Trans = np.zeros((4, 4), np.float32)
        Trans[0] = (1, 0, 0, 0)
        Trans[1] = (0, 1, 0, 0)
        Trans[2] = (0, 0, 1, 0)
        Trans[3] = (0, 0, 0, 1)
        # end of initialisation
        rodRotMat = cv2.Rodrigues(rotation_vector)
        Rmat[:3, :3] = rodRotMat[0]
        # print('Rmat= ' + str(Rmat))
        Trans[0, 3] = translation_vector[0]
        Trans[1, 3] = translation_vector[1]
        Trans[2, 3] = translation_vector[2]
        # Updating the odometry
        self.Position = np.dot(Rmat, self.Position)
        # applying the transformation
        self.Position = np.dot(Trans, self.Position)
        print('Position= '+str(self.Position))

    def generate3DPoints(self, points2D_L, points2D_R):
        numPoints = points2D_L.shape[0]
        d3dPoints = np.ones((numPoints, 3))

        for i in range(numPoints):
            pLeft = points2D_L[i, :]
            pRight = points2D_R[i, :]

            X = np.zeros((4, 4))
            X[0, :] = pLeft[0] * self.P1_roi[2, :] - self.P1_roi[0, :]
            X[1, :] = pLeft[1] * self.P1_roi[2, :] - self.P1_roi[1, :]
            X[2, :] = pRight[0] * self.P2_roi[2, :] - self.P2_roi[0, :]
            X[3, :] = pRight[1] * self.P2_roi[2, :] - self.P2_roi[1, :]

            [u, s, v] = np.linalg.svd(X)
            v = v.transpose()
            vSmall = v[:, -1]
            vSmall /= vSmall[-1]

            d3dPoints[i, :] = vSmall[0:-1]
        # print('generate3DPoints = ' + str(d3dPoints))
        return d3dPoints

    def write_to_file(self):
        poses_dict = {
            "pose_x": self.Position[0, 3], "pose_y": self.Position[1, 3], "pose_z": self.Position[2, 3]}
        with open('vo_poses.csv', 'a') as csvfile:
            headers = ["pose_x", "pose_y", "pose_z"]
            self.writer = csv.DictWriter(csvfile, fieldnames=headers)
            self.writer.writerow(poses_dict)

    def visual_odometry(self):
        if self.process_frame == True:
            # stereo matching and temporal tracking using Point matching between rich feature descriptors
            self.stereo_and_tracking_2d_3d()
            # pose estimation using essential matrix
            self.motion_estimation_pnp()
            # saving the current pair for the next step
            self.cv_gray_left_pre = self.cv_gray_left_cur
            self.cv_gray_right_pre = self.cv_gray_right_cur
        else:
            self.cv_gray_left_pre = self.cv_gray_left_cur
            self.cv_gray_right_pre = self.cv_gray_right_cur
            self.process_frame = True

        if self.save_txt:
            self.write_to_file()


def main(args):
    try:
        VO_Demo()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
