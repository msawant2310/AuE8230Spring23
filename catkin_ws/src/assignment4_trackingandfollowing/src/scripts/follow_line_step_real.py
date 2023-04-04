#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from PID import PID

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.twist_object = Twist()
        self.height = 0
        self.width = 0
        self.traj_flag = False
        # self.pid_controller = PID(P = 0.0015, I = 0.000, D = 0.0007)
        self.pid_controller = PID(P = 0.0008, I = 0.000, D = 0.0005)
        self.pid_controller.clear()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        self.height, self.width, channels = cv_image.shape
        crop_img = cv_image[int((self.height/2)+100):int((self.height/2)+120)][1:int(self.width)]
        # crop_img = cv_image[int((self.height/2)+60):int((self.height/2)+120)][1:int(self.width)]
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        # lower_yellow = np.array([80,20,120])
        # upper_yellow = np.array([130,60,180])
        # Threshold the RGB image to get only white colors
        lower_white = np.array([100,100,100])
        upper_white = np.array([255,255,255])
        mask = cv2.inRange(crop_img, lower_white, upper_white)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            self.traj_flag = True
        except ZeroDivisionError:
            cx, cy = self.height/2, self.width/2
            self.traj_flag = False
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("crop image", crop_img)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        self.find_trajectory(cx, cy)

        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist_object.angular.z))
        # Make it start turning
        self.moveTurtlebot3_object.move_robot(self.twist_object)

    def find_trajectory(self, cx, cy):
        # if not see the trajectory
        if self.traj_flag == False:
            self.twist_object.linear.x = 0.07
            # rospy.loginfo("lose the trajectory")
        else:
            err = -1* (self.width/2 - cx)
            self.pid_controller.update(err)
            self.twist_object.linear.x = 0.07
            self.twist_object.angular.z = self.pid_controller.output
            rospy.loginfo("the error is {}".format(-1*err))

        # rospy.loginfo("cx value is {}, cy value is {}".format(cx, cy))


    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        main()
