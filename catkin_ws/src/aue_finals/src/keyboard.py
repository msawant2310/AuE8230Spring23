#!/usr/bin/env python3
import time
import threading

import cv2
import torch
import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class CustomThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.value = 'a'

    def run(self):
        while True:
            self.value = input()
            print(f'you entered {self.value}')


class KeyBoard():
    def __init__(self):

        # start the keyboard detection
        self.a = CustomThread()
        self.a.start()

        # initializing the node
        rospy.init_node('status', anonymous=True)
        # pulisher - staus of the robot
        self.status_pub = rospy.Publisher('/status', String, queue_size=10)
        # publish frequency
        self.rate = rospy.Rate(20)

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f'working on {device}')
        # yolo model for object detection
        self.yolo_model = torch.hub.load(
            'ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.yolo_model.cuda()
        self.yolo_model.eval()
        # start the camera
        self.count = 0
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/robot1/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)

        # init status
        self.keyboard_input = 'a'
        self.once_time_flag = False
    
    def detect_stop_sign(self):
        # perform object detection on the camera image
        results = self.yolo_model(self.cv2_image)

        data = results.pandas().xyxy[0]

        # the stop sign is class 11. this determines if a stop sign was
        # detected in the camera frame
        is_stop = True if 11 in data['class'].values else False
        row = ...

        # if stop sign is detected, loop through to get the row of the class
        if is_stop:
            # rospy.loginfo("stop sign detected!")
            for i, row in enumerate(data.loc[:, 'class']):
                if row == 11:
                    is_stop = True
                    row = data.iloc[i]
                    break

            # get the bounds of the box
            xmin = int(row.loc['xmin'])
            xmax = int(row.loc['xmax'])
            ymin = int(row.loc['ymin'])
            ymax = int(row.loc['ymax'])

            # calculate the diagonal distance
            distance = np.linalg.norm([xmax - xmin, ymax - ymin])
            rospy.loginfo(f"diagonal distance is {distance}")

            if distance >= 70 and not self.once_time_flag:
                rospy.loginfo("stop the turtlebot")
                self.keyboard_input = 'd'
                self.count += 1
                if self.count >= 5:
                    self.once_time_flag = True
                    self.keyboard_input = self.a.value

            self.draw_box(xmin, xmax, ymin, ymax)

    def draw_box(self, xmin, xmax, ymin, ymax):
        # top
        cv2.line(self.cv2_image, (xmin, ymax), (xmax, ymax), (0, 255, 0), 2)
        # bottom
        cv2.line(self.cv2_image, (xmin, ymin), (xmax, ymin), (0, 255, 0), 2)
        # left
        cv2.line(self.cv2_image, (xmin, ymin), (xmin, ymax), (0, 255, 0), 2)
        # right
        cv2.line(self.cv2_image, (xmax, ymin), (xmax, ymax), (0, 255, 0), 2)

    def camera_callback(self, msg):
        try:
            self.cv2_image = cv2.resize(self.bridge_object.compressed_imgmsg_to_cv2(
                msg, desired_encoding="rgb8"), (400, 400), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            rospy.loginfo(e)

    def publish_status(self):
        ''' publish the status from the keyboard'''
        close_windows = False
        while not rospy.is_shutdown():

            if self.once_time_flag == False:

                self.detect_stop_sign()
                cv2.imshow("Sign detection", cv2.cvtColor(
                    self.cv2_image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

            else:
                if close_windows == False:
                    cv2.destroyWindow("Sign detection")
                    rospy.loginfo("close the window!")
                    close_windows = True

            if self.keyboard_input != 'd':
                self.keyboard_input = self.a.value

            # rospy.loginfo(f"keyboard input is {self.keyboard_input}")
            self.status_pub.publish(self.keyboard_input)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = KeyBoard()
        rospy.sleep(2)
        node.publish_status()
    except rospy.ROSInterruptException:
        pass
