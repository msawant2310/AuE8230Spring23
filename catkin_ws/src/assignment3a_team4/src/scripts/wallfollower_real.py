#!/usr/bin/env python3
import time

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowing():
    def __init__(self):
        
        #initializing the node
        rospy.init_node('wall_follower',anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)          #publisher - velocity commands
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_update)    #subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.rate = rospy.Rate(10)
        self.laser_scan90 = 1 #random value
        self.laser_scan270 = 1 #random value
        self.lookahead_dist = 1 #random value
        self.last_error_lat = 0
        self._last_time = None

    def scan_update(self,data):
        window = 15
        # self.lookahead_dist = data.ranges[0]
        self.lookahead_dist = (sum(data.ranges[0:3]+data.ranges[356:359]))/(2*3)
        self.laser_scan90 = (sum(data.ranges[89-3*window:89]))/(3*window)
        self.laser_scan270 = (sum(data.ranges[269:269+3*window]))/(3*window)
        if self.lookahead_dist == 0:
            self.lookahead_dist = 1.5
        rospy.loginfo(f"lookahead dist is {self.lookahead_dist}")
    def currentError(self):
        d_90 = self.laser_scan90
        d_270 = self.laser_scan270
        rospy.loginfo(f"Lidar error is {d_90 - d_270}")
        return d_90-d_270
    
    def pdController_lat(self,error_lat):
        pGain_lat = 5.0 #0.2 0.3
        dGain_lat = 0.5 #0.1 0.15
        now = self._current_time()
        dt = now - self._last_time if self._last_time is not None else 1e-10
        pValue = pGain_lat*error_lat
        dValue = dGain_lat*(error_lat - self.last_error_lat)/dt
        dValue = np.clip(dValue, -0.2, 0.2)
        dValue = 0
        self.last_error_lat = error_lat
        self._last_time = now 
        
        return pValue + dValue
    
    def pController_long(self,dist):
        pGain_long = 0.1
        return pGain_long*dist

    def wallFollowing(self):
        self.vel_msg = Twist()
           
        while not rospy.is_shutdown():
           lin_x = self.pController_long(min(1.5,self.lookahead_dist))
           #changing the angular about z based on the error value
           self.vel_msg.linear.x = lin_x
        #    self.vel_msg.linear.x = 0.12
        #    rospy.loginfo(f"linear vel is {self.vel_msg.linear.x}")
           error_lat = self.currentError()
           ang_z = min(self.pdController_lat(error_lat),0.2) #0.2
           self.vel_msg.angular.z = ang_z
            
           #publishing these values
           self.vel_pub.publish(self.vel_msg)
           self.rate.sleep()
    

    @staticmethod
    def _current_time():
        """Return the current time obtained from time.monotonic()
           if python >= 3.3.
        """
        try:
            # Get monotonic time to ensure that time deltas are positive
            return time.monotonic()
        except AttributeError:
            # Fall back to time.time() if time.monotonic() is not available
            return time.time()
           
if __name__=='__main__':
    try:
        x = WallFollowing()
        x.wallFollowing()
    except rospy.ROSInterruptException: pass
