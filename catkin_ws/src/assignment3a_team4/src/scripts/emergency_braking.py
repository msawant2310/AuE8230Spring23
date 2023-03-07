#!/usr/bin/env python3
	

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class EmergyBrake():
    def __init__(self):
        
        #initializing the node
        rospy.init_node('brake',anonymous=True)
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
        self.lookahead_dist = (sum(data.ranges[0:window])+sum(data.ranges[360-window:360]))/(2*window)
        self.laser_scan90 = (sum(data.ranges[89-window:89+window]))/(2*window)
        self.laser_scan270 = (sum(data.ranges[269-window:269+window]))/(2*window)
        

    def emergy_brake(self):
        self.vel_msg = Twist()
        constant_x_vel = 0.5
        while not rospy.is_shutdown():
           if self.lookahead_dist >= 1.0:
               self.vel_msg.linear.x = constant_x_vel
           else:
               self.vel_msg.linear.x = 0

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
        x = EmergyBrake()
        x.emergy_brake()
    except rospy.ROSInterruptException: pass
