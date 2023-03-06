#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstacleAvoidance():
    def __init__(self):
        
        #initializing the node
        rospy.init_node('obstacle_avoidance',anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)          #publisher - velocity commands
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_update)    #subscriber - Laser scans, callback function that stores the subscribed data in a variable
        self.rate = rospy.Rate(10)
        self.laser_scan90 = 1 #random value
        self.laser_scan270 = 1 #random value
        self.lookahead_dist = 1 #random value
        
    def scan_update(self,data):
        window = 5
        lookahead_window = 15
        self.lookahead_dist = min(3.5,(sum(data.ranges[0:lookahead_window])+sum(data.ranges[360-lookahead_window:360]))/(2*lookahead_window))
        self.laser_scan90 = min(3.5,sum(data.ranges[89-window:89+window])/(2*window))
        self.laser_scan270 = min(3.5,sum(data.ranges[269-window:269+window])/(2*window))

    def currentError(self):
        d_90 = self.laser_scan90
        d_270 = self.laser_scan270
        return d_90-d_270
    
    def pController_lat(self,error_lat,dist):
        pGain_lat_x = 1
        pGain_lat_y = 0.1
        return pGain_lat_x*(1/dist) + pGain_lat_y*error_lat
    
    def pController_long(self,dist):
        pGain_long = 0.05
        return pGain_long*dist

    def avoid(self):
        self.vel_msg = Twist()
        
        
        while not rospy.is_shutdown():
           if self.lookahead_dist == 0:
               self.lookahead_dist = 3.5
           lin_x = min(0.22,self.pController_long(self.lookahead_dist))
           self.vel_msg.linear.x = lin_x
           
           #changing the angular about z based on the error value
           error_lat = self.currentError()
           ang_z = min(2.84,self.pController_lat(error_lat,self.lookahead_dist))
           self.vel_msg.angular.z = ang_z
           
           #publishing these values
           self.vel_pub.publish(self.vel_msg)
           self.rate.sleep()
           
if __name__=='__main__':
    try:
        x = obstacleAvoidance()
        x.avoid()
    except rospy.ROSInterruptException: pass