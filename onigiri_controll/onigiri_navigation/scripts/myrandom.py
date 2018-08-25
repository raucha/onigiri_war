#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

spd = 0.5
th = 0
dist = 1000

def callback(arg):
    global dist
    l = len(arg.ranges)
    t = arg.ranges[0:int(l/24)] + arg.ranges[-int(l/24):]
    dist = min(t)
    # dist = arg.ranges[int(len(arg.ranges)/2)]

class RandomBot():
    def __init__(self, bot_name):
        # bot name
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def calcTwist(self):
        global spd, th
        print "dist ",dist
        tmp = min((dist-0.4)*3,1.8)
        spd = random.normalvariate(tmp, 0.4)
        th = random.normalvariate(th*0.9, 1.0)
        # th = random.normalvariate(th*0.97, max(abs(th*0.4),1.5))
        # value = random.randint(1,1000)
        twist = Twist()
        twist.linear.x = spd; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(10) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('myrandom')
    rospy.Subscriber("scan", LaserScan, callback)
    bot = RandomBot('Random')
    bot.strategy()

