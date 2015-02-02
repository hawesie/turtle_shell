#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from math import radians, fmod, pi

from turtlesim.msg import Pose


def radian_abs(r1, r2):
    if r1 >= 0 and r2 >= 0:
        return abs(r1 - r2)
    elif r1 < 0 and r2 < 0:
        return abs(r1 - r2)
    elif r1 >= 0: #means r2 < 0 also
        return abs(r1 - (r2 + (2 * pi)))
    else:
        return abs(r1 - (r2 - (2 * pi)))

def wrap_radians(rads):
    if rads > 0:
        return fmod(rads, pi)
    else:
        return fmod(rads, -pi)

class TurtleShell(object):
    """docstring for TurtleShell"""

    def __init__(self):
        super(TurtleShell, self).__init__()
        rospy.init_node('twister', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = None
        rospy.Subscriber("/turtle1/pose", Pose, self.callback, queue_size=1)
        while not rospy.is_shutdown() and self.theta == None:
            rospy.sleep(0.1)


    def callback(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = wrap_radians(data.theta)

    
    def rotate(self, theta):
        # assuming input in degrees, so convert to radians
        theta = radians(theta)
    
        # how far the robot can turn in a second
        # needs to be checked for individual robots
        speed = radians(20)

        # acceptable target rotation error
        error = radians(10)

        # target 
        target_theta = wrap_radians(self.theta + theta)

        tw = Twist()
        tw.angular.z = speed if theta > 0 else -speed

        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and radian_abs(target_theta, self.theta) > error: 
            print self.theta
            print target_theta
            print radian_abs(target_theta, self.theta)
            print '\n'
            pub.publish(tw)
            rate.sleep()

        tw.angular.z = 0
        pub.publish(tw) 

    

        
    
