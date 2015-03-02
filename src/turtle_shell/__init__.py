import rospy


from geometry_msgs.msg import Twist, Pose
from math import radians, fmod, pi, degrees

def wrap_radians(rads):
    if rads > 0:
        return fmod(rads, pi)
    else:
        return fmod(rads, -pi)

class TurtleShell(object):
    """docstring for TurtleShell"""

    def __init__(self, cmd_vel_topic):
        super(TurtleShell, self).__init__()
        rospy.init_node('turtle_shell', anonymous=True)
        self.x = 0
        self.y = 0
        self.theta = None
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
    
    def _wait_for_first_pose(self):
        if self.theta is None:
            while not rospy.is_shutdown() and self.theta is None:
                rospy.loginfo('Waiting for initial pose')
                rospy.sleep(0.1)
            rospy.loginfo('Got it')

    def heading(self):
        self._wait_for_first_pose()
        return degrees(self.theta)

    def position(self):
        self._wait_for_first_pose()
        return (self.x, self.y)

    def rotate(self, theta):

        self._wait_for_first_pose()

        # assuming input in degrees, so convert to radians
        theta = radians(theta)
    
        # how far the robot can turn in a second
        # needs to be checked for individual robots
        speed = radians(20)

        # acceptable target rotation error
        error = radians(15)

        # target 
        target_theta = wrap_radians(self.theta + theta)

        tw = Twist()
        tw.angular.z = speed if theta > 0 else -speed

        # loop a little faster than pose updates
        rate = rospy.Rate(12)

        while not rospy.is_shutdown() and abs(target_theta - self.theta) > error: 
            rospy.logdebug(self.theta)
            rospy.logdebug(target_theta)
            rospy.logdebug(abs(target_theta - self.theta))
            rospy.logdebug('\n')
            self.pub.publish(tw)
            rate.sleep()

        tw.angular.z = 0
        self.pub.publish(tw) 

    

