import rospy


from geometry_msgs.msg import Twist, Pose
from math import radians, fmod, pi, degrees, cos, sin

def wrap_radians(rads):
    if rads > pi:
        return -pi + fmod(rads, pi)
    elif rads < -pi:
        return pi + fmod(rads, -pi)
    else:
        return rads

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

    def right(self, theta):
        self.rotate(-theta)

    def left(self, theta):
        self.rotate(theta)

    def rotate(self, theta):

        self._wait_for_first_pose()

        # assuming input in degrees, so convert to radians
        theta = radians(theta)
    
        # how far the robot can turn in a second
        # needs to be checked for individual robots
        speed = radians(20)

        # acceptable target rotation error
        error = radians(5)

        # target 
        target_theta = wrap_radians(self.theta + theta)

        tw = Twist()
        tw.angular.z = speed if theta > 0 else -speed

        rate = rospy.Rate(12)

        while not rospy.is_shutdown() and abs(target_theta - self.theta) > error: 
            # rospy.loginfo(self.theta)
            # rospy.loginfo(target_theta)
            # rospy.loginfo(abs(target_theta - self.theta))
            # rospy.loginfo('\n')
            self.pub.publish(tw)
            rate.sleep()

        # stop the robot
        tw.angular.z = 0
        count = 0
        while not rospy.is_shutdown() and count < 10: 
            self.pub.publish(tw)
            rate.sleep()
            count += 1

    def forward(self, distance):
        self.travel(distance)

    def backwards(self, distance):
        self.travel(-distance)

    def travel(self, distance):
        self._wait_for_first_pose()

        target_x = self.x + distance * cos(radians(self.theta))
        target_y = self.y + distance * sin(radians(self.theta))

        # rospy.loginfo('%s, %s' % (self.x, self.y))
        # rospy.loginfo('%s, %s' % (target_x, target_y))
        # rospy.loginfo('%s, %s' % (abs(target_x - self.x), abs(target_y - self.y)))

        # how far the robot can move in a second in m
        # needs to be checked for individual robots
        speed = 1.0

        # acceptable target rotation error
        error_x = 0.10
        error_y = 0.10

        tw = Twist()
        tw.linear.x = speed if distance > 0 else -speed

        rate = rospy.Rate(12)

        while not rospy.is_shutdown() and (abs(target_x - self.x) > error_x or abs(target_y - self.y) > error_y):             
            # rospy.loginfo('%s, %s' % (abs(target_x - self.x), abs(target_y - self.y)))
            self.pub.publish(tw)
            rate.sleep()

        # stop the robot
        tw.linear.x = 0
        count = 0
        while not rospy.is_shutdown() and count < 10: 
            self.pub.publish(tw)
            rate.sleep()
            count += 1

