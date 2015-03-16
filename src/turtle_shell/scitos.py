import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from turtle_shell import TurtleShell, wrap_radians


class ScitosShell(TurtleShell):
    """Turtle shell for a Scito"""
    def __init__(self):
        super(ScitosShell, self).__init__('cmd_vel')
        rospy.Subscriber('odom', Odometry, self.callback, queue_size=5)        

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])        
        self.theta = wrap_radians(y)
        # print(self.x, self.y, self.theta)


impl = None

def get_impl():
    global impl
    if impl is None:
        impl = ScitosShell()
    return impl

def rotate(theta):           
    get_impl().rotate(theta)

def pos():
    return position()

def position():
    return get_impl().position()

def heading(): 
    return get_impl().heading()

