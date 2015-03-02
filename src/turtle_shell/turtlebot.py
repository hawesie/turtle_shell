import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from turtle_shell import TurtleShell, wrap_radians


class TurtlebotShell(TurtleShell):
    """Turtle shell for the Turtlebot"""
    def __init__(self):
        super(TurtlebotShell, self).__init__('cmd_vel_mux/input/navi')
        rospy.Subscriber('robot_pose', Pose, self.callback, queue_size=1)        
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mb_client.wait_for_server()

    def callback(self, data):
        self.x = data.position.x
        self.y = data.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])        
        self.theta = wrap_radians(y)
        # print(self.x, self.y, self.theta)

    def goto(self, x, y, heading = 0):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        orientation = tf.transformations.quaternion_from_euler(0, 0, heading)
        goal.target_pose.pose.orientation = Quaternion(*orientation)
        goal.target_pose.header.frame_id = 'map'        
        self.mb_client.send_goal_and_wait(goal)


impl = None

def get_impl():
    global impl
    if impl is None:
        impl = TurtlebotShell()
    return impl

def rotate(theta):           
    get_impl().rotate(theta)

def pos():
    return position()

def position():
    return get_impl().position()

def heading(): 
    return get_impl().heading()

def goto(x, y): 
    return get_impl().goto(x, y)