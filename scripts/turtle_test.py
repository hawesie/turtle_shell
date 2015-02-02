#!/usr/bin/env python
import rospy


from turtle_shell.shell import TurtleShell

if __name__ == '__main__':
    try:
        ts = TurtleShell()    
        ts.rotate(-90)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass