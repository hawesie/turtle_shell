#!/usr/bin/env python
import rospy

from turtle_shell import turtlebot

if __name__ == '__main__':
    try:
  
        (x,y) = turtlebot.position()
        print(x, y)
        # turtlebot.rotate(90)        
        # turtlebot.goto(x + 2, y)
        turtlebot.goto(0, 0)
        (x,y) = turtlebot.position()
        print(x, y)

    except rospy.ROSInterruptException:
        pass