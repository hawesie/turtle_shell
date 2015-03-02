#!/usr/bin/env python

from turtle_shell import turtlebot

(x,y) = turtlebot.position()
print(x, y)
# turtlebot.rotate(90)        
turtlebot.goto(x + 2, y)
# turtlebot.goto(0, 0)
(x,y) = turtlebot.position()
print(x, y)

