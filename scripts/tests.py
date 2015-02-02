#!/usr/bin/env python
PKG = 'turtle_shell'


import sys
import unittest
from turtle_shell.shell import wrap_radians, radian_abs
import math

## A sample python unit test
class TestFunctions(unittest.TestCase):

 
    def test_wrap_radians(self):
    	self.assertEquals(wrap_radians(math.pi + 1), 1)
    	self.assertEquals(wrap_radians(-math.pi - 1), -1)
    	self.assertEquals(wrap_radians(math.pi/2), math.pi/2)
    	self.assertEquals(wrap_radians(-math.pi/2), -math.pi/2)

    def test_radian_abs(self):
    	self.assertEquals(radian_abs(0,0), 0)
    	self.assertEquals(radian_abs(math.pi, math.pi), 0)
    	self.assertEquals(radian_abs(math.pi/2, -math.pi/2), math.pi)
    	self.assertEquals(radian_abs(-math.pi/2, math.pi/2), math.pi)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_turtle_shell', TestFunctions)