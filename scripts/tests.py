#!/usr/bin/env python
PKG = 'turtle_shell'


import sys
import unittest
from turtle_shell import wrap_radians 
import math

## A sample python unit test
class TestFunctions(unittest.TestCase):

    def test_wrap_radians(self):
    	self.assertAlmostEquals(wrap_radians(0), 0)
    	self.assertAlmostEquals(wrap_radians(math.pi), math.pi)
    	self.assertAlmostEquals(wrap_radians(math.pi+1), -math.pi+1)
    	self.assertAlmostEquals(wrap_radians(-math.pi-1), math.pi-1)



    # def test_radian_abs(self):
    # 	self.assertEquals(radian_abs(0,0), 0)
    # 	self.assertEquals(radian_abs(math.pi, math.pi), 0)
    # 	self.assertEquals(radian_abs(math.pi/2, -math.pi/2), math.pi)
    # 	self.assertEquals(radian_abs(-math.pi/2, math.pi/2), math.pi)



# -0.0121359736339
# 1.0827346816
# 5.18831465195



# 0.0227706114059
# 1.0827346816
# 1.05996407019


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_turtle_shell', TestFunctions)