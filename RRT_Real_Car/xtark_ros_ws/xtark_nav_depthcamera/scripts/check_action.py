#!/usr/bin/env python
#coding=utf-8
import rospy
from pyc import Check_action

if __name__ == '__main__':
  rospy.init_node('check_server')
  server = Check_action.CheckServer()
  rospy.spin()
