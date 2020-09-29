#!/usr/bin/env python

import actionlib
from geometry_msgs.msg import PointStamped
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def goalCallBack(msg, args):
    goal = MoveBaseGoal()
    goal.target_pose.header = msg.header
    goal.target_pose.pose.position.x = msg.point.x
    goal.target_pose.pose.position.y = msg.point.y 
    goal.target_pose.pose.orientation.w = 1.0
    args.send_goal(goal)

def node():
    rospy.init_node('robot_driver', anonymous=False)
    robot_id  = rospy.get_param('~robot_id','1')
    goal_topic  = rospy.get_param('~goal_topic','robot1/rrt_goal')
    # actionlib.SimpleActionClient('robot'+ str(i+1) +'/move_base', MoveBaseAction)
    ac = actionlib.SimpleActionClient('robot'+ str(robot_id) +'/move_base', MoveBaseAction)
    rospy.Subscriber(goal_topic, PointStamped, callback=goalCallBack, callback_args=ac)
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 