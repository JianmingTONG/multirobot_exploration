#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

pub = None

def callback(data):
    global pub
    '''
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "robot1/map"
    pose.pose.position.x = data.pose.pose.position.x
    pose.pose.position.y = pose.y
    pose.pose.position.z = pose.z
      
    pose.pose.orientation.x = i
    pose.pose.orientation.y = 
    pose.pose.orientation.z = 
    pose.pose.orientation.w = 
    '''
    data.header.frame_id = "robot1/map"

    pub.publish(data)

def listener():
    global pub
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('robot1/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('pub/move_base_simple/goal', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()