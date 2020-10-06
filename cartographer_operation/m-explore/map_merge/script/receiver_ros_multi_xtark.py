#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import torch
import torch.nn as nn
import torchvision
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.models as models

from std_msgs.msg import String

import numpy as np
from PIL import Image
import os
import rospy
from geometry_msgs.msg import TransformStamped
from dslam_sp.msg import image_depth, PRrepresentor, TransformStampedArray, MatchStamp
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import sys, getopt 

meta = {'outputdim': 2048}


self_ID = 'robot1'
# frame 规则是 self_ID*1e8 + framecount

representors_record = {}
representors_locks = {}
representor_stamps = {} #没加锁 todo

frameIDstr = []
match_loop_pub = None
other_match_loop_pub = None

matched_stamps = {} #展示和第i号机器人是否已经匹配，并且记录自己的第几帧和对方匹配。

looptrans_pub = None 

def matchrepresentor(query, matchs):
    results = np.matmul(matchs, query)
    return results

def loop_callback(data):
    #
    global representors_record, representors_locks, match_loop_pub, matched_stamps
    imageHeader = data.imageHeader
    imageFrameID = imageHeader.frame_id

    FrameIDList = imageFrameID.split('/')
    current_robot_id = FrameIDList[1] if FrameIDList[0]=='' else FrameIDList[0]
    # assert(current_robot_id == self_ID)
    
    representor = data.representor
    #更新记录
    if not (representors_locks.has_key(current_robot_id) ):
        representors_locks[current_robot_id] = threading.Lock()
        representors_record[current_robot_id] = np.zeros((0,meta['outputdim']),dtype=float)
        representor_stamps[current_robot_id] = []
        matched_stamps[current_robot_id] = set()
    representors_locks[current_robot_id].acquire()
    if len(representors_record[current_robot_id]) == 0:
        representors_record[current_robot_id] = np.vstack(( representors_record[current_robot_id] , np.array(representor) ) )
    else:
        last_keyframe_rep = representors_record[current_robot_id][-1]
        print("np.matmul(representor, last_keyframe_rep)")
        print(np.matmul(representor, last_keyframe_rep))
        print("np.matmul")
        if np.matmul(representor, last_keyframe_rep) < 0.95:
            representors_record[current_robot_id] = np.vstack(( representors_record[current_robot_id] , np.array(representor) ) )
        else:
            representors_locks[current_robot_id].release()
            return
    representors_locks[current_robot_id].release()
    
    #记录对应帧ID
    representor_stamps[current_robot_id].append(imageHeader.stamp)
    # print(representor_stamps)

    #处理match
    #和之前记录的值进行比较
    matchresults = {}
    if (current_robot_id == self_ID): #如果是自己的图片需要和所有图片进行比较
        for robot_id, representor_id_record in representors_record.items():
            representors_locks[robot_id].acquire()
            matchresults[robot_id] = matchrepresentor(representor,representor_id_record )
            representors_locks[robot_id].release()

        for robot_id, representor_id_record in matchresults.items():
            if robot_id == self_ID : #自己和自己比
                continue
                # if len(matchresults[robot_id] > 10):
                #     validresults = matchresults[robot_id][0:-10]
                #     validmatch = validresults>0.94
                #     if ( validmatch.any() ):
                #         maxargs = np.argmax(validresults)
                        
                #         matchpair = MatchStamp()
                #         matchpair.robotid1 = current_robot_id
                #         matchpair.robotid2 = robot_id
                #         matchpair.stamp1 = imageHeader.stamp
                #         matchpair.stamp2 = representor_stamps[robot_id][maxargs]
                #         print ("matchpair: " + str(matchpair))
                #         match_loop_pub.publish(matchpair) # to publish rel fnames  
            else : #新进来的帧和之前别人的图片能够匹配
                if len(matchresults[robot_id] > 1):
                    validresults = matchresults[robot_id][0:-1]
                    validmatch = validresults>0.94
                    if ( validmatch.any() ):
                        maxargs = np.argmax(validresults)
                        print(imageFrameID)
                        if not ( imageHeader.stamp in matched_stamps[robot_id]):# to do: 改为范围判断
                            matchpair = MatchStamp()
                            matchpair.robotid1 = current_robot_id
                            matchpair.robotid2 = robot_id
                            matchpair.stamp1 = imageHeader.stamp
                            matchpair.stamp2 = representor_stamps[robot_id][maxargs]
                            print ("matchpair: " + str(matchpair))
                            match_loop_pub.publish(matchpair) # to publish rel fnames  
                            matched_stamps[robot_id] = matched_stamps[robot_id].union(set([imageHeader.stamp]))
    else : # 不是自己的图片，就只和自己进行比较
        representors_locks[self_ID].acquire()
        matchresults[self_ID] = matchrepresentor(representor,representors_record[self_ID] )
        representors_locks[self_ID].release()
        if len(matchresults[self_ID] > 1):
            validresults = matchresults[self_ID][0:-1]
            validmatch = validresults>0.94
            if ( validmatch.any() ):
                maxargs = np.argmax(validresults)
                print(representor_stamps[self_ID][maxargs])
                if not ( representor_stamps[self_ID][maxargs] in matched_stamps[current_robot_id]):
                    matchpair = MatchStamp()
                    matchpair.robotid1 = self_ID
                    matchpair.robotid2 = current_robot_id
                    matchpair.stamp1 = representor_stamps[self_ID][maxargs]
                    matchpair.stamp2 = imageHeader.stamp
                    print ("matchpair: " + str(matchpair))
                    match_loop_pub.publish(matchpair) # to publish rel fnames  
                    matched_stamps[current_robot_id] = matched_stamps[str(current_robot_id)].union(set([representor_stamps[self_ID][maxargs]]))
    print("matchresults" + str(matchresults))
    print("matched_stamps" + str(matched_stamps))


def main(argv):
    global looptrans_pub, match_loop_pub, self_ID, representors_record, representors_locks, other_match_loop_pub, matched_stamps
    opts, args = getopt.getopt(argv,"i:")
    for opt, arg in opts:
        if opt in ("-i"):
            self_ID = arg

    representors_record[self_ID] = np.zeros((0,meta['outputdim']),dtype=float)
    representors_locks[self_ID] = threading.Lock()
    representor_stamps[self_ID] = []
    matched_stamps[self_ID] = set()
    
    rospy.init_node('matcher', anonymous=True)
    rospy.Subscriber("PRrepresentor", PRrepresentor, loop_callback)
    # rospy.Subscriber("OtherPresentor", PRrepresentor, loop_callback)
    looptrans_pub = rospy.Publisher("looppose_deee",TransformStamped, queue_size=3)
    match_loop_pub = rospy.Publisher("loopstamps",MatchStamp, queue_size=3)
    rospy.spin()



if __name__ == '__main__':
    main(sys.argv[1:])