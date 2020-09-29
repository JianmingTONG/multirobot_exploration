#!/usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rrt_exploration.msg import PointArray
from functions import robot
import numpy as np
from numpy import array, vstack, delete, floor, linalg
from copy import copy

def informationRectangleGain(mapData, point, r):
    infoGain = 0
    index = (floor((point[1]-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((point[0]-mapData.info.origin.position.x)/mapData.info.resolution))
                  
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    last_index = index+r_region*(mapData.info.width+1)
    length     = 2*r_region
    start      = int(init_index)
    end        = start + int(length)
    if (last_index < len(mapData.data)):
        for n in range(0, 2*r_region+1):
            for i in range(start, end+1):
                if(mapData.data[i] == -1):
                    infoGain += 1
                elif(mapData.data[i] == 100):
                    infoGain -= 1
            start += mapData.info.width
            end   += mapData.info.width
    else:
        for n in range(0, 2*r_region+1):
            for i in range(start, end+1):
                limit = ((start/mapData.info.width) + 2)*mapData.info.width  # part of rectangle is outside the map
                if (i >= 0 and i < limit and  i < len(mapData.data)):
                    if(mapData.data[i] == -1):
                        infoGain += 1
                    elif(mapData.data[i] == 100):
                        infoGain -= 1
            start += mapData.info.width
            end   +=mapData.info.width

    return infoGain*(mapData.info.resolution**2)

mapData        = OccupancyGrid()
merged_costmap = OccupancyGrid()
centroids = []

def mapCallBack(msg):
    global mapData
    mapData=msg

def costmapCallBack(msg):
    global merged_costmap
    merged_costmap = msg

def filtedCentroidBack(msg):
	global centroids
	centroids=[]
	for point in msg.points:
		centroids.append(array([point.x,point.y]))

def node():
    global mapData, centroids, merged_costmap
    robots=[]
    robotsFrameId=[]
    rospy.init_node('assigner', anonymous=False)
    
    # ---------------------- get params from ros server
    map_topic            = rospy.get_param('~map_topic','/map')
    costmap_topic  = rospy.get_param("~costmap_topic",'map_merge/costmap')
    centroids_topic      = rospy.get_param('~frontiers_topic','filtered_points')
    # info_radius= rospy.get_param('~info_radius',1.0)	
    n_robots             = rospy.get_param('~n_robots',1)
    namespace            = rospy.get_param('~namespace','')
    namespace_init_count = rospy.get_param('namespace_init_count',1)
    mini_step            = rospy.get_param("~mini_step",0.3)
    rateHz               = rospy.get_param('~rate',100)
    info_radius          = rospy.get_param('~info_radius',1.0)

    rospy.loginfo("namespace is %s"%namespace)
    
    # ---------------------- configure the nodes' params
    rate   = rospy.Rate(rateHz)
    tfLisn = tf.TransformListener()
    
    for i in range(0,n_robots):
        robotsFrameId.append(namespace + str(i+1) +'/map')
        rospy.loginfo('robot target frame %s'%(namespace + str(i+1) +'/map'))
    
    # ---------------------- subscribe to the map topics
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(centroids_topic, PointArray, filtedCentroidBack)
    rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallBack)

    # ---------------------- receive the topics from ros server.
    while (len(mapData.data)<1):
        pass

    while(len(merged_costmap.data)==0):
        rospy.loginfo("waiting for the merged costmap")
        continue

    #----------------------- create robot objects
    if len(namespace)>0:
        for i in range(0,n_robots):
            # rospy.loginfo("robot %d name is %s"%(i+1, namespace+str(i+namespace_init_count)))
            robots.append(robot(namespace+str(i+namespace_init_count) ) )
    elif len(namespace)==0:
        robots.append(robot(namespace))
    pointstamp = PointStamped()
    pointstamp.header.frame_id = mapData.header.frame_id
    rospy.loginfo('robot source frame %s'%(mapData.header.frame_id))
    
    #-------------------------------------------------------------------------
    #---------------------     Main   Loop     -------------------------------
    #-------------------------------------------------------------------------
    while not rospy.is_shutdown():
        revenue_record     = []
        centroid_record = []
        id_record       = []
        if(len(centroids) == 0):
            rate.sleep()
            continue

        #----------------------- copy the centroids to avoid bugs because of centroids change in the loop (callback thread will change centroids)  
        centroids_process = copy(centroids)
        # rospy.loginfo("assginer loop start: %s"%rospy.get_rostime())  #############################
        
        #----------------------- get number of available/busy robots    
        na=[] #available robots
        nb=[] #busy robots
        for i in range(0,n_robots):
            if (robots[i].getState()==actionlib.SimpleGoalState.ACTIVE):

                #----------------------- whether the current goal is in obstacle    
                tempTransformedPoint = tfLisn.transformPoint(merged_costmap.header.frame_id, robots[i].assigned_point_merged_map_frame)
                index_costmap = (floor((tempTransformedPoint.point.y-merged_costmap.info.origin.position.y)/merged_costmap.info.resolution)*merged_costmap.info.width) + (floor((tempTransformedPoint.point.x-merged_costmap.info.origin.position.x)/merged_costmap.info.resolution))
                
                #----------------------- whether the current goal has been already explored
                tempTransformedPoint_map = tfLisn.transformPoint(mapData.header.frame_id, robots[i].assigned_point_merged_map_frame)
 
                if(merged_costmap.data[int(index_costmap)] > 0 or informationRectangleGain(mapData, [tempTransformedPoint_map.point.x, tempTransformedPoint_map.point.y], 0.5) ):
                    na.append(i)
                    rospy.loginfo("reset robot %d's goal "%i)
                else:
                    nb.append(i)
            else:
                na.append(i)
        # rospy.loginfo("available robots: "+str(na))
        
        if(len(na) == 0):
            rate.sleep()
            continue

        rospy.loginfo("### -------------------new loop -------------------- ###")
        
        #----------------------- display the goals whose distance to robot < 0.5
        for item in centroids_process:
            rospy.loginfo("assigner.py receive centroids: location %3.2f, %3.2f"%(item[0], item[1]))
        
        #----------------------- calculate the cost of robots with respect to the goal
        ip = 0
        for ir in na:
            while ip < len(centroids_process):
                if(linalg.norm(robots[ir].getPosition() - centroids_process[ip]) < mini_step):
                    centroids_process = delete(centroids_process, (ip), axis=0)
                    ip = ip-1
                ip += 1
        
        for pointIdx in range(0, len(centroids_process)):
            infoGain = 3*informationRectangleGain(mapData, centroids_process[pointIdx], info_radius)
            for robotIdx in na:    
                # rospy.loginfo("robot %d: infogain = %3.2f"%(robotIdx, infoGain))
                dis = linalg.norm(robots[robotIdx].getPosition() - centroids_process[pointIdx])
                # rospy.loginfo("robot %d: cost = %3.2f"%(robotIdx, dis))
                revenue_record.append(infoGain - dis)
                centroid_record.append(centroids_process[pointIdx])
                id_record.append(robotIdx)

        winner_id = 0
        if(len(revenue_record) == 0):
            rate.sleep()
            continue
        winner_id=revenue_record.index(max(revenue_record))
        pointstamp.point.x = centroid_record[winner_id][0]
        pointstamp.point.y = centroid_record[winner_id][1]
        targetFrameId      = robotsFrameId[id_record[winner_id]]

        tfLisn.waitForTransform(mapData.header.frame_id, targetFrameId, rospy.Time(0), rospy.Duration(10.0))
        transformedPointStamp = tfLisn.transformPoint(targetFrameId, pointstamp)
        transformedPoint = []
        transformedPoint.append(transformedPointStamp.point.x)
        transformedPoint.append(transformedPointStamp.point.y)
        robots[id_record[winner_id]].assigned_point_merged_map_frame = pointstamp
        robots[id_record[winner_id]].sendGoal(transformedPoint)
        rospy.loginfo("send goal point %3.2f, %3.2f"%(transformedPointStamp.point.x, transformedPointStamp.point.y))
        centroids = []
        # rospy.loginfo("assginer loop end: %s"%rospy.get_rostime())  #############################
        rate.sleep()
        rospy.loginfo("\n\n\n")


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 