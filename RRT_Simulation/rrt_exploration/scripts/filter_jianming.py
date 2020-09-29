#!/usr/bin/env python
 

# --------Include modules---------------
import rospy
import tf
from numpy import array, vstack, delete, floor, linalg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped
from rrt_exploration.msg import PointArray, PointStampedArray
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import MeanShift
from functions import gridValue
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



merged_map     = OccupancyGrid()
merged_costmap = OccupancyGrid()
frontiers = []

def mapCallBack(msg):
    global merged_map
    merged_map = msg


def costmapCallBack(msg):
    global merged_costmap
    merged_costmap = msg


def detectFrontiersCallBack(msg, args):
    global frontiers
    transformedPoint = args[0].transformPoint(args[1], msg)
    tempList = [transformedPoint.point.x, transformedPoint.point.y]
    frontiers.append(tempList)
    # rospy.loginfo("receive %d numeber of points"%(len(frontiers)))

def node():
    global merged_costmap, merged_map, frontiers

    rospy.init_node("filter_jianming", anonymous=False)
    
    # ---------------------- get params from ros server
    map_topic      = rospy.get_param("~map_topic",'map_merge/map')
    costmap_topic  = rospy.get_param("~costmap_topic",'map_merge/costmap')
    frontier_topic = rospy.get_param("~frontier_topic", "detected_points")
    n_robots       = rospy.get_param("~n_robots", 1)
    rateHz         = rospy.get_param("~rateHz", 1)
    namespace      = rospy.get_param("~namespace", "robot")
    robot_frame    = rospy.get_param("~robot_frame","base_link")
    info_radius    = rospy.get_param("~info_radius", 1.0)
    costmap_pixel_threshold = rospy.get_param("~costmap_pixel_threshold", 40) # goals location with costmap data larger than threshold will be deleted

    # ---------------------- configure the nodes' params
    rate = rospy.Rate(rateHz)

    # ---------------------- subscribe to the map topics
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallBack)
    
    # ---------------------- publish topics for robots assigner.
    dataFilteredPointPub  = rospy.Publisher("filtered_points", PointArray, queue_size=10)

    # ---------------------- publish topics for display
    # displayFrontiersPub = rospy.Publisher("frontiers", Marker, queue_size=10)
    displayCentroidsPub = rospy.Publisher("centroids", Marker, queue_size=10)

    # ---------------------- receive the topics from ros server.
    while(len(merged_map.data)==0 ):
        rospy.loginfo("waiting for the merged map")
        continue

    while(len(merged_costmap.data)==0):
        rospy.loginfo("waiting for the merged costmap")
        continue

    # ---------------------- initialize display Markers' params.
    display_centroids = Marker()

    display_centroids.type     = Marker.POINTS
    display_centroids.header   = merged_map.header
    display_centroids.action   = Marker.ADD 
    display_centroids.ns       = "points"
    display_centroids.pose.orientation.w = 1
    display_centroids.scale.x  = 0.2
    display_centroids.scale.y  = 0.2
    display_centroids.color.r  = 0   # 255/255
    display_centroids.color.g  = 1   # 255/255
    display_centroids.color.b  = 0   # 0  /255
    display_centroids.color.a  = 1
    display_centroids.lifetime = rospy.Duration()

    # ---------------------- initialize transform.
    merge_map_frame = merged_map.header.frame_id
    tfLisn = tf.TransformListener()
    # robotsTransform = []
    if n_robots > 0:
        for i in range(0, n_robots):
            tfLisn.waitForTransform(merge_map_frame, namespace+str(i+1)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
            # (trans, rot) = tfLisn.lookupTransform(namespace+str(i+1)+'/'+robot_frame, merge_map_frame, rospy.Time(0))
            # robotsTransform.append(copy(trans))
    elif n_robots == 1:
        tfLisn.waitForTransform(merge_map_frame, 'robot1/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
        # (trans, rot) = tfLisn.lookupTransform('robot1/'+robot_frame, merge_map_frame, rospy.Time(0))
        # robotsTransform.append(copy(trans))

    # ---------------------- subscribe to the detected points.
    rospy.Subscriber(frontier_topic, PointStamped,  callback=detectFrontiersCallBack, callback_args=[tfLisn, merge_map_frame] )

    # ---------------------- initialize the temporary vars.
    tempPointStamp                 = PointStamped()
    tempPointStamp.header.frame_id = merged_map.header.frame_id
    tempPointStamp.header.stamp    = rospy.Time(0)
    tempPointStamp.point.z         = 0.0
    
    tempPointArray = PointArray()

    #-------------------------------------------------------------------------
    #---------------------     Main   Loop     -------------------------------
    #-------------------------------------------------------------------------
    while not rospy.is_shutdown():
        if(len(frontiers) == 0):
            rate.sleep()
            continue
        
        # ---------------------- cluster the frontiers into several centers.
        if(len(frontiers)>1):
            # frontiers_copy = frontiers
            ms = MeanShift(bandwidth=0.6) # bandwidth determines the cluster radius
            ms.fit(frontiers)
            centroids = ms.cluster_centers_
        elif(len(frontiers)==1):
            centroids = frontiers
        # rospy.loginfo("detect %d points"%len(centroids))

        # ---------------------- display detected centroids.
        # for item in centroids:
            # rospy.loginfo("centroids after cluster: location %3.2f, %3.2f"%(item[0], item[1]))

        # ---------------------- remove centroids whose distances to obstacle < radius of robots.
        # ---------------------- remove old centroids which have been explored. 
        index = 0
        # rospy.loginfo("filter.py start transform loop: %s"%rospy.get_rostime())
        while index < len(centroids):
            tempPointStamp.point.x = centroids[index][0]
            tempPointStamp.point.y = centroids[index][1]
            tempTransformedPoint = tfLisn.transformPoint(merged_costmap.header.frame_id, tempPointStamp)
            # avoid robots hit the walls
            index = (floor((tempTransformedPoint.point.y-merged_costmap.info.origin.position.y)/merged_costmap.info.resolution)*merged_costmap.info.width) + (floor((tempTransformedPoint.point.x-merged_costmap.info.origin.position.x)/merged_costmap.info.resolution))
            condition = merged_costmap.data[int(index)] > costmap_pixel_threshold   
            # delete old frontiers which have been explored
            condition = (informationRectangleGain(merged_map, [tempPointStamp.point.x, tempPointStamp.point.y], info_radius) < 0.5) or condition
            if(condition):
                centroids = delete(centroids, index, axis=0)
                index = index - 1
            index+=1
        rospy.loginfo("remain %d points after delete"%len(centroids))
        # rospy.loginfo("filter.py after transform loop: %s"%rospy.get_rostime())

        # ---------------------- publish the filtered centroids for following data processing & display
        tempPointArray.points = []
        display_centroids.points = []
        for item in centroids:
            tempPoint   = Point()
            tempPoint.z = 0.0 
            tempPoint.x = item[0]
            tempPoint.y = item[1]
            tempPointArray.points.append(tempPoint)
            display_centroids.points.append(tempPoint)
            
        dataFilteredPointPub.publish(tempPointArray)
        displayCentroidsPub.publish(display_centroids)

        # rospy.loginfo("time after loop: %s"%rospy.get_rostime())
        frontiers = []
        rate.sleep()
        

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass