#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray, PointStampedArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

########################### added by jianming
# frontiersPointStamp = PointStampedArray()
# def callBackPointStamp(data):
# 	global frontiersPointStamp
# 	for pointstamp in data.pointstamps:
# 		frontiersPointStamp.pointstamps.append(pointstamp)
###########################

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
	
	########################### added by jianming
	# rospy.Subscriber('filtered_points_stamp_list', PointStampedArray, callBackPointStamp)
	###########################
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count) ) )
	elif len(namespace)==0:
			robots.append(robot(namespace))
	# for i in range(0,n_robots):
	# 	robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		rospy.loginfo("### -------------------new loop -------------------- ###")
		centroids=copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,n_robots):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		# rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]

		for ir in na:
			for ip in range(0, len(centroids)):
				cost = norm(robots[ir].getPosition() - centroids[ip])		
				rospy.loginfo(robots[ir].getPosition())
				threshold = 1
				information_gain = infoGain[ip]
				if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
					information_gain *= hysteresis_gain
				revenue = information_gain * info_multiplier - cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)

		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain

					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)

		# rospy.loginfo("revenue record: "+str(revenue_record))	
		# rospy.loginfo("centroid record: "+str(centroid_record))	
		# rospy.loginfo("robot IDs record: "+str(id_record))	

#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			########################### original 
			# robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			########################### modified version 1 using the pointstamps
			# robots[id_record[winner_id]].sendGoalPointStamp(frontiersPointStamp.pointstamps[winner_id])
			########################### modified version 2 using transform here.
			targetFrameId = []
			for i in range(0,n_robots):
				targetFrameId.append(namespace + str(i+1) +'/map')
			pointstamp = PointStamped()
			pointstamp.header.frame_id = mapData.header.frame_id
			pointstamp.point.x = centroid_record[winner_id][0]
			pointstamp.point.y = centroid_record[winner_id][1]
			tfLisn = tf.TransformListener()
			targetFrame = targetFrameId[id_record[winner_id]]
			tfLisn.waitForTransform(mapData.header.frame_id, targetFrame, rospy.Time(0), rospy.Duration(10.0))
			transformedPointStamp = tfLisn.transformPoint(targetFrame, pointstamp)
			transformedPoint = []
			transformedPoint.append(transformedPointStamp.point.x)
			transformedPoint.append(transformedPointStamp.point.y)
			# rospy.loginfo("send goal point %d, %d"%(transformedPointStamp.point.x, transformedPointStamp.point.y))
			robots[id_record[winner_id]].sendGoal(transformedPoint)
			###########################
			rospy.loginfo("in frame=" + pointstamp.header.frame_id + namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rospy.loginfo("in frame=" + transformedPointStamp.header.frame_id + namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to (changed) "+str(transformedPoint))
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
		rospy.loginfo("\n\n\n")
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
