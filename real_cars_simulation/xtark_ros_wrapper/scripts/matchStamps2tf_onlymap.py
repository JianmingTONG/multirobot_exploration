#!/usr/bin/env python
#coding=utf-8
import rospy
import threading
import numpy as np
import struct
#导入自定义的数据类型
# from dslam_sp.msg import MatchStamp
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry
from cartographer_ros_msgs.srv import OccupancyGridQuery
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from multirobot_map_merge.srv import mapPair2tf
from icp_registration.srv import mapdata, Laserdata
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray, Transform
# import PyKDL as kdl
# from tf_conversions import posemath
import tf
import sys, getopt

def trans2pose(trans):
    pose = Pose()
    pose.orientation = trans.rotation
    pose.position = trans.translation
    return pose

def pose2trans(pose):
    trans = Transform()
    trans.rotation = pose.orientation
    trans.translation = pose.position
    return trans
class MultiRobotTF_Publisher:

    def __init__(self, method):
        assert (method == "submap_img" or method == "submap_pcl" or method == "laser_scan")
        self.method = method
        rospy.wait_for_service('GetMapTransform_submap_img')
        self.img_client = rospy.ServiceProxy('GetMapTransform_submap_img', mapPair2tf)
        if self.method == "submap_pcl":
            rospy.wait_for_service('GetMapTransform_submap_pcl')
            self.pcl_client = rospy.ServiceProxy('GetMapTransform_submap_pcl', mapdata)
        elif self.method == "laser_scan":
            rospy.wait_for_service('GetMapTransform_laser_scan')
            self.scan_client = rospy.ServiceProxy('GetMapTransform_laser_scan', Laserdata)
        self.pointcloud_pub = rospy.Publisher("pointcloud_sub", PointCloud2, queue_size=1)
        self.occupancy_grid_pub = rospy.Publisher("occupancy_grid_sub", OccupancyGrid, queue_size=1)
        #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
        self.list_sub = rospy.Subscriber("submap_list", SubmapList, self.listcallback)
        # self.match_sub = rospy.Subscriber("match_pair", MatchStamp, self.matchcallback)
        self.submap_list = {}
        self.submap_locks = {}
        self.br = tf.TransformBroadcaster()
        

    #回调函数输入的应该是msg
    def listcallback(self, data):
        
        print("update submap list " + str(len(data.submap)))
        FrameIDList = data.header.frame_id.split('/')
        current_robot_id = FrameIDList[1] if FrameIDList[0]=='' else FrameIDList[0]
        
        if not self.submap_list.has_key(current_robot_id):
            self.submap_list[current_robot_id] = SubmapList()
            self.submap_locks[current_robot_id] = threading.Lock()
        
        self.submap_locks[current_robot_id].acquire()
        if len(self.submap_list[current_robot_id].submap)<len(data.submap) and len(data.submap)>=2:
            for robot_id in self.submap_list:
                if robot_id != current_robot_id and len(self.submap_list[robot_id].submap)>=2:
                    self.findMatches(data.submap[-2], self.submap_list[robot_id], current_robot_id, robot_id)
                    if len(data.submap)>=3:
                        self.findMatches(data.submap[-3], self.submap_list[robot_id], current_robot_id, robot_id)
        self.submap_list[current_robot_id] = data
        self.submap_locks[current_robot_id].release()


    def findMatches(self, self_subamap, other_list, self_id, other_id):
        print("New matching")
        rospy.wait_for_service('/{}/get_occupancy_grid'.format(self_id))
        submap_client1 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(self_id), OccupancyGridQuery)
        submap_info1 = SubmapList()
        print("findMatches: 82")
        submap_info1.header.frame_id = '{}/map'.format(self_id)
        submap_info1.submap.append(self_subamap)
        submap_result1 = submap_client1(submap_info1)
        print("findMatches: 85")

        if self.method == "submap_pcl":
            pointcloud1 = self.OccupancyGrid2PointCloud(submap_result1.map)
            pub = rospy.Publisher("/{}/pointcloud_sub".format(self_id), PointCloud2, queue_size=1)
            pub.publish(pointcloud1)
            
        print("findMatches: 91")
        rospy.wait_for_service('/{}/get_occupancy_grid'.format(other_id))
        submap_client2 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(other_id), OccupancyGridQuery)
        map2_to_map1_tf_list = []
        confidence_list= []

        print("findMatches: 96")

        for submap_entry in other_list.submap[:-1]:
            submap_info2 = SubmapList()
            submap_info2.header.frame_id = '{}/map'.format(other_id)
            submap_info2.submap.append(submap_entry)
            print("findMatches: 102")
            submap_result2 = submap_client2(submap_info2)

            print("findMatches: 105")
            map2_to_map1_msg = Transform()
            print("img_tf_client start")
            try:
                tf_result = self.img_client.call(submap_result1.map, submap_result2.map)
                print("transform:" + str(tf_result.transform) + "confidence:" + str(tf_result.confidence))
            except Exception as e:
                outstr = "self_id: " + self_id + "\n others_id: " + other_id
                print(outstr)
                print("self_len: " + str(len(self.submap_list[self_id].submap ) ) )
                print("OTHER_len: " + str(len(self.submap_list[other_id].submap ) ) )
                print(e)
                continue
            if tf_result.confidence < 0.5:
                continue
            map2_to_map1_msg = tf_result.transform
            confidence_list.append(tf_result.confidence)

            if self.method == "submap_pcl":
                pointcloud2 = self.OccupancyGrid2PointCloud(submap_result2.map)
                pub = rospy.Publisher("/{}/pointcloud_sub".format(other_id), PointCloud2, queue_size=1)
                pub.publish(pointcloud2)
                print("icp_tf_client start")
                tf_result = self.pcl_client.call(pointcloud1, pointcloud2, map2_to_map1_msg)
                print(tf_result.transformation)
                map2_to_map1_msg = tf_result.transformation

            map2_to_map1_tf_list.append(map2_to_map1_msg)
            
        if(len(confidence_list)>0):
            pub_tf = TransformStamped()
            pub_tf.header.stamp = rospy.Time.now()
            pub_tf.header.frame_id = '{}/map'.format(other_id)
            pub_tf.child_frame_id = '{}/map'.format(self_id)
            pub_tf.transform = map2_to_map1_tf_list[confidence_list.index(max(confidence_list))]
            self.br.sendTransformMessage(pub_tf)
            #TODO
            #We should save the matched submaps to disk for visual checking
            #
            #TODO
            print(pub_tf)


    def find_submap_info(self, robotid, stamp):
        output_submap_info = SubmapEntry()
        output_submap_stamp = rospy.Time(0)
        
        self.keyFrame_locks[robotid].acquire()
        assert(len(self.submap_list[robotid].submap) == len(self.keyframe_list[robotid])), "submap_len:"+str(len(self.submap_list[robotid].submap))+" keyframe_len:"+str(len(self.keyframe_list[robotid]))
        for submap_info, keyframe_header in zip(self.submap_list[robotid].submap, self.keyframe_list[robotid]):
            if stamp>keyframe_header.stamp and (stamp-keyframe_header.stamp)<(stamp-output_submap_stamp):
                output_submap_info = submap_info
        self.keyFrame_locks[robotid].release()
        
        return output_submap_info

    def OccupancyGrid2PointCloud(self, occupancy_grid):
        map = occupancy_grid.data
        map = np.array(map)
        map[map==255] = 50
        max_grid = np.max(map)
        min_grid = np.min(map)
        map = map.reshape((occupancy_grid.info.height, occupancy_grid.info.width))
        
        buffer = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i,j]>((max_grid-min_grid)*0.67+min_grid) and map[i,j]>50:
                    z = 0
                    x = j * occupancy_grid.info.resolution + occupancy_grid.info.origin.position.x
                    y = i * occupancy_grid.info.resolution + occupancy_grid.info.origin.position.y
                    buffer.append(struct.pack('ffff', x, y, z, 1))

        pcl_msg = PointCloud2()

        pcl_msg.header = occupancy_grid.header
        print( pcl_msg.header )

        pcl_msg.height = 1
        pcl_msg.width = len(buffer)

        pcl_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1))

        pcl_msg.is_bigendian = False
        pcl_msg.point_step = 16
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width * pcl_msg.height
        pcl_msg.is_dense = False
        pcl_msg.data = "".join(buffer)

        return pcl_msg

def main(argv):
    method = "submap_img"
    opts, args = getopt.getopt(argv,"m:")
    for opt, arg in opts:
        if opt in ("-m"):
            method = arg
    rospy.init_node('match_stamps_to_TF', anonymous=True)
    tfp = MultiRobotTF_Publisher(method)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])