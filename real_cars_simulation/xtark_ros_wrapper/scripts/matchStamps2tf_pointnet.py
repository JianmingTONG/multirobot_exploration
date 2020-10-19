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
import torch
import torch.nn as nn
import PointNetVlad as PNV
import random

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

    def __init__(self, method, weightpath):
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
        self.feature_record = {}
        self.submap_locks = {}
        self.br = tf.TransformBroadcaster()

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # device = torch.device("cpu")
        self.model = PNV.PointNetVlad(global_feat=True, feature_transform=True, max_pool=False,
                                output_dim=512, num_points=4096)
        self.model = self.model.to(device)
        resume_filename = weightpath
        print("Resuming From ", resume_filename)
        checkpoint = torch.load(resume_filename)
        # checkpoint = torch.load(resume_filename,map_location=lambda storage, loc: storage)
        saved_state_dict = checkpoint['state_dict']
        self.model.load_state_dict(saved_state_dict)
        

    #回调函数输入的应该是msg
    def listcallback(self, data):
        FrameIDList = data.header.frame_id.split('/')
        current_robot_id = FrameIDList[1] if FrameIDList[0]=='' else FrameIDList[0]
        
        if not self.submap_list.has_key(current_robot_id):
            self.submap_list[current_robot_id] = SubmapList()
            self.feature_record[current_robot_id] = np.zeros((0,512),dtype=float)
            self.submap_locks[current_robot_id] = threading.Lock()
        
        self.submap_locks[current_robot_id].acquire()
        if len(self.submap_list[current_robot_id].submap)<len(data.submap) and len(data.submap)>2:
            newsub = len(data.submap) - max(len(self.submap_list[current_robot_id].submap),2)
            for robot_id in self.submap_list:
                if robot_id != current_robot_id:
                    for submap in data.submap[-newsub-2:-2]:
                        self.findMatches(submap, current_robot_id, robot_id)
        self.submap_list[current_robot_id] = data
        self.submap_locks[current_robot_id].release()
        
        print("update submap list")


    def findMatches(self, self_subamap, self_id, other_id):
        print("New matching")
        rospy.wait_for_service('/{}/get_occupancy_grid'.format(self_id))
        submap_client1 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(self_id), OccupancyGridQuery)
        submap_info1 = SubmapList()
        submap_info1.header.frame_id = '{}/map'.format(self_id)
        submap_info1.submap.append(self_subamap)
        submap_result1 = submap_client1(submap_info1)

        feature = self.map2feature(submap_result1.map, 4096)
        self.feature_record[self_id] = np.vstack(( self.feature_record[self_id] , np.array(feature) ) )
        if self.feature_record[other_id].shape[0]<1:
            return
        print("feature.shape:"+str(feature.shape) )
        print("self.feature_record[{}].shape:".format(self_id)+str(self.feature_record[self_id].shape) )
        print("self.feature_record[{}].shape:".format(other_id)+str(self.feature_record[other_id].shape) )
        results = np.matmul(self.feature_record[other_id], feature.T)
        print("results:"+str(results))
        validmatch = results>0.3
        if ( validmatch.any() ):
            rospy.wait_for_service('/{}/get_occupancy_grid'.format(other_id))
            submap_client2 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(other_id), OccupancyGridQuery)

            maxargs = np.argmax(results)
            submap_info2 = SubmapList()
            submap_info2.header.frame_id = '{}/map'.format(other_id)
            submap_info2.submap.append(self.submap_list[other_id].submap[maxargs])
            submap_result2 = submap_client2(submap_info2)

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
                return
            if tf_result.confidence < 0.5:
                return
            map2_to_map1_msg = tf_result.transform

            if self.method == "submap_pcl":
                pointcloud1 = self.OccupancyGrid2PointCloud(submap_result1.map)
                pub = rospy.Publisher("/{}/pointcloud_sub".format(self_id), PointCloud2, queue_size=1)
                pub.publish(pointcloud1)
                pointcloud2 = self.OccupancyGrid2PointCloud(submap_result2.map)
                pub = rospy.Publisher("/{}/pointcloud_sub".format(other_id), PointCloud2, queue_size=1)
                pub.publish(pointcloud2)
                print("icp_tf_client start")
                tf_result = self.pcl_client.call(pointcloud1, pointcloud2, map2_to_map1_msg)
                print(tf_result.transformation)
                map2_to_map1_msg = tf_result.transformation

            pub_tf = TransformStamped()
            pub_tf.header.stamp = rospy.Time.now()
            pub_tf.header.frame_id = '{}/map'.format(other_id)
            pub_tf.child_frame_id = '{}/map'.format(self_id)
            pub_tf.transform = map2_to_map1_msg
            self.br.sendTransformMessage(pub_tf)
            print(pub_tf)

    def map2feature(self, submap, input_dim):
        self.model.eval()
        scaled_grid = self.msg2scaled(submap, input_dim)
        with torch.no_grad():
            feature = np.array(self.model(torch.from_numpy(scaled_grid).float().view((-1, 1, input_dim, 2)).cuda()).cpu())
        return feature

    def msg2scaled(self, map_msg, NUM_POINTS):
        map = map_msg.data
        map = np.array(map)
        map[map==255] = 50
        map = map.reshape((map_msg.info.height, map_msg.info.width))
        points = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i][j] > 51 and map[i][j] < 255:
                    points.append(np.array([i*map_msg.info.resolution, j*map_msg.info.resolution]))
        if len(points) == 0:
            return np.zeros((NUM_POINTS, 2))
        points = np.array(points)
        # downsample
        output = np.zeros((NUM_POINTS, 2))
        if len(points) >= NUM_POINTS:
            idx = np.random.rand(len(points))
            idx = np.argsort(idx)
            output = points[idx[:NUM_POINTS]]
        else:
            idx = np.random.randint(0, len(points), NUM_POINTS - len(points))
            output = np.concatenate((points, points[idx]))
        # transform
        centroid = np.mean(output, axis=0) 
        d = np.sum(np.sqrt(np.sum(np.square(output-centroid),axis=1)))/len(output)
        s = 0.5/d
        T = np.array([[s, 0, -s*centroid[0]],[0, s, -s*centroid[1]],[0, 0, 1]])
        scaled_output = np.dot(T, np.transpose(np.concatenate((output, np.ones((len(output), 1))), axis=1)))
        scaled_output = np.transpose(scaled_output[:2])
        if np.min(scaled_output) > -1 and np.max(scaled_output) < 1:
            return scaled_output
        else:
            idx = (np.min(scaled_output,axis=1) < -1)+(np.max(scaled_output,axis=1) > 1)
            idx = np.where(idx==True)[0]
            for i in range(len(idx)):
                while True:
                    new_point = points[random.randint(0,len(points)-1)]
                    new_point=np.dot(T, np.transpose(np.concatenate((new_point, np.ones(1)), axis=0)))[:2]
                    if np.min(new_point) > -1 and np.max(new_point) < 1:
                        scaled_output[idx[i]] = new_point
                        break
            return scaled_output

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
    weightpath = ""
    opts, args = getopt.getopt(argv,"m:w:")
    for opt, arg in opts:
        if opt in ("-m"):
            method = arg
        if opt in ("-w"):
            weightpath = arg
    rospy.init_node('match_stamps_to_TF', anonymous=True)
    tfp = MultiRobotTF_Publisher(method, weightpath)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])