#!/usr/bin/env python
import rospy
import os,time
import sys,traceback
import roslib
from math import pi as PI, degrees,radians,sin,cos
from threading import Thread 
from geometry_msgs.msg import Twist,Quaternion,Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16,Int32,Float32
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
from std_srvs.srv import Trigger,TriggerResponse
import XMiddleWare as xmw
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dynamic_reconfigure.server import Server
from xtark_ros_wrapper.cfg import PID_reconfigConfig


ODOM_POSE_COVARIANCE = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0, 
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-3]

ODOM_TWIST_COVARIANCE = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

IMU_ORIENTATION_COVARIANCE = [1e6,0,0,
                                0,1e6,0,
                                0,0,1e1]
        
IMU_ANGULAR_VELOCITY_COVARIANCE = [1e6,0,0,
                                    0,1e6,0,
                                    0,0,1e-1]
IMU_LINEAR_ACCELERATION_COVARIANCE = [-1,0,0,
                                        0,0,0,
                                        0,0,0]

class XMIDDLEWARE:
    def __init__(self):

        rospy.init_node('XMiddleWare',log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
 
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,self.handle_cmd,queue_size=1)
        self.odom_pub    = rospy.Publisher('odom',Odometry,queue_size=5)
        self.battery_pub = rospy.Publisher('voltage',Float32,queue_size=5)
        self.imu_pub     = rospy.Publisher('imu',Imu,queue_size=5)
        self.avel_pub    = rospy.Publisher('xtark/avel',Int32,queue_size=5)
        self.aset_pub    = rospy.Publisher('xtark/aset',Int32,queue_size=5)
        self.bvel_pub    = rospy.Publisher('xtark/bvel',Int32,queue_size=5)
        self.bset_pub    = rospy.Publisher('xtark/bset',Int32,queue_size=5)
        self.cvel_pub    = rospy.Publisher('xtark/cvel',Int32,queue_size=5)
        self.cset_pub    = rospy.Publisher('xtark/cset',Int32,queue_size=5)
        self.dvel_pub    = rospy.Publisher('xtark/dvel',Int32,queue_size=5)
        self.dset_pub    = rospy.Publisher('xtark/dset',Int32,queue_size=5)

        self.odom_broadcaster = TransformBroadcaster()

        self.port_name                 = rospy.get_param('~port_name',"/dev/ttyAMA0")
        self.baud_rate                 = rospy.get_param('~baud_rate',115200)
        self.odom_frame                = rospy.get_param('~odom_frame',"odom")
        self.base_frame                = rospy.get_param('~base_frame',"base_footprint")
        self.imu_frame                 = rospy.get_param('~imu_frame',"base_imu_link")
        self.control_rate              = rospy.get_param('~control_rate',50)
        self.publish_odom_transform    = rospy.get_param('~publish_odom_transform',True)
        self.is_omni                   = rospy.get_param('~is_omni',True)
        self.Kp                        = rospy.get_param('~Kp',300)
        self.Ki                        = rospy.get_param('~Ki',0)
        self.Kd                        = rospy.get_param('~Kd',200)
        self.encoder_resolution        = rospy.get_param('~encoder_resolution',1600)
        self.wheel_diameter            = rospy.get_param('~wheel_diameter',0.15)
        self.wheel_track               = rospy.get_param('~wheel_track',0.3)
        self.ax_cm_k                   = rospy.get_param('~ax_cm_k',0.08)
        self.linear_correction_factor  = rospy.get_param('~linear_correction_factor',1.0)
        self.angular_correction_factor = rospy.get_param('~angular_correction_factor',1.0)
        self.robot_linear_acc          = rospy.get_param('~robot_linear_acc',1.5)
        self.robot_angular_acc         = rospy.get_param('~robot_angular_acc',3.0)
        self.serialport = self.port_name
        self.baudrate   = self.baud_rate
        self.x          = xmw.XMiddleWare(self.serialport,self.baudrate)

        
        self.odom_data     = Odometry()
        self.vel_data      = Twist()
        self.battery_data  = Float32()
        self.imu_data      = Imu()
        self.wheel_a_speed = Int32()
        self.wheel_b_speed = Int32()
        self.wheel_c_speed = Int32()
        self.wheel_d_speed = Int32()
        self.wheel_a_set = Int32()
        self.wheel_b_set = Int32()
        self.wheel_c_set = Int32()
        self.wheel_d_set = Int32()
        self.odom_time_last = rospy.Time.now()
        

        self.rate_timer = rospy.Rate(self.control_rate)
        #self.rate_timer = rospy.Rate(25)
        self.connect()
        DynamicReconfigSrv    = Server(PID_reconfigConfig,self.PIDReconfigCallback)
        #self.x.SetPID(self.Kp,self.Ki,self.Kd)
    


    def connect(self):
       print(self.x.Init())
       #time.sleep(2)

    def setParams(self,encoder_resolution = 1440,wheel_diameter = 0.097,robot_linear_acc = 1.0, robot_angular_acc = 2.0, wheel_track = 0.0):
        encoder_resolution_calibrated = int(encoder_resolution/self.linear_correction_factor)
        #wheel_a_mec_calibrated        = wheel_a_mec/self.angular_correction_factor
        wheel_track_calibrated        =wheel_track/self.angular_correction_factor
        self.x.SetParams(encoder_resolution_calibrated,wheel_diameter,robot_linear_acc,robot_angular_acc,wheel_track_calibrated)

    def shutdown(self):
        try:
            rospy.loginfo("Robot Stoping")
            self.x.SetVelocity(0,0,0)
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Robot Stopped")

    def handle_cmd(self,req):
#        print("Set Vel: %.3f %.3f %.3f"%(req.linear.x,req.linear.y,req.angular.z))
        if self.is_omni :
            self.x.SetVelocity(req.linear.x,req.linear.y,req.angular.z)
        else:
            self.x.SetVelocity(req.linear.x,0,req.angular.z)
    
    def handle_imu(self):
        imu_list = self.x.GetIMU()
        
        self.imu_data.header.stamp = rospy.Time.now()
        #imu_q = quaternion_from_euler(imu_list[6],imu_list[7],imu_list[8])
        imu_q = quaternion_from_euler(0,0,imu_list[8])
        self.imu_data.orientation.x = imu_q[0]
        self.imu_data.orientation.y = imu_q[1]
        self.imu_data.orientation.z = imu_q[2]
        self.imu_data.orientation.w = imu_q[3]
        self.imu_data.angular_velocity.x    = imu_list[0]
        self.imu_data.angular_velocity.y    = imu_list[1]
        self.imu_data.angular_velocity.z    = imu_list[2]
        self.imu_data.linear_acceleration.x = imu_list[3]
        self.imu_data.linear_acceleration.y = imu_list[4]
        self.imu_data.linear_acceleration.z = imu_list[5]
        self.imu_data.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE
        self.imu_data.orientation_covariance         = IMU_ORIENTATION_COVARIANCE
        self.imu_data.angular_velocity_covariance    = IMU_ANGULAR_VELOCITY_COVARIANCE
        self.imu_pub.publish(self.imu_data)




    def handle_odom(self):
        odom_list = self.x.GetOdom()
        q = quaternion_from_euler(0,0,odom_list[2])
        quaternion = Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        self.odom_data.header.stamp = rospy.Time.now()
        self.odom_data.pose.pose.position.x = odom_list[0]
        self.odom_data.pose.pose.position.y = odom_list[1]
        self.odom_data.pose.pose.position.z = 0
        self.odom_data.pose.pose.orientation = quaternion
        #self.odom_data.twist.twist.linear.x = odom_list[3]*self.control_rate
        #self.odom_data.twist.twist.linear.y = odom_list[4]*self.control_rate
        #self.odom_data.twist.twist.angular.z = odom_list[5]*self.control_rate
        self.odom_data.twist.twist.linear.x = odom_list[3]/((rospy.Time.now() - self.odom_time_last).to_sec())
        self.odom_data.twist.twist.linear.y = odom_list[4]/((rospy.Time.now() - self.odom_time_last).to_sec())
        self.odom_data.twist.twist.angular.z = odom_list[5]/((rospy.Time.now() - self.odom_time_last).to_sec())
        self.odom_data.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_data.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom_pub.publish(self.odom_data)

        if( self.publish_odom_transform == True):
            self.odom_broadcaster.sendTransform(
                (self.odom_data.pose.pose.position.x,self.odom_data.pose.pose.position.y,0.0),
                (quaternion.x,quaternion.y,quaternion.z,quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                self.odom_frame
            )
        self.odom_time_last = rospy.Time.now()

    def handle_bat(self):
        self.battery_data.data = self.x.GetBattery()
        self.battery_pub.publish(self.battery_data)

    def handle_wheelspeed(self):
        (self.wheel_a_speed.data, self.wheel_b_speed.data, self.wheel_c_speed.data, self.wheel_d_speed.data,self.wheel_a_set.data,self.wheel_b_set.data,self.wheel_c_set.data,self.wheel_d_set.data) = self.x.GetWheelSpeed()
        self.avel_pub.publish(self.wheel_a_speed)
        self.bvel_pub.publish(self.wheel_b_speed)
        self.cvel_pub.publish(self.wheel_c_speed)
        self.dvel_pub.publish(self.wheel_d_speed)
        self.aset_pub.publish(self.wheel_a_set)
        self.bset_pub.publish(self.wheel_b_set)
        self.cset_pub.publish(self.wheel_c_set)
        self.dset_pub.publish(self.wheel_d_set)

    def PIDReconfigCallback(self,config,level):
        self.x.SetPID(config['Kp'],config['Ki'],config['Kd'])
        #print(config['Kp'],config['Ki'],config['Kd'])
        return config


    def loop(self):

        #imu_tmp  = self.getIMU()
        self.odom_data.header.frame_id = self.odom_frame
        self.odom_data.child_frame_id  = self.base_frame
        self.imu_data.header.frame_id  = self.imu_frame
        self.x.SetPID(self.Kp,self.Ki,self.Kd)
        time.sleep(0.1)
        self.setParams(encoder_resolution = self.encoder_resolution,wheel_diameter=self.wheel_diameter,robot_linear_acc=self.robot_linear_acc,robot_angular_acc=self.robot_angular_acc,wheel_track=self.wheel_track)
        time.sleep(0.1)
        print("Start Robot!")

        #rospy.spin()
        while not rospy.is_shutdown():
            self.rate_timer.sleep()

            self.handle_odom()
            self.handle_bat()
            self.handle_wheelspeed()
            self.handle_imu()

            
if __name__ == '__main__':
    robot = XMIDDLEWARE()
    robot.loop()
    




    

    





                    
