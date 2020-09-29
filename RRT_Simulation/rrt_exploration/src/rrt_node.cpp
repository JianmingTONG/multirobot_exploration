#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <mutex>
// #define SEND_PYTHON_ACTIONLIB
// #define SEND_PYTHON_MOVEBASE
#define SEND_CPP_ACTIONLIB

std::mutex _mt;

// -----------------------------------  functions
double mapValue(const nav_msgs::OccupancyGrid & mapDataIn, const geometry_msgs::PointStamped & point){
    _mt.lock();
    double tempResult;
    int idx;
    idx = (floor((point.point.y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((point.point.x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
    tempResult = mapDataIn.data[idx];
    _mt.unlock();
    return tempResult;
}

double norm2Dpoints(const double& point1_x, const double& point1_y, const geometry_msgs::PointStamped & point2)
{
    _mt.lock();
    double tempResult;
    tempResult= pow(	(pow((point2.point.x-point1_x),2)+pow((point2.point.y-point1_y),2))	,0.5);
    _mt.unlock();
    return tempResult;
}


double informationRectangleGain(const nav_msgs::OccupancyGrid& mapDataIn, const geometry_msgs::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}

// -----------------------------------  global variables
nav_msgs::OccupancyGrid     mapData, costmapData;
geometry_msgs::PointStamped clickedpoint;
visualization_msgs::Marker  points,line;

// -----------------------------------  for genrating random numbers
rdm r; 

// -----------------------------------  Subscribers callback functions-
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	mapData=*msg;
    // std::cout << "assigner receives map" << std::endl;
    _mt.unlock();
}

void costmapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	costmapData=*msg;
    std::cout << "assigner receives costmap" << std::endl;
    _mt.unlock();
    
}

geometry_msgs::PointStamped clickedpoint_transformed;

bool finishedTransform;
geometry_msgs::Point p;  


void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg, tf::TransformListener *listener)
{ 
	finishedTransform=false;
	
	clickedpoint.header = msg->header;
	clickedpoint.point = msg->point;
	
	while(!finishedTransform){
		try{
			listener->transformPoint(mapData.header.frame_id, clickedpoint, clickedpoint_transformed);
			finishedTransform=true;
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	p.x=clickedpoint_transformed.point.x;
	p.y=clickedpoint_transformed.point.y;
	p.z=clickedpoint_transformed.point.z;
	points.points.push_back(p);

}


int main(int argc, char **argv)
{
	// -------------------------------------random number generator
 	// this is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer
	// as a seed for a simpler initialization
	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
	MTRand_int32 irand(init, length); // 32-bit int generator
	MTRand drand; // double in [0, 1) generator, already init

	// -------------------------------------initialize variables
	std::string map_topic, costmap_topic, base_frame_topic, frontier_topic;
    std::string robot_frame, robot_base_frame;
	int rateHz;
	double costmap_pixel_threshold, info_radius;
    geometry_msgs::PointStamped exploration_goal, goalRobotFrame, previousGoalMapFrame;
    float xdim, ydim, resolution, centerX, centerY, map_lengthX, map_lengthY, eta, range;
#ifdef SEND_PYTHON_MOVEBASE
    geometry_msgs::PoseStamped moveBaseGoal;
#endif
    // -------------------------------------initialized the variables which are in need in exploration
	std::vector<geometry_msgs::PointStamped> frontiers;
	int i=0;
	float xr,yr;
	std::vector<float> x_rand,x_nearest,x_new;
    double infoGain;
	double goal_score = -500.0, goal_infoGain = 0.0;
    int    goal_idx   = 0;
    
    // -------------------------------------initialize the robot node
	ros::init(argc, argv, "rrt_exploration_node");
	ros::NodeHandle nh;
	std::string nodename, ns;
	nodename=ros::this_node::getName();
	
	// -------------------------------------fetch params
	ros::param::param<std::string>(nodename+"/namespace", ns, "robot1/");
	ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map"); 
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/map");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_base_frame, "robot1/base_link");
    
    ros::param::param<int>(nodename+"/rate", rateHz, 20);
    ros::param::param<double>(nodename+"/info_radius", info_radius, 1.0);
    ros::param::param<double>(nodename+"/costmap_pixel_threshold", costmap_pixel_threshold, 1.0);
	ros::param::param<float>(nodename+"/eta", eta, 0.5);

	ros::Rate rate(rateHz);

	// -------------------------------------subscribe the map topics & clicked points
	tf::TransformListener listener;
	ros::Subscriber sub       = nh.subscribe(map_topic, 100 ,mapCallBack);	
    ros::Subscriber costMapSub= nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 10, costmapMergedCallBack);
	
    ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, boost::bind(&rvizCallBack, _1, &listener));

	// -------------------------------------publish the detected points for following processing & display
	ros::Publisher pub        = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 10);
#ifdef SEND_PYTHON_ACTIONLIB
	ros::Publisher goalPub    = nh.advertise<geometry_msgs::PointStamped>("/robot1/rrt_goal", 10);
#endif
#ifdef SEND_PYTHON_MOVEBASE
    ros::Publisher goalMoveBasePub = nh.advertise<move_base_msgs::MoveBaseGoal>("/robot1/rrt_goal", 10);
#endif
	// -------------------------------------wait until map is received
    std::cout << "wait for map "<< std::endl;
	while (mapData.header.seq<1     or mapData.data.size()<1    )  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
	std::cout << "wait for costmap "<< std::endl;
    while ( costmapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
    
    // ------------------------------------- action lib
#ifdef SEND_CPP_ACTIONLIB 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(ns + "/move_base", true);
    std::cout << "wait for actionserver"<< std::endl;
    ac.waitForServer();
#endif
#ifdef SEND_CPP_ACTIONLIB
    move_base_msgs::MoveBaseGoal robotGoal;
    robotGoal.target_pose.header.frame_id = robot_frame;
    robotGoal.target_pose.pose.position.z = 0;
    robotGoal.target_pose.pose.orientation.w = 1.0;
#endif

	// -------------------------------------initilize the visualized points & lines  
	points.header.frame_id  = mapData.header.frame_id;
	points.header.stamp     = ros::Time(0);
	points.type 			= points.POINTS;
	points.action           = points.ADD;
	points.pose.orientation.w =1.0;
	points.scale.x 			= 0.3; 
	points.scale.y			= 0.3; 
	points.color.r 			= 1.0;   // 255.0/255.0;
	points.color.g 			= 0.0;   // 0.0/255.0;
	points.color.b 			= 0.0;   // 0.0/255.0;
	points.color.a			= 1.0;
	points.lifetime         = ros::Duration();

	line.header.frame_id    = mapData.header.frame_id;
	line.header.stamp       = ros::Time(0);
	line.type				= line.LINE_LIST;
	line.action             = line.ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x 			= 0.03;
	line.scale.y			= 0.03;
	line.color.r			= 1.0;   // 0.0/255.0;
	line.color.g			= 0.0;   // 0.0/255.0;
	line.color.b 			= 1.0;   // 236.0/255.0;
	line.color.a 			= 1.0;
	line.lifetime           = ros::Duration();


	// -------------------------------------calculate the map size and the center point which are used in random number generation afterwards

	// centerX     =   costmapData.info.origin.position.x + map_lengthX*.5;
	// centerY     =   costmapData.info.origin.position.y + map_lengthY*.5;
	// ROS_INFO("Map width: %3.2f: height %3.2f", map_lengthX, map_lengthY);

	// -------------------------------------set the origin of the RRT
	std::vector< std::vector<float>  > V;  
	std::vector<float> xnew; 
	std::cout << "wait for transform"<< std::endl;
	tf::StampedTransform transform;
	int  temp=0;
	while (temp==0){
		try{
			temp=1;
			listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			temp=0;
			ros::Duration(0.1).sleep();
		}
	}

	xnew.push_back(transform.getOrigin().x()+0.3);
	xnew.push_back(transform.getOrigin().y()); 
	V.push_back(xnew);

    // -------------------------------------initialize the previous goal point
    previousGoalMapFrame.header.frame_id = robot_frame;
    previousGoalMapFrame.point.x         = 0;
    previousGoalMapFrame.point.y         = 0;
    previousGoalMapFrame.point.z         = 0;

	
	ROS_INFO("RRT Tree starts growing origin: %3.2f, %3.2f", xnew[0], xnew[1]);

    // -------------------------------------wait the clicked points  
	ROS_INFO("wait to start");
    while(points.points.size()<1)
	{
		ros::spinOnce();
		pub.publish(points) ;
	}
    // -------------------------------------clear clicked points
	// points.points.clear();
	// pub.publish(points);
    //---------------------------------------------------------------
    //------------------     Main   Loop     ------------------------
    //---------------------------------------------------------------
	while (ros::ok()){
        // ------------------------------- find 20 frontiers
        if(frontiers.size()< 10){
            map_lengthX = mapData.info.width*mapData.info.resolution;
	        map_lengthY = mapData.info.height*mapData.info.resolution;
            // std::cout << "start rrt tree growing" << std::endl;
            // Sample free
            x_rand.clear();
            xr=(drand()*map_lengthX) + mapData.info.origin.position.x;
            yr=(drand()*map_lengthY) + mapData.info.origin.position.y;
            x_rand.push_back( xr ); x_rand.push_back( yr );
            
            // ------------------ Nearest
            x_nearest=Nearest(V,x_rand);

            // ------------------ Steer
            x_new=Steer(x_nearest,x_rand,eta);

            // ------------------ ObstacleFree -1:unkown (frontier region)  0:free 1:obstacle
            char checking=ObstacleFree(x_nearest, x_new, mapData);

            if (checking==-1){
                exploration_goal.header.stamp=ros::Time(0);
                exploration_goal.header.frame_id=mapData.header.frame_id;
                exploration_goal.point.x=x_new[0];
                exploration_goal.point.y=x_new[1];
                exploration_goal.point.z=0.0;
                
                frontiers.push_back(exploration_goal);

                // points.pose.
            }
                
            else if (checking==0){
                V.push_back(x_new);
                
                p.x=x_new[0]; 
                p.y=x_new[1]; 
                p.z=0.0;
                line.points.push_back(p);
                p.x=x_nearest[0]; 
                p.y=x_nearest[1]; 
                p.z=0.0;
                line.points.push_back(p);	
                
            }
            pub.publish(line); 
        }
        else{
        // ------------------------------- find the goal which has the highest score.
            goal_score = -500;
            std::cout << "start goal filtering number of frontiers " << frontiers.size() << std::endl;
            listener.lookupTransform(mapData.header.frame_id, robot_base_frame, ros::Time(0), transform);
            goal_infoGain= 0.0;
            for(int i=0; i<frontiers.size(); i++){
                double score, pixel;
                infoGain = informationRectangleGain(mapData, frontiers[i], info_radius);
                std::cout << " infoGain " << infoGain << std::endl;
                pixel = mapValue(costmapData, frontiers[i]);
                std::cout << "get pixel value "<< pixel << std::endl;
                if(infoGain<0.2 || pixel > costmap_pixel_threshold){continue;}
                score = infoGain - norm2Dpoints(transform.getOrigin().x(), transform.getOrigin().y(), frontiers[i]);
                std::cout << "get score" << score << "goal_score" << goal_score << std::endl;
                if(score > goal_score){
                    goal_score    = score;
                    goal_infoGain = infoGain;
                    goal_idx      = i;  
                    std::cout << "replace goal: " << goal_idx << std::endl;
                }
            }
std::cout << "Goal infoGain new: "<< goal_infoGain << "\n\n" << std::endl;
                
#ifdef SEND_PYTHON_ACTIONLIB 
            goalPub.publish(frontiers[goal_idx]);
#endif

            try{
                // std::cout << "try converts wait for transform" << std::endl;
                if(listener.waitForTransform(frontiers[goal_idx].header.frame_id, robot_frame, ros::Time(0), ros::Duration(5))){
                    // std::cout << "try converts goal point" << std::endl;
                    listener.transformPoint(robot_frame, frontiers[goal_idx], goalRobotFrame);
                    // std::cout << "convert done" << std::endl;
                }
            }
            catch (tf::TransformException ex){
                continue;
            }

#if defined SEND_PYTHON_MOVEBASE 
            // ------------------------------ send move_base goal actionlib 
            std::cout << "send goal point" << std::endl;
            moveBaseGoal.header = mapData.header;
            moveBaseGoal.pose.orientation.w = 1.0;
            moveBaseGoal.pose.position.x    = frontiers[goal_idx].point.x;
            moveBaseGoal.pose.position.y    = frontiers[goal_idx].point.y;
            std::cout << "defined movebase goal" << std::endl;
            goalMoveBasePub.publish(moveBaseGoal);
#endif

            //  ------------------------------ actionlib 
#ifdef SEND_CPP_ACTIONLIB 
           
            robotGoal.target_pose.pose.position.x = goalRobotFrame.point.x;
            robotGoal.target_pose.pose.position.y = goalRobotFrame.point.y;
            robotGoal.target_pose.header.stamp    = ros::Time(0);
            // std::cout  <<"if state != active? "  << (ac.getState() != actionlib::SimpleClientGoalState::ACTIVE) << std::endl;
            if(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE){
                ac.sendGoal(robotGoal);
                previousGoalMapFrame = frontiers[goal_idx];
                std::cout << "goal: (" << robotGoal.target_pose.pose.position.x << ", " << robotGoal.target_pose.pose.position.y << ") " << std::endl;       
            }
            else{
                double score, pixel;
                // std::cout << "start infoGain Calculating" << std::endl;
                infoGain = informationRectangleGain(mapData, previousGoalMapFrame, info_radius);
                std::cout << "Previous Goal infoGain :" <<  infoGain << " vs new: "<< goal_infoGain << std::endl;
                pixel = mapValue(costmapData, previousGoalMapFrame);
                score = infoGain - norm2Dpoints(transform.getOrigin().x(), transform.getOrigin().y(), frontiers[goal_idx]);
                std::cout << "Previous Goal distance cost :" <<  norm2Dpoints(transform.getOrigin().x(), transform.getOrigin().y(), frontiers[goal_idx]) << std::endl;
                std::cout << "previous goal pixel data:" << pixel << std::endl;
                if( ((goal_infoGain - infoGain > 0.2) && (goal_score - score>1)) || pixel > costmap_pixel_threshold){
                    previousGoalMapFrame = frontiers[goal_idx];
                    ac.sendGoal(robotGoal);
                    std::cout << "goal: (" << robotGoal.target_pose.pose.position.x << ", " << robotGoal.target_pose.pose.position.y << ") " << std::endl;
                }
            }
#endif
            std::cout << "flush frontiers" << std::endl;
            frontiers.clear();
            std::vector<geometry_msgs::PointStamped>().swap(frontiers);
            std::cout << "loop ends\n\n" << std::endl;
        }

        
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
