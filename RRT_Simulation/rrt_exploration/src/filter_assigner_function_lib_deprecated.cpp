#include "ros/ros.h"
#include "MeanShift.h"
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include "rrt_exploration/PointArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include <iostream>
#include <stdio.h>

nav_msgs::OccupancyGrid           mapData;
nav_msgs::OccupancyGrid           costmapData;
std::vector<MeanShift::Point>     frontiers;
MeanShift::Point                  currentGoalMapFrame;

double norm2Dpoints(const MeanShift::Point& x1, const MeanShift::Point& x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}

int informationRectangleGain(const nav_msgs::OccupancyGrid& mapDataIn, const MeanShift::Point& point, const double r){
    int infoGainValue = 0;
    int index, init_index, last_index, length, start, end, r_region, limit;

    index = (floor((point[1]-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((point[0]-mapData.info.origin.position.x)/mapData.info.resolution));
                  
    r_region = int(r/mapData.info.resolution);
    init_index = index-r_region*(mapData.info.width+1);
    last_index = index+r_region*(mapData.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if(last_index < mapData.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                switch(mapData.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapData.info.width;
            end   += mapData.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                limit = ((start/mapData.info.width) + 2)*mapData.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && j < mapData.data.size()){
                    switch(mapData.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapData.info.width;
            end   += mapData.info.width;
        }
    }

    return infoGainValue*(pow(mapData.info.resolution,2));
}


void print_points(std::vector<std::vector<double> > points){
    for(int i=0; i<points.size(); i++){
        for(int dim = 0; dim<points[i].size(); dim++) {
            printf("%f ", points[i][dim]);
        }
        printf("\n");
    }
}

void frontiersCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{   
    // -------------------------------------initialize variables
    // received points should in global_frame 
    // map frame -- the publisher should also published point with coordinate in this frame
    MeanShift::Point temp;
    temp.push_back(msg->point.x);
    temp.push_back(msg->point.y);
	frontiers.push_back(temp);
    // ROS_INFO("assigner receives frontiers");
}

void mapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData=*msg;
    ROS_INFO("assigner receives map");
}

void costmapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	costmapData=*msg;
    ROS_INFO("assigner receives costmap");
}

int main(int argc, char **argv)
{
    // -------------------------------------initialize variables
    std::string map_topic, costmap_topic, frontier_topic, robot_frame, ns, nodename; 
    float rateHz, info_radius, costmap_pixel_threshold;
    int n_robots;
	
    MeanShift *msp = new MeanShift();
    double kernel_bandwidth = 0.5;
    bool isFirstGoal = true;

    ros::init(argc, argv, "filter_assigner");
    ros::NodeHandle nh;
    nodename = ros::this_node::getName();
    ros::param::param<std::string>(nodename+ "/namespace", ns, "robot1");
    ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map");
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base_node/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/frontier_topic", frontier_topic, "frontiers");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/map");
    ros::param::param<float>(nodename+"/rate", rateHz, 1.0);
    ros::param::param<int>(nodename+"/n_robots", n_robots, 1);
    ros::param::param<float>(nodename+"/info_radius", info_radius, 1.0);
    ros::param::param<float>(nodename+"/costmap_pixel_threshold", costmap_pixel_threshold, 1.0);

    ros::Rate rate(rateHz);
    tf::TransformListener listener;
    ros::Subscriber mapSub                = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, mapMergedCallBack);
    ros::Subscriber costMapSub            = nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 10, costmapMergedCallBack);
	ros::Subscriber frontierSub           = nh.subscribe<geometry_msgs::PointStamped>(frontier_topic, 40, frontiersCallBack);

	ros::Publisher  displayCentroidsPub   = nh.advertise<visualization_msgs::Marker>(ns + "/centroids", 10);

    while (mapData.data.size()<1)      {  ros::spinOnce(); ROS_INFO("Waiting for the mapData"); ros::Duration(0.1).sleep(); }
    while (costmapData.data.size()<1)  {  ros::spinOnce(); ROS_INFO("Waiting for the costmapData"); ros::Duration(0.1).sleep(); }
    
    // ------------------------------------- display centroids
    visualization_msgs::Marker displayCentroids;
    displayCentroids.header   =  mapData.header;
    displayCentroids.color.r  =  1;
    displayCentroids.color.g  =  1;
    displayCentroids.color.b  =  0;
    displayCentroids.color.a  =  1;
    displayCentroids.lifetime =  ros::Duration();
    displayCentroids.scale.x  =  0.2;
    displayCentroids.scale.y  =  0.2; 
    displayCentroids.type     =  displayCentroids.POINTS;
    displayCentroids.action   =  displayCentroids.ADD;
    displayCentroids.pose.orientation.w  = 1.0;
    ROS_INFO("assigner initialized the params & vars");
    
    // ------------------------------------- action lib
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(ns + "/move_base", true);
    ac.waitForServer();
    
    ROS_INFO("assigner starts the actionlib");

    // -------------------------------------------------------------------------
    // ---------------------     Main   Loop     -------------------------------
    // -------------------------------------------------------------------------
    while(nh.ok()){
        ROS_INFO("\n\n\nstart loop");
        
        while (frontiers.size()<20)       { ros::spinOnce(); }
        while (mapData.data.size()<1)     { ros::spinOnce(); }
        while (costmapData.data.size()<1) { ros::spinOnce(); }
    

        std::cout << "receive " << frontiers.size() << " frontiers" << std::endl;
        // ------------------------------------- cluster frontiers into centroids
        std::vector<MeanShift::Point> centroids = msp->meanshift_center(frontiers, kernel_bandwidth);
        if(!isFirstGoal){
            centroids.push_back(currentGoalMapFrame);
        }

        // ------------------------------------- filter centroids
        // calculate the info gain of each points.
        std::vector<int> infoGain;
        for(int i=0; i<centroids.size(); i++){
            infoGain.push_back(informationRectangleGain(mapData, centroids[i], info_radius));
        }

        // deleted the points with low infoGain and goals lying in obstacles
        int idx   = 0, idx_costmap = 0;
        bool cond = false;
        while(idx < centroids.size()){
            cond        = infoGain[idx] < 0.2;
            idx_costmap = (floor((centroids[idx][1]-costmapData.info.origin.position.y)/costmapData.info.resolution)*costmapData.info.width)+(floor((centroids[idx][0]-costmapData.info.origin.position.x)/costmapData.info.resolution));
            cond  = cond | (costmapData.data[idx_costmap] > 0);
            if(cond){
                centroids.erase(centroids.begin()+idx_costmap);
                infoGain.erase(infoGain.begin()+idx_costmap);
                idx--;
            }
            idx++;
        }
        std::cout << "remain " << centroids.size() << " cetroids" << std::endl;

        // ----------------------- calcuate the credits of each goal
        std::vector<double> credits;
        MeanShift::Point robotLocation, goalMapFrame;
        tf::StampedTransform transform;
		try{
			listener.waitForTransform(mapData.header.frame_id, robot_frame, ros::Time(0), ros::Duration(3));
			listener.lookupTransform(mapData.header.frame_id, robot_frame, ros::Time(0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue;
		}

        robotLocation.push_back(transform.getOrigin().x());
        robotLocation.push_back(transform.getOrigin().y());

        std::cout <<"robot location: (" << robotLocation[0] << ", " <<  robotLocation[1] << ")" << std::endl;
        
        for(int i=0; i<centroids.size(); i++){
            credits.push_back(infoGain[i]*3 - norm2Dpoints(robotLocation, centroids[i]));
        }

        std::vector<double>::iterator biggest = std::max_element(std::begin(credits), std::end(credits)); 
        goalMapFrame.push_back(centroids[std::distance(std::begin(credits), biggest)][0]);
        goalMapFrame.push_back(centroids[std::distance(std::begin(credits), biggest)][1]);

        geometry_msgs::PointStamped goalMapFrame_PointStamp, goalRobotFrame_PointStamp;
        goalMapFrame_PointStamp.header  = mapData.header;
        goalMapFrame_PointStamp.point.x = goalMapFrame[0];
        goalMapFrame_PointStamp.point.y = goalMapFrame[1];
        goalMapFrame_PointStamp.point.z = 0;
        if(goalMapFrame_PointStamp.point.x != currentGoalMapFrame[0] || goalMapFrame_PointStamp.point.x != currentGoalMapFrame[1] ){
            try{
                listener.transformPoint(robot_frame, goalMapFrame_PointStamp, goalRobotFrame_PointStamp);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            move_base_msgs::MoveBaseGoal robotGoal;
            robotGoal.target_pose.header = goalRobotFrame_PointStamp.header;
            robotGoal.target_pose.pose.position.z = 0;
            robotGoal.target_pose.pose.position.x = goalRobotFrame_PointStamp.point.x;
            robotGoal.target_pose.pose.position.y = goalRobotFrame_PointStamp.point.y;
            robotGoal.target_pose.pose.orientation.w = 1.0;

            currentGoalMapFrame = goalMapFrame;
            ac.sendGoal(robotGoal);
            std::cout << "goal: (" << goalMapFrame[0] << ", "<< goalMapFrame[1] << ")" << std::endl;
        }

        // ------------------------------ display centroids
        for(int i = 0; i< centroids.size(); i++){
            geometry_msgs::Point p; 
            p.x = centroids[i][0];
            p.y = centroids[i][1];
            p.z = 0;
            displayCentroids.points.push_back(p);
        }

        frontiers.clear();
        std::vector<MeanShift::Point>().swap(frontiers);
        
        nav_msgs::OccupancyGrid   emptyMap;
        mapData.data     = emptyMap.data;
        costmapData.data = emptyMap.data;
 
        ros::spinOnce();
        rate.sleep();
        std::cout << "finish loop" << std::endl;
    }
    
    
    return 0;
}