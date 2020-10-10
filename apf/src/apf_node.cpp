// --------------------- ros things
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include <actionlib/client/simple_action_client.h>
#include <mutex>

// --------------------- APF include things
#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<opencv2/opencv.hpp>
#include<cstdlib>
#define K_ATTRACT 1
#define ETA_REPLUSIVE 1
#define DIS_OBTSTACLE 5



// --------------------- ROS global variable
nav_msgs::OccupancyGrid     mapData;
std::mutex _mt;
geometry_msgs::PointStamped clickedpoint;
visualization_msgs::Marker  points,line;

// -----------------------------------  Subscribers callback functions-
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	mapData=*msg;
    // std::cout << "assigner receives map" << std::endl;
    _mt.unlock();
}

geometry_msgs::Point p;  
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 
	p.x=msg->point.x;
	p.y=msg->point.y;
	p.z=msg->point.z;
	points.points.push_back(p);
}

// --------------------------------------------------//
// ----------------------Main------------------------//
// --------------------------------------------------//
int main(int argc, char** argv) {
    // ---------------------------------------- ros initialization;
    std::string map_topic;
    std::string robot_frame, robot_base_frame;
    int rateHz;
    ros::init(argc, argv, "apf_node");
    ros::NodeHandle nh;
	std::string nodename, ns;
    
    nodename=ros::this_node::getName();

    ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map"); 
    ros::param::param<std::string>(nodename+"/robot_base_frame", robot_base_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/namespace", ns, "robot1/");
    ros::param::param<int>(nodename+"/rate", rateHz, 1);
    
    ros::Rate rate(rateHz);

    // ------------------------------------- subscribe the map topics & clicked points
    ros::Subscriber sub       = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 100 ,mapCallBack);
    ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, rvizCallBack);

    // ------------------------------------- subscribe the map topics & clicked points
	tf::TransformListener listener;

    // ------------------------------------- publish the detected points for following processing & display
	ros::Publisher pub        = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 10);

    // ------------------------------------- wait until map is received
    std::cout << ns << "wait for map "<< std::endl;
	while ( mapData.header.seq < 1  or  mapData.data.size() < 1 ){  ros::spinOnce();  ros::Duration(0.1).sleep(); }

    // ------------------------------------- action lib    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(ns + "/move_base", true);
    std::cout << ns << "wait for actionserver"<< std::endl;
    ac.waitForServer();

    move_base_msgs::MoveBaseGoal robotGoal;
    robotGoal.target_pose.header.frame_id = robot_frame;
    robotGoal.target_pose.pose.position.z = 0;
    robotGoal.target_pose.pose.orientation.z = 1.0;

    // ------------------------------------- initilize the visualized points & lines  
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

    // ------------------------------------- wait the clicked points  
    std::cout << ns << "wait to start" << std::endl;
    while(points.points.size()<1)
	{
		ros::spinOnce();
		pub.publish(points);
	}

    while(ros::ok()){
        // ---------------------------------------- variables from ROS input;
        int HEIGHT = mapData.info.height;
        int WIDTH  = mapData.info.width;

        // ---------------------------------------- define variables;
        std::vector<std::vector<int>> obstacles, path, targets;
        std::vector<int> obstacle, target, currentLoc, goal;
        std::vector<float> infoGain;
        float currentPotential, tempInfoGain, minDis2Frontier;
        cv::Mat mapmat;
        int map[HEIGHT][WIDTH];
        float dismap_backup[HEIGHT][WIDTH], dismap[HEIGHT][WIDTH], potentialmap[HEIGHT][WIDTH];

        // ---------------------------------------- initialize the potentialmap
        for (int i = 0; i < HEIGHT; i++) {
            for (int j = 0; j < WIDTH; j++) {
                potentialmap[i][j] = 0;
                dismap[i][j] = 0;
            }
        }

        // ---------------------------------------- initialize the dismap & potential map from external files;
        for (int i = 0; i < HEIGHT; i++) {
            for (int j = 0; j < WIDTH; j++) {
                int index = i*mapData.info.width + j;
                if(mapData.data[index] == 100){
                    dismap_backup[i][j] = 0;    // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
                }
                else
                if(mapData.data[index] == -1){
                    dismap_backup[i][j] = -400; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
                }
                else{
                    dismap_backup[i][j] = 0;
                }
            }
        }

        // ---------------------------------------- define the current point;
         tf::StampedTransform  transform;
        int  temp=0;
        while (temp==0){
            try{
                temp=1;
                listener.lookupTransform( mapData.header.frame_id, robot_base_frame, ros::Time(0), transform );
            }
            catch ( tf::TransformException ex ){
                temp=0;
                ros::Duration(0.1).sleep();
            }
        }
        currentLoc.push_back( floor((transform.getOrigin().x()-mapData.info.origin.position.x)/mapData.info.resolution) );
        currentLoc.push_back( floor((transform.getOrigin().y()-mapData.info.origin.position.y)/mapData.info.resolution));
        path.push_back( currentLoc );
        
        // ---------------------------------------- intialize the path visualization variable
        int path_view[HEIGHT][WIDTH];
        for( int i = 0 ; i< HEIGHT; i++){
            for( int j = 0 ; j< HEIGHT; j++) {
                path_view[i][j] = 0;
            }
        }

        // ------------------------------------------ find the obstacles & targets
        for (int i = 1; i < HEIGHT-1; i++) {
            for (int j = 1; j < WIDTH-1; j++) {
                int index = i*mapData.info.width +j;
                if(mapData.data[index] == 100){
                    obstacle.push_back(i);
                    obstacle.push_back(j);
                    obstacles.push_back(obstacle);
                    std::vector<int>().swap(obstacle);
                }
                else if(mapData.data[index] == -1) {
                    // accessiable frontiers
                    int numFree = 0, temp1 = 0;

                    if (mapData.data[index + mapData.info.width] == 0) {
                        temp1 = temp1 + (mapData.data[index + 2 * mapData.info.width] == 0) ? 1 : 0;  //[i+2][ j ]
                        temp1 = temp1 + (mapData.data[index + mapData.info.width + 1] == 0) ? 1 : 0;  //[i+1][j+1]
                        temp1 = temp1 + (mapData.data[index + mapData.info.width - 1] == 0) ? 1 : 0;  //[i+1][j-1]
                        numFree += +(temp1 > 0);
                    }

                    if (mapData.data[index+1] == 0) {
                        temp1 = 0;
                        temp1 = temp1 + (mapData.data[index + 2                     ] == 0) ? 1 : 0;   //[i  ][j+2]
                        temp1 = temp1 + (mapData.data[index + mapData.info.width + 1] == 0) ? 1 : 0;   //[i + 1][j + 1]
                        temp1 = temp1 + (mapData.data[index - mapData.info.width + 1] == 0) ? 1 : 0;   //[i - 1][j + 1]
                        numFree += (temp1 > 0);
                    }

                    if (mapData.data[index - mapData.info.width] == 0) {
                        temp1 = 0;
                        temp1 = temp1 + (mapData.data[index - mapData.info.width + 1] == 0) ? 1 : 0;  //[i - 1][j + 1]
                        temp1 = temp1 + (mapData.data[index - mapData.info.width - 1] == 0) ? 1 : 0;  //[i - 1][j - 1]
                        temp1 = temp1 + (mapData.data[index - 2 * mapData.info.width] == 0) ? 1 : 0;  //[i - 2][j    ]
                        numFree += (temp1 > 0);
                    }

                    if (mapData.data[index- 1] == 0) {
                        temp1 = 0;
                        temp1 += (mapData.data[index - 2                     ] == 0) ? 1 : 0;  //[i    ][j - 2]
                        temp1 += (mapData.data[index + mapData.info.width - 1] == 0) ? 1 : 0;  //[i + 1][j - 1]
                        temp1 += (mapData.data[index - mapData.info.width - 1] == 0) ? 1 : 0;  //[i - 1][j - 1]
                        numFree += (temp1 > 0);
                    }

                    if( numFree > 0 ) {
                        target.push_back(i);
                        target.push_back(j);
                        targets.push_back(target);
                        std::vector<int>().swap(target);
                    }
                }
            }
        }

        // ------------------------------------------ calculate infoGain of targets
        for ( int i = 0; i < targets.size(); i++){
            tempInfoGain = 0;
            tempInfoGain += (map[targets[i][0]+1][targets[i][1]+1] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]+1][targets[i][1]-1] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]+1][targets[i][1]  ] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]-1][targets[i][1]+1] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]-1][targets[i][1]-1] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]-1][targets[i][1]  ] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]  ][targets[i][1]+1] == 255)?1:0;
            tempInfoGain += (map[targets[i][0]  ][targets[i][1]-1] == 255)?1:0;

            infoGain.push_back(tempInfoGain);
        }

        // ------------------------------------------ set locations in dismap corresponding to targets as 0 to ensure that targets' distance will be calculated in the following operation. (dismapConstruction function).
        for(int i = 0; i< targets.size(); i++){
            dismap_backup[targets[i][0]][targets[i][1]] = 0;
        }
        for(int i = 0; i< obstacles.size(); i++){
            dismap_backup[obstacles[i][0]][obstacles[i][1]] = 0;
        }

        // ------------------------------------------ print the obstacles and targets' size.
        std::cout << obstacles.size() << std::endl;
        std::cout << targets.size()   << std::endl;

        // ------------------------------------------ calculate path.
        int iteration = 0;

        currentPotential = 500;  // a random initialized value greater than all possible potentials.
        minDis2Frontier  = 500;  // a random initialized value greater than all possible distances.
        while(iteration < 2000 && minDis2Frontier > 1){

            // ------------------------------------------ get the minimial potential of the points around currentLoc
            {
                // ------------------------------------------ put locations around the current location into loc_around
                std::vector<double> potential;
                std::vector<int> curr_around;
                std::vector<std::vector<int>> loc_around;

                // upper
                curr_around.push_back(currentLoc[0]);
                curr_around.push_back(currentLoc[1]+1);
                loc_around.push_back(curr_around);

                // left
                std::vector<int>().swap(curr_around);
                curr_around.push_back(currentLoc[0]-1);
                curr_around.push_back(currentLoc[1]);
                loc_around.push_back(curr_around);

                // down
                std::vector<int>().swap(curr_around);
                curr_around.push_back(currentLoc[0]);
                curr_around.push_back(currentLoc[1]-1);
                loc_around.push_back(curr_around);

                // right
                std::vector<int>().swap(curr_around);
                curr_around.push_back(currentLoc[0]+1);
                curr_around.push_back(currentLoc[1]);
                loc_around.push_back(curr_around);


                // ------------------------------------------ calculate potentials of four neighbors of currentLoc
                for (int i = 0; i < loc_around.size(); i++){
                    std::vector<int>().swap(curr_around);
                    curr_around.push_back(loc_around[i][0]);
                    curr_around.push_back(loc_around[i][1]);

                    { // ------------------------------------------ calculate dismap
                        std::vector<std::vector<int> > curr_iter, next_iter;

                        // initialize the dis_map with number of dismapBackup
                        memcpy(dismap, dismap_backup,sizeof(float)*HEIGHT*WIDTH);

                        for(int i = 0; i < obstacles.size(); i++){
                            dismap[obstacles[i][0] ][obstacles[i][1] ] = 0;
                        }

                        for(int i = 0; i < targets.size(); i++){
                            dismap[targets[i][0] ][targets[i][1] ] = 0;
                        }

                        int iter = 1;
                        curr_iter.push_back(curr_around);

                        // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
                        dismap[curr_around[0]][curr_around[1]] = -500;

                        while (curr_iter.size() > 0) {
                            for (int i = 0; i < curr_iter.size(); i++) {
                                if (dismap[curr_iter[i][0] + 1][curr_iter[i][1]] == 0) {
                                    dismap[curr_iter[i][0] + 1][curr_iter[i][1]] = iter;
                                    std::vector<int> tempLoc;
                                    tempLoc.push_back(curr_iter[i][0] + 1);
                                    tempLoc.push_back(curr_iter[i][1]);
                                    next_iter.push_back(tempLoc);
                                }


                                if (dismap[curr_iter[i][0]][curr_iter[i][1] + 1] == 0) {
                                    dismap[curr_iter[i][0]][curr_iter[i][1] + 1] = iter;
                                    std::vector<int> tempLoc;
                                    tempLoc.push_back(curr_iter[i][0]);
                                    tempLoc.push_back(curr_iter[i][1] + 1);
                                    next_iter.push_back(tempLoc);
                                }

                                if (dismap[curr_iter[i][0] - 1][curr_iter[i][1]] == 0) {
                                    dismap[curr_iter[i][0] - 1][curr_iter[i][1]] = iter;
                                    std::vector<int> tempLoc;
                                    tempLoc.push_back(curr_iter[i][0] - 1);
                                    tempLoc.push_back(curr_iter[i][1]);
                                    next_iter.push_back(tempLoc);
                                }

                                if (dismap[curr_iter[i][0]][curr_iter[i][1] - 1] == 0) {
                                    dismap[curr_iter[i][0]][curr_iter[i][1] - 1] = iter;
                                    std::vector<int> tempLoc;
                                    tempLoc.push_back(curr_iter[i][0]);
                                    tempLoc.push_back(curr_iter[i][1] - 1);
                                    next_iter.push_back(tempLoc);
                                }
                            }
                            curr_iter.swap(next_iter);
                            std::vector<std::vector<int> >().swap(next_iter);
                            iter++;
                        }
                        dismap[curr_around[0]][curr_around[1]] = 0.1;
                    }

                    { // ------------------------------------ calculate current potential
                        float attract = 0, repulsive = 0;
                        for (int i = 0; i < targets.size(); i++){
                            float temp = dismap[targets[i][0]][targets[i][1]];
                            if(temp < 0.1){
                                std::cout << "zero loc: (" <<  targets[i][0] << ", " <<  targets[i][1] << ")" << " temp" << temp << std::endl;
                                std::cout << "curr loc: (" <<  curr_around[0]   << ", " <<        curr_around[1] << ")" << std::endl;
                            }
                            attract     = attract - K_ATTRACT*infoGain[i]/temp;
                        }

                        for (int j = 0; j < obstacles.size(); j++){
                            float dis_obst = dismap[ obstacles[j][0] ][ obstacles[j][1] ];
                            if( dis_obst <= DIS_OBTSTACLE) {
                                float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
                                repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
                            }
                        }

                        potential.push_back(attract + repulsive);
                    }

                }

                { // ------------------------------------ calculate current potential
                    float attract = 0, repulsive = 0;
                    for (int i = 0; i < targets.size(); i++){
                        float temp = dismap[targets[i][0]][targets[i][1]];
                        if(temp < 0.1){
                            std::cout << "zero loc: (" <<  targets[i][0] << ", " <<  targets[i][1] << ")" << " temp" << temp << std::endl;
                            std::cout << "curr loc: (" <<  curr_around[0]   << ", " <<        curr_around[1] << ")" << std::endl;
                        }
                        attract     = attract - K_ATTRACT*infoGain[i]/temp;
                    }

                    for (int j = 0; j < obstacles.size(); j++){
                        float dis_obst = dismap[ obstacles[j][0] ][ obstacles[j][1] ];
                        if( dis_obst <= DIS_OBTSTACLE) {
                            float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
                            repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
                        }
                    }
                    potential.push_back(attract + repulsive);
                }

                std::vector<int>().swap(curr_around);

                // find the minimal potential around the current location
                std::vector<double>::iterator min_idx = std::min_element(potential.begin(), potential.end());

                if(currentPotential>potential[std::distance(std::begin(potential), min_idx)]){
                    path.push_back(loc_around[std::distance(std::begin(potential), min_idx)]);
                    currentPotential = potential[std::distance(std::begin(potential), min_idx)];
                }
                else{
                    // to avoid local minimum
                    std::srand(time(NULL));
                    int randIndex = std::rand()%4;
                    path.push_back(loc_around[randIndex]);
                    currentPotential = potential[randIndex];
                }
            }

            path_view[path.back()[0]][path.back()[1]] = 1;
            currentLoc[0] = (path.back())[0];
            currentLoc[1] = (path.back())[1];

            // ---------------------------------------- publish path for displaying in rviz
            if(iteration >= 1){
                p.x=(path[iteration-1])[0] * mapData.info.resolution + mapData.info.origin.position.x; 
                p.y=(path[iteration-1])[1] * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                p.x=currentLoc[0] * mapData.info.resolution + mapData.info.origin.position.x;
                p.y=currentLoc[1] * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                pub.publish(line); 
            }

            // ----------------------------------------- calculate whether the terminated condition has been satisfied
            {   // ------------------------------------------ calculate dismap
                std::vector<std::vector<int> > curr_iter, next_iter;

                // initialize the dis_map with number of dismapBackup
                memcpy(dismap, dismap_backup,sizeof(float)*HEIGHT*WIDTH);

                for(int i = 0; i < obstacles.size(); i++){
                    dismap[obstacles[i][0] ][obstacles[i][1] ] = 0;
                }

                for(int i = 0; i < targets.size(); i++){
                    dismap[targets[i][0] ][targets[i][1] ] = 0;
                }

                int iter = 1;
                curr_iter.push_back(currentLoc);

                // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
                dismap[currentLoc[0]][currentLoc[1]] = -500;

                while (curr_iter.size() > 0) {
                    for (int i = 0; i < curr_iter.size(); i++) {
                        if (dismap[curr_iter[i][0] + 1][curr_iter[i][1]] == 0) {
                            dismap[curr_iter[i][0] + 1][curr_iter[i][1]] = iter;
                            std::vector<int> tempLoc;
                            tempLoc.push_back(curr_iter[i][0] + 1);
                            tempLoc.push_back(curr_iter[i][1]);
                            next_iter.push_back(tempLoc);
                        }


                        if (dismap[curr_iter[i][0]][curr_iter[i][1] + 1] == 0) {
                            dismap[curr_iter[i][0]][curr_iter[i][1] + 1] = iter;
                            std::vector<int> tempLoc;
                            tempLoc.push_back(curr_iter[i][0]);
                            tempLoc.push_back(curr_iter[i][1] + 1);
                            next_iter.push_back(tempLoc);
                        }

                        if (dismap[curr_iter[i][0] - 1][curr_iter[i][1]] == 0) {
                            dismap[curr_iter[i][0] - 1][curr_iter[i][1]] = iter;
                            std::vector<int> tempLoc;
                            tempLoc.push_back(curr_iter[i][0] - 1);
                            tempLoc.push_back(curr_iter[i][1]);
                            next_iter.push_back(tempLoc);
                        }

                        if (dismap[curr_iter[i][0]][curr_iter[i][1] - 1] == 0) {
                            dismap[curr_iter[i][0]][curr_iter[i][1] - 1] = iter;
                            std::vector<int> tempLoc;
                            tempLoc.push_back(curr_iter[i][0]);
                            tempLoc.push_back(curr_iter[i][1] - 1);
                            next_iter.push_back(tempLoc);
                        }
                    }
                    curr_iter.swap(next_iter);
                    std::vector<std::vector<int> >().swap(next_iter);
                    iter++;
                }
                dismap[currentLoc[0]][currentLoc[1]] = 0.1;
            }

            for (int i = 0; i < targets.size() ; i++){
                if(minDis2Frontier > dismap[targets[i][0]][targets[i][1]] ){
                    minDis2Frontier = dismap[targets[i][0]][targets[i][1]];
                }
            }
            iteration++;
        }
        goal.push_back(path.back()[0]);
        goal.push_back(path.back()[1]);


        robotGoal.target_pose.pose.position.x = goal[0]*mapData.info.resolution + mapData.info.origin.position.x;
        robotGoal.target_pose.pose.position.y = goal[1]*mapData.info.resolution + mapData.info.origin.position.y;
        robotGoal.target_pose.header.stamp    = ros::Time(0);
        ac.sendGoal(robotGoal);
        
        std::cout << "goal: (" << robotGoal.target_pose.pose.position.x << ", " << robotGoal.target_pose.pose.position.y << ") " << std::endl;   

        // ------------------------------------------- keep frequency stable
        ros::spinOnce();
		rate.sleep();
    }
    return 0;

}