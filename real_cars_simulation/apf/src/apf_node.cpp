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
#define RESOLUTION 0.05
#define K_ATTRACT (1/RESOLUTION)
#define ETA_REPLUSIVE (3/RESOLUTION)
#define DIS_OBTSTACLE 6
#define DISTANCE_THRES_VALID_OBSTACLE 160
#define THRESHOLD_TRANSFORM 0.5
#define ROBOT_INTERFERE_RADIUS 5

// #define FILE_DEBUG
// --------------------- ROS global variable
nav_msgs::OccupancyGrid     mapData, costmapData;
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

void costmapMergedCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    _mt.lock();
	costmapData=*msg;
    // std::cout << "assigner receives costmap" << std::endl;
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

void dismapConstruction_start_target(int* dismap_, int* dismap_backup_, int* curr, int HEIGHT, int WIDTH){
    std::vector<int *> curr_iter;
    std::vector<int *> next_iter;

    // initialize the dis_map with number of dismapBackup
    memcpy(dismap_, dismap_backup_, sizeof(int) * HEIGHT * WIDTH);

    int iter = 1;
    curr_iter.push_back(new int[2]{curr[0], curr[1]});

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dismap_[(curr[0]) * WIDTH + curr[1]] = -500;

    while (curr_iter.size() > 0) {
        for (int i = 0; i < curr_iter.size(); i++) {
            if (dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] - 1});
            }
        }
        curr_iter.swap(next_iter);
        std::vector<int *>().swap(next_iter);
        iter++;
    }
    dismap_[(curr[0]) * WIDTH + curr[1]] = 0;  // int only zero is available
    return ;
}

// --------------------------------------------------//
// ----------------------Main------------------------//
// --------------------------------------------------//
int main(int argc, char** argv) {
    // ---------------------------------------- ros initialization;
    std::string map_topic, costmap_topic;
    std::string robot_frame, robot_base_frame;
    int rateHz;
    ros::init(argc, argv, "apf_node");
    ros::NodeHandle nh;
	std::string nodename, ns, robot_ano_frame_suffix, robot_ano_frame_preffix;
    int rotation_count   = 0, n_robot, this_robot_idx;
    float inflation_radius;    // max 4 degree;
    bool start_condition = true;
    
    float rotation_w[3]  = {0.866,  0.500, 1.0};
    float rotation_z[3]  = {0.5  , -0.866, 0.0};
     
    nodename=ros::this_node::getName();

    ros::param::param<std::string>(nodename+"/map_topic", map_topic, "robot1/map"); 
    ros::param::param<std::string>(nodename+"/costmap_topic", costmap_topic, "robot1/move_base/global_costmap/costmap");
    ros::param::param<std::string>(nodename+"/robot_base_frame", robot_base_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/robot_frame", robot_frame, "robot1/base_link");
    ros::param::param<std::string>(nodename+"/namespace", ns, "robot1");
    ros::param::param<int>(nodename+"/rate", rateHz, 1);
    ros::param::param<float>(nodename+"/inflation_radius", inflation_radius, 4);
    ros::param::param<int>(nodename+"/n_robot", n_robot, 1);
    ros::param::param<int>(nodename+"/this_robot_idx", this_robot_idx, 1);
    ros::param::param<std::string>(nodename+"/robot_ano_frame_preffix", robot_ano_frame_preffix, "robot");
    ros::param::param<std::string>(nodename+"/robot_ano_frame_suffix", robot_ano_frame_suffix, "/map");

    ros::Rate rate(rateHz);

    // ------------------------------------- subscribe the map topics & clicked points
    ros::Subscriber sub       = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 20 ,mapCallBack);
    ros::Subscriber costMapSub= nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 20, costmapMergedCallBack);
    ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, rvizCallBack);

    // ------------------------------------- subscribe the map topics & clicked points
	tf::TransformListener listener;

    // ------------------------------------- publish the detected points for following processing & display
	ros::Publisher pub        = nh.advertise<visualization_msgs::Marker>(nodename+"_shapes", 100);

    // ------------------------------------- wait until map is received
    std::cout << ns << "wait for map "<< std::endl;
	while ( mapData.header.seq < 1  or  mapData.data.size() < 1 ){  ros::spinOnce();  ros::Duration(0.1).sleep(); }
    std::cout << ns << "wait for costmap "<< std::endl;
    while ( costmapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
    
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

    // -------------------------------------Transforms between all robots initilization;
    std::vector<tf::StampedTransform> transform_vec;
    for (int i = 0; i < n_robot; i++){
        tf::StampedTransform trans_ ;
        tf::Vector3 translation;
        translation.setX(0);
        translation.setY(0);
        translation.setZ(0);
        translation.setW(0);

        trans_.setOrigin(translation);
        transform_vec.push_back(trans_);
    }
    
    // -------------------------------------Initialize all robots' frame;
    std::vector<std::string> robots_frame;

    for (int i = 0; i < n_robot; i++){

        std::stringstream ss;              
        ss << robot_ano_frame_preffix;
        ss << i+1;
        ss << robot_ano_frame_suffix;

        robots_frame.push_back(ss.str());
    }

    // ------------------------------------- wait the clicked points  
    std::cout << ns << "wait to start" << std::endl;
    while(points.points.size()<1)
	{
		ros::spinOnce();
		pub.publish(points);
	}

    // -------------------------------------clear clicked points
	points.points.clear();
	pub.publish(points);

#ifdef FILE_DEBUG
    // ------------------------------------- define output file
    std::ofstream ofile_int;
    ofile_int.open("/home/nics/work/ROS_BACKUP/catkin_ws/src/apf/output_dismap.txt");

    std::ofstream ofile_map;
    ofile_map.open("/home/nics/work/ROS_BACKUP/catkin_ws/src/apf/output_map.txt");

    std::ofstream output_zero;
    output_zero.open("/home/nics/work/ROS_BACKUP/catkin_ws/src/apf/output_zero.txt");
#endif

    while(ros::ok()){
        // _mt.lock();
        // std::cout << "main loop"  << std::endl;
        std::vector<bool > ifmapmerged_vec;
        for(int i = 0; i < n_robot; i++){
            ifmapmerged_vec.push_back(false);
        }

        // detect whether robots' maps have been merged.
        if(n_robot>1){    
            for (int i_ = 0; i_ < n_robot; i_++){
                if(i_ + 1 == this_robot_idx){
                    continue;
                }

                tf::StampedTransform transform_;
                try{
                    listener.lookupTransform(robot_frame, robots_frame[i_], ros::Time(0), transform_);
                }
                catch (tf::TransformException &ex) {
                    continue;
                }

                float dis_origin = 0;
                dis_origin = abs(transform_.getOrigin().x() - transform_vec[i_].getOrigin().x()) ;
                dis_origin += abs(transform_.getOrigin().y() - transform_vec[i_].getOrigin().y()) ;
                dis_origin += abs(transform_.getOrigin().z() - transform_vec[i_].getOrigin().z()) ;
                dis_origin += abs(transform_.getRotation().x() - transform_vec[i_].getRotation().x()) ;
                dis_origin += abs(transform_.getRotation().y() - transform_vec[i_].getRotation().y()) ;
                dis_origin += abs(transform_.getRotation().z() - transform_vec[i_].getRotation().z()) ;
                dis_origin += abs(transform_.getRotation().w() - transform_vec[i_].getRotation().w()) ;

                if(dis_origin > THRESHOLD_TRANSFORM){
                    ifmapmerged_vec[i_] = true;
                    transform_vec[i_].setOrigin(transform_.getOrigin());
                    transform_vec[i_].setRotation(transform_.getRotation());
                }  
            }
        }

        // ---------------------------------------- variables from ROS input;
        int HEIGHT = mapData.info.height;
        int WIDTH  = mapData.info.width;
        uchar* pData;
        
        // ---------------------------------------- define variables;
        std::vector<int* > obstacles, path, targets;
        int currentLoc[2], goal[2]; //target[2], obstacle[2]
        float  minDis2Frontier;
        std::ifstream infile;
        cv::Mat mapmat;
        int map[HEIGHT*WIDTH];
        std::vector<int * > dismap_targets_ptr;
        int* dismap_backup = new int[HEIGHT*WIDTH];

        // ---------------------------------------- initialize the map & dismap
        for (int i=0; i<HEIGHT; i++)
        {
            for (int j=0; j<WIDTH; j++)
            {
                map[i*WIDTH + j] = (int) mapData.data[i*mapData.info.width + j];
                dismap_backup[i*WIDTH + j] = map[i*WIDTH + j];
            }
        }

        // ------------------------------------------ find the obstacles & targets
        for (int i = 2; i < HEIGHT-2; i++){
            for (int j = 2; j < WIDTH-2; j++){
                if(map[i*WIDTH + j] == 100){
                    obstacles.push_back(new int[2]{i,j});
                }
                else if(map[i*WIDTH + j] == -1){
                    // accessiable frontiers
                    int numFree = 0, temp1 = 0;

                    if (map[(i + 1)*WIDTH + j] == 0){
                        temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[i*WIDTH + j + 1] == 0){
                        temp1 = 0;
                        temp1 += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[(i - 1) *WIDTH + j] == 0){
                        temp1 = 0;
                        temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if (map[i * WIDTH + j - 1] == 0){
                        temp1 = 0;
                        temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                        temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                        numFree += (temp1 > 0);
                    }

                    if( numFree > 0 ) {
                        targets.push_back(new int[2]{i,j});
                    }
                }
            }
        }
        

        {
            // remove targets within the inflation radius of obstacles.
            std::cout << "number targets" << targets.size() << std::endl;
      
            for (int idx_target = targets.size()-1; idx_target >= 0; idx_target--) {
                
                float loc_x = targets[idx_target][1]*mapData.info.resolution + mapData.info.origin.position.x;
                float loc_y = targets[idx_target][0]*mapData.info.resolution + mapData.info.origin.position.y;
                int index_costmap = (loc_y - costmapData.info.origin.position.y)/costmapData.info.resolution * costmapData.info.width + (loc_x - costmapData.info.origin.position.x)/costmapData.info.resolution;
                if (costmapData.data[index_costmap] >0){
                    targets.erase(targets.begin() + idx_target);
                    continue;
                }
            }
            std::cout << "(costmap) number targets after erase" << targets.size() << std::endl;
        }
        
        {
            // remove targets within the inflation radius of obstacles.
            for(int idx_target = targets.size()-1; idx_target>=0; idx_target--) {
                for (int i = 0; i < obstacles.size(); i++) {
                    if (abs(targets[idx_target][0] - obstacles[i][0]) +
                        abs(targets[idx_target][1] - obstacles[i][1]) < inflation_radius) {
                        targets.erase(targets.begin() + idx_target);
                        break;
                    }
                }
            }
            std::cout << "(inforadius) number targets after erase" << targets.size() << std::endl;
        }

        // ---------------------------------------- define the current point;
        tf::StampedTransform  transform;
        int  temp=0;
        while (temp==0){
            try{
                temp=1;
                listener.lookupTransform( mapData.header.frame_id, robot_base_frame, ros::Time(0), transform );
            }
            catch( tf::TransformException ex ){
                temp=0;
                ros::Duration(0.1).sleep();
            }
        }
        currentLoc[0] = floor((transform.getOrigin().y()-mapData.info.origin.position.y)/mapData.info.resolution);
        currentLoc[1] = floor((transform.getOrigin().x()-mapData.info.origin.position.x)/mapData.info.resolution);
        path.push_back( currentLoc );


        {
            std::cout << "(far away) number obstacles before erase" << obstacles.size() << std::endl;
            // remove obstacles which are so far away than targets because they have no impact on targets.

            for(int idx_obstacle = obstacles.size()-1; idx_obstacle>=0; idx_obstacle--) {
                float dis_temp_ = abs(obstacles[idx_obstacle][0] - currentLoc[0]) + abs(obstacles[idx_obstacle][1] - currentLoc[1]);
                if (dis_temp_ > DISTANCE_THRES_VALID_OBSTACLE + 10) {
                    obstacles.erase(obstacles.begin() + idx_obstacle);
                    continue;
                }
            }
            std::cout << "(far away) number obstacles after erase" << obstacles.size() << std::endl << std::endl;
        }

        // ------------------------------------------ replace value 100 to -2 to avoid obstacles being regarded as valid distance
        // for(int i = 0; i< targets.size(); i++){
        //     dismap_backup[(targets[i][0])*WIDTH + targets[i][1]] = 0;
        // }

        for(int i = 0; i< obstacles.size(); i++){
            dismap_backup[(obstacles[i][0])*WIDTH + obstacles[i][1]] = -2;
        }


        // ------------------------------------------ cluster targets into different groups and find the center of each group.
        // Note: x & y value of detected targets are in increasing order because of the detection is in laser scan order.
        std::vector<int* > target_process(targets);
        std::vector<int* > cluster_center;
        std::vector<int>   infoGain_cluster;

        while(target_process.size() > 0){
            std::vector<int* > target_cluster;
            target_cluster.push_back(target_process.back());
            target_process.pop_back();

            bool condition = true;
            while(condition){
                condition = false;
                int size_target_process = target_process.size();
                for (int i = size_target_process-1; i >= 0 ; i--){
                    for (int j = 0; j < target_cluster.size(); j++){
                        int dis_ = abs(target_process[i][0] - target_cluster[j][0]) +  abs(target_process[i][1] - target_cluster[j][1]);
                        if(dis_ < 3){
                            target_cluster.push_back(target_process[i]);
                            target_process.erase(target_process.begin() + i);
                            condition = true;
                            break;
                        }
                    }
                }
            }

            int center_[2]={0, 0};
            int num_ = target_cluster.size();
            for(int i = 0; i < num_; i++){
                center_[0] += target_cluster[i][0];
                center_[1] += target_cluster[i][1];
            }

            float center_float[2] = {float(center_[0]), float(center_[1])};
            center_float[0] = center_float[0]/float(num_);
            center_float[1] = center_float[1]/float(num_);

            float min_dis_ = 100.0;
            int min_idx_   = 10000;
            for(int i = 0; i < num_; i++){
                float temp_dis_ = abs(center_float[0]-float(target_cluster[i][0])) + abs(center_float[1]-float(target_cluster[i][1]));
                if(temp_dis_ < min_dis_){
                    min_dis_ = temp_dis_;
                    min_idx_ = i;
                }
            }

            cluster_center.push_back(new int[2]{target_cluster[min_idx_][0], target_cluster[min_idx_][1]});
            infoGain_cluster.push_back(num_);
        }
        std::cout << "(cluster) number targets after erase" << cluster_center.size() << std::endl;
        // ------------------------------------------ Generate Dismap starting from targets
        int cluster_num = cluster_center.size();
        int** dismap_target = new int* [cluster_num];
        for (int i = 0; i<cluster_num; i++){
            dismap_target[i] = new int[HEIGHT*WIDTH];
        }

        for(int i = 0; i< cluster_num; i++) {
            dismapConstruction_start_target(dismap_target[i], dismap_backup, cluster_center[i], HEIGHT, WIDTH);
            dismap_targets_ptr.push_back(dismap_target[i]);
        }

        // ------------------------------------------ calculate path.
        int iteration = 1;
        minDis2Frontier  = 500;  // a random initialized value greater than all possible distances.
        while(iteration < 2000 && minDis2Frontier > 1){

            // ------------------------------------------
            // ------------------------------------------
            // ------------------------------------------ get the minimial potential of the points around currentLoc
            {
                // ------------------------------------------ put locations around the current location into loc_around
                float potential[4];
                int min_idx = 0;
                float min_potential = 10000;
                int* loc_around[4];

                // upper
                loc_around[0] = new int[2]{currentLoc[0], currentLoc[1]+1};
                // left
                loc_around[1] = new int[2]{currentLoc[0] - 1, currentLoc[1]};
                // down
                loc_around[2] = new int[2]{currentLoc[0]   , currentLoc[1] - 1};
                // right
                loc_around[3] = new int[2]{currentLoc[0] + 1, currentLoc[1]};

                // ------------------------------------------ calculate potentials of four neighbors of currentLoc
                for (int i = 0; i < 4; i++){
                    int curr_around[2]={loc_around[i][0], loc_around[i][1]};

                    { // ------------------------------------ calculate current potential
                        float attract = 0, repulsive = 0;
                        for (int j = 0; j < cluster_center.size(); j++){
                            // int temp_int = dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]];
                            float temp = float(dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]]);
                            if(temp < 1){
                                std::cout << "zero loc: (" <<  cluster_center[j][0]   << ", " <<  cluster_center[j][1] << ")" << " temp" << temp << std::endl;
                                std::cout << "curr loc: (" <<  curr_around[0]  << ", " << curr_around[1] << ")" << std::endl;
                                continue;
                            }
                            attract     = attract - K_ATTRACT*infoGain_cluster[j]/temp;
                        }

                        for (int j = 0; j < obstacles.size(); j++){
                            float dis_obst = abs(obstacles[j][0]- curr_around[0]) + abs(obstacles[j][1]- curr_around[1]);
                            if( dis_obst <= DIS_OBTSTACLE) {
                                float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
                                repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
                            }
                        }

                        // to increase the potential if currend point has been passed before
                        for (int j =0; j < path.size(); j++){
                            if(curr_around[0] == path[j][0] && curr_around[1] == path[j][1]){
                                attract += 5;
                            }
                        }

                        // Add impact of robots.
                        for (int j =0; j < n_robot; j++){
                            if(ifmapmerged_vec[i]){
                                int index_[2] = {int(round((transform_vec[i].getOrigin().y() - mapData.info.origin.orientation.y)/mapData.info.resolution)), int(round((transform_vec[i].getOrigin().x() - mapData.info.origin.orientation.x)/mapData.info.resolution))};
                                int dis_ = abs(curr_around[0] - index_[0]) + abs(curr_around[1] - index_[1]);  
                                if( dis_ < ROBOT_INTERFERE_RADIUS){
                                    attract += (1 / float(dis_) - 1 / ROBOT_INTERFERE_RADIUS);
                                }
                            }
                        }

                        potential[i] = attract + repulsive;
                        if(min_potential > potential[i] ){
                            min_potential = potential[i];
                            min_idx = i;
                        }
                    }
                }
                // std::cout << "potential" <<std::setprecision(5) << min_potential <<std::endl;
                path.push_back(loc_around[min_idx]);
            }

            currentLoc[0] = (path.back())[0];
            currentLoc[1] = (path.back())[1];
            
            for (int i = 0; i < cluster_center.size() ; i++){
                int temp_dis_ =  dismap_targets_ptr[i][(currentLoc[0])*WIDTH + currentLoc[1]];
                if( (temp_dis_ == 0) && (abs(currentLoc[0]-cluster_center[i][0]) + abs(currentLoc[1]-cluster_center[i][1])) > 0){
                    continue;
                }

                if(minDis2Frontier > temp_dis_ ){
                    minDis2Frontier = temp_dis_;
                }
            }
            iteration++;

            // ---------------------------------------- publish path for displaying in rviz
            if(iteration >= 1){
                p.x=(path[path.size()-2])[1] * mapData.info.resolution + mapData.info.origin.position.x; 
                p.y=(path[path.size()-2])[0] * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                p.x=currentLoc[1] * mapData.info.resolution + mapData.info.origin.position.x;
                p.y=currentLoc[0] * mapData.info.resolution + mapData.info.origin.position.y;
                p.z=0.0;
                line.points.push_back(p);
                pub.publish(line); 
            }

            // if(iteration%50 == 49){
            //     robotGoal.target_pose.pose.orientation.z = 1;
            //     robotGoal.target_pose.pose.orientation.w = 0;
            //     robotGoal.target_pose.pose.position.x = currentLoc[1]*mapData.info.resolution + mapData.info.origin.position.x;
            //     robotGoal.target_pose.pose.position.y = currentLoc[0]*mapData.info.resolution + mapData.info.origin.position.y;
            //     robotGoal.target_pose.header.stamp    = ros::Time(0);
            //     // std::cout << "goal: (" << robotGoal.target_pose.pose.position.x << ", " << robotGoal.target_pose.pose.position.y << ") " << "orientation.w=" << robotGoal.target_pose.pose.orientation.w <<  "orientation.z=" << robotGoal.target_pose.pose.orientation.z << std::endl;   
            //     ac.sendGoal(robotGoal);
            // }
        }
        goal[0] = path.back()[0];
        goal[1] = path.back()[1];
      
        if(start_condition){
            tf::StampedTransform  transform;
            int  temp=0;
            while (temp==0){
                try{
                    temp=1;
                    listener.lookupTransform( mapData.header.frame_id, robot_base_frame, ros::Time(0), transform );
                }
                catch( tf::TransformException ex ){
                    temp=0;
                    ros::Duration(0.1).sleep();
                }
            }
            int loc_x = transform.getOrigin().x();
            int loc_y = transform.getOrigin().y();

            robotGoal.target_pose.pose.orientation.z = rotation_z[rotation_count];
            robotGoal.target_pose.pose.orientation.w = rotation_w[rotation_count];
  
            robotGoal.target_pose.pose.position.x = loc_x + 0.2;
            robotGoal.target_pose.pose.position.y = loc_y + 0.2;
        
            start_condition = false;
        }
        else{
            robotGoal.target_pose.pose.orientation.z = 1;
            robotGoal.target_pose.pose.orientation.w = 0;
            robotGoal.target_pose.pose.position.x = goal[1]*mapData.info.resolution + mapData.info.origin.position.x;
            robotGoal.target_pose.pose.position.y = goal[0]*mapData.info.resolution + mapData.info.origin.position.y;
            robotGoal.target_pose.header.stamp    = ros::Time(0);
            // std::cout << "goal: (" << robotGoal.target_pose.pose.position.x << ", " << robotGoal.target_pose.pose.position.y << ") " << "orientation.w=" << robotGoal.target_pose.pose.orientation.w <<  "orientation.z=" << robotGoal.target_pose.pose.orientation.z << std::endl;   
            ac.sendGoal(robotGoal);
        }
        line.points.clear();

        // ------------------------------------------- clear memory
        delete [] dismap_backup;
        for (int i = 0; i<cluster_num; i++){
            delete []  dismap_target[i];
        }
        delete [] dismap_target;

        // ------------------------------------------- keep frequency stable
        // _mt.unlock();
        ros::spinOnce();
        rate.sleep();
    } 


    return 0;

}