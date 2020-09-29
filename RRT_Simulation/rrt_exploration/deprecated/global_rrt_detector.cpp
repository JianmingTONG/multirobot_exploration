#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"
#include <boost/make_shared.hpp>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

// -----------------------------------  global variables
nav_msgs::OccupancyGrid     mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker  points,line;
float xdim, ydim, resolution, centerX, centerY, map_lengthX, map_lengthY;

// -----------------------------------  for genrating random numbers
rdm r; 

// -----------------------------------  Subscribers callback functions-
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData=*msg;
}


////////////////////original code start
// void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg, tf::TransformListener *listener)
// { 
	// geometry_msgs::Point p;  
	// p.x=msg->point.x;
	// p.y=msg->point.y;
	// p.z=msg->point.z;
	// points.points.push_back(p);	
// }
////////////////////original code ends


//////////////////// new code starts (added by jianming)
// ------------------------------------ transform the received points.
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
//////////////////// new code ends (added by jianming)





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
	float eta, map_lengthX, map_lengthY,range;
	std::string map_topic, base_frame_topic, ns;

	// -------------------------------------initialize the robot node
	ros::init(argc, argv, "global_rrt_frontier_detector");
	ros::NodeHandle nh;

	ros::Rate rate(100);
	ns=ros::this_node::getName();
	
	// -------------------------------------fetch params
	ros::param::param<float>(ns+"/eta", eta, 0.5);
	ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot1/map"); 
	
	// -------------------------------------subscribe the map topics & clicked points
	tf::TransformListener listener;
	ros::Subscriber sub       = nh.subscribe(map_topic, 100 ,mapCallBack);	
	ros::Subscriber rviz_sub  = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, boost::bind(&rvizCallBack, _1, &listener));

	// -------------------------------------publish the detected points for following processing & display
	ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 30);
	ros::Publisher pub        = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);
	
	// -------------------------------------wait until map is received  
	while (mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

	// -------------------------------------initilize the visualized points & lines  
	points.header.frame_id  = mapData.header.frame_id;
	points.header.stamp     = ros::Time(0);
	points.type 			= points.POINTS;
	points.action           = points.ADD;
	points.pose.orientation.w =1.0;
	points.scale.x 			= 0.3; 
	points.scale.y			= 0.3; 
	points.color.r 			= 0.0;   // 255.0/255.0;
	points.color.g 			= 0.0;   // 0.0/255.0;
	points.color.b 			= 1.0;   // 0.0/255.0;
	points.color.a			= 1.0;
	points.lifetime         = ros::Duration();

	line.header.frame_id    = mapData.header.frame_id;
	line.header.stamp       = ros::Time(0);
	line.type				= line.LINE_LIST;
	line.action             = line.ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x 			= 0.03;
	line.scale.y			= 0.03;
	line.color.r			= 0.0;   // 0.0/255.0;
	line.color.g			= 0.0;   // 0.0/255.0;
	line.color.b 			= 1.0;   // 236.0/255.0;
	line.color.a 			= 1.0;
	line.lifetime           = ros::Duration();

	// -------------------------------------wait the clicked points  
	while(points.points.size()<2)
	{
		ros::spinOnce();
		pub.publish(points) ;
	}

	// -------------------------------------calculate the map size and the center point which are used in random number generation afterwards
	map_lengthX = abs(points.points[1].x - points.points[0].x);
	map_lengthY = abs(points.points[1].y - points.points[0].y);
	centerX     =    (points.points[0].x + points.points[1].x)*.5;
	centerY     =    (points.points[0].y + points.points[1].y)*.5;

	// -------------------------------------set the first goal
	std::vector< std::vector<float>  > V;  
	std::vector<float> xnew; 
	xnew.push_back(-0.5);xnew.push_back(-0.5); 
	V.push_back(xnew);

	// -------------------------------------clear clicked points
	points.points.clear();
	pub.publish(points);

	// -------------------------------------initialized the variables which are in need in exploration
	int   i = 0;
	float xr, yr;
	std::vector<float> frontiers;
	std::vector<float> x_rand, x_nearest, x_new;

    //---------------------------------------------------------------
    //------------------     Main   Loop     ------------------------
    //---------------------------------------------------------------
	while (ros::ok()){
		// Sample free
		x_rand.clear();
		xr=(drand()*map_lengthX)-(map_lengthX*0.5)+centerX;
		yr=(drand()*map_lengthY)-(map_lengthY*0.5)+centerY;
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
			p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
			points.points.push_back(p);
			pub.publish(points) ;
			targetspub.publish(exploration_goal);
			points.points.clear();
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

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
