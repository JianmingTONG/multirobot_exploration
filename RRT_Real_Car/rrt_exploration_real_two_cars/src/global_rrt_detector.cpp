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

// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points,line;
float xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y;

rdm r; // for genrating random numbers


/////////////// added by jianming

geometry_msgs::PointStamped clickedpoint_transformed;
bool finishedTransform;
geometry_msgs::Point p;  
///////////////


//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData=*msg;
}

 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg, tf::TransformListener *listener)
{ 

////////////////////// original
	// geometry_msgs::Point p;  
	// p.x=msg->point.x;
	// p.y=msg->point.y;
	// p.z=msg->point.z;
	// points.points.push_back(p);	

	////////////////////// added by jianming
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

	////////////////////// done by jianming

}



int main(int argc, char **argv)
{

	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
	MTRand_int32 irand(init, length); // 32-bit int generator
	// this is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer
	// as a seed for a simpler initialization
	MTRand drand; // double in [0, 1) generator, already init

	// generate the same numbers as in the original C test program
	ros::init(argc, argv, "global_rrt_frontier_detector");
	tf::TransformListener listener;
	ros::NodeHandle nh;

	// fetching all parameters
	float eta, init_map_x, init_map_y,range;
	std::string map_topic, base_frame_topic;

	std::string ns;
	ns=ros::this_node::getName();

	ros::param::param<float>(ns+"/eta", eta, 1);
	ros::param::param<std::string>(ns+"/map_topic", map_topic, "map_merge/map"); 
	//---------------------------------------------------------------
	ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);	

	ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 30);
	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);

	ros::Rate rate(100); 
	
	
	// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
	while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

	ros::Subscriber rviz_sub=nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, boost::bind(&rvizCallBack, _1, &listener));

	//visualizations  points and lines..
	points.header.frame_id=mapData.header.frame_id;
	line.header.frame_id=mapData.header.frame_id;
	points.header.stamp=ros::Time(0);
	line.header.stamp=ros::Time(0);
		
	points.ns=line.ns = "markers";
	points.id = 0;
	line.id =1;


	points.type = points.POINTS;
	line.type=line.LINE_LIST;

	//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	points.action =points.ADD;
	line.action = line.ADD;
	points.pose.orientation.w =1.0;
	line.pose.orientation.w = 1.0;
	line.scale.x =  0.03;
	line.scale.y= 0.03;
	points.scale.x=0.3; 
	points.scale.y=0.3; 

	line.color.r =9.0/255.0;
	line.color.g= 91.0/255.0;
	line.color.b =236.0/255.0;
	points.color.r = 255.0/255.0;
	points.color.g = 0.0/255.0;
	points.color.b = 0.0/255.0;
	points.color.a=1.0;
	line.color.a = 1.0;
	points.lifetime = ros::Duration();
	line.lifetime = ros::Duration();

	geometry_msgs::Point p;  


	while(points.points.size()<2)
	{
		ros::spinOnce();

		pub.publish(points) ;
	}




	std::vector<float> temp1;
	temp1.push_back(points.points[0].x);
	temp1.push_back(points.points[0].y);
		
	std::vector<float> temp2; 
	temp2.push_back(points.points[1].x);
	temp2.push_back(points.points[0].y);


	init_map_x=Norm(temp1,temp2);
	temp1.clear();		temp2.clear();

	temp1.push_back(points.points[0].x);
	temp1.push_back(points.points[0].y);

	temp2.push_back(points.points[0].x);
	temp2.push_back(points.points[1].y);

	init_map_y=Norm(temp1,temp2);
	temp1.clear();		temp2.clear();

	Xstartx=(points.points[0].x+points.points[1].x)*.5;
	Xstarty=(points.points[0].y+points.points[1].y)*.5;





	// geometry_msgs::Point trans;
	// trans=points.points[2];
	std::vector< std::vector<float>  > V; 
	// xnew.push_back( trans.x);xnew.push_back( trans.y); 
	std::vector<float> xnew; 
	xnew.push_back( 0.5);xnew.push_back( 0.5); 
	V.push_back(xnew);

	points.points.clear();
	pub.publish(points) ;


	std::vector<float> frontiers;
	int i=0;
	float xr,yr;
	std::vector<float> x_rand, x_nearest, x_new;


	// Main loop
	while (ros::ok()){
		// Sample free
		x_rand.clear();
		xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
		yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
		x_rand.push_back( xr ); x_rand.push_back( yr );

		// Nearest
		x_nearest=Nearest(V,x_rand);

		// Steer
		x_new=Steer(x_nearest,x_rand,eta);

		// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
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
		else if (checking==1){
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
