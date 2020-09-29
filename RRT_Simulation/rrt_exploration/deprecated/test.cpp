#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "MeanShift.h"

std::vector<MeanShift::Point>     frontiers;

void frontiersCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{   
    // -------------------------------------initialize variables
    // received points should in global_frame 
    // map frame -- the publisher should also published point with coordinate in this frame
    MeanShift::Point temp;
    temp.push_back(msg->point.x);
    temp.push_back(msg->point.y);
	frontiers.push_back(temp);
    ROS_INFO("assigner receives frontiers");
}


int main(int argc, char **argv)
{
    // -------------------------------------initialize variables
    std::string map_topic, costmap_topic, frontier_topic, robot_frame, ns, nodename; 
    float rateHz, info_radius, costmap_pixel_threshold;
    int n_robots;

    ros::init(argc, argv, "filter_assigner123");
    ros::NodeHandle nh;
    ros::Subscriber frontierSub           = nh.subscribe<geometry_msgs::PointStamped>("/robot1/frontiers", 30, frontiersCallBack);
    ros::spin();
    return 0;
}