#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>

using namespace std;
#define SINGLE_ROBOT
#define SECOND_ROBOT
// #define THIRD_ROBOT



nav_msgs::OccupancyGrid msg1;
nav_msgs::OccupancyGrid mapData1;

#ifdef SECOND_ROBOT
nav_msgs::OccupancyGrid msg2;
nav_msgs::OccupancyGrid mapData2;
#endif

#ifdef THIRD_ROBOT
nav_msgs::OccupancyGrid msg3;
nav_msgs::OccupancyGrid mapData3;
#endif

nav_msgs::OccupancyGrid msg_merge;


void mapCallBack1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData1=*msg;
ROS_INFO("gointo callback 1");
}

#ifdef SECOND_ROBOT
void mapCallBack2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData2=*msg;
ROS_INFO("gointo callback 2");
}
#endif

#ifdef THIRD_ROBOT
void mapCallBack3(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData3=*msg;
ROS_INFO("gointo callback 3");
}
#endif

signed char pixel_merge(signed char merged_pixel,signed char local_pixel)
{
  
  if(merged_pixel==-1){merged_pixel=local_pixel;}
  else if (merged_pixel==0)
  {
    if(local_pixel==100){merged_pixel=local_pixel;}
  }
  else if(merged_pixel==100){}
  return merged_pixel;
}

signed char costmap_pixel_merge(signed char merged_pixel,signed char local_pixel)
{
  
  if(merged_pixel > local_pixel){return merged_pixel;}
  else {return local_pixel;}
}

//nav_msgs::OccupancyGrid
void merge(nav_msgs::OccupancyGrid &mergedMap, std::vector<nav_msgs::OccupancyGrid> robotMaps, std::vector<float> transform)
{
    ROS_INFO("\n\n\n start map merge");
    int size=robotMaps.size();
    float resolution=robotMaps.front().info.resolution;
    std::vector<nav_msgs::OccupancyGrid>::iterator it;
    int count=0;
    for(it=robotMaps.begin(); it!=robotMaps.end(); it++)
    {
        ROS_INFO("start merging map of robot%d", count+1);
        nav_msgs::OccupancyGrid it_map = *it;
        int x_leftdown = (transform[2*count]   + it_map.info.origin.position.x) / resolution;//像素坐标
        int y_leftdown = (transform[2*count+1] + it_map.info.origin.position.y) / resolution;//像素坐标
        int row_start  = (mergedMap.info.height/2 + y_leftdown) * mergedMap.info.width + (mergedMap.info.width/2 + x_leftdown);//每行起始坐标
        int row_end=row_start+it_map.info.width-1;//每行停止坐标
        ROS_INFO("calucate the relative location of submap robot%d", count+1);
        for(int j=0;j<it_map.info.height;j++)//对小地图的每一列
        {
            for (int i=row_start;i<=row_end;i++)//对小地图的每一行对应的大地图中的位置
            {
                int k=j*it_map.info.width+i-row_start;
                mergedMap.data[i]= costmap_pixel_merge(mergedMap.data[i], it_map.data[k]);
            }
            row_start=row_start+mergedMap.info.width;
            row_end=row_start+it_map.info.width-1;
        }
        count++;
    }
    mergedMap.header.stamp = ros::Time::now(); 
    ROS_INFO("map merge end");
 //return mergedMap;
}



int main(int argc, char **argv){

    //rviz 显示map1,map2,map_merge
    ros::init(argc,argv,"map_merge_costmap");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<nav_msgs::OccupancyGrid>("/map_merge/costmap",10);
    ros::Rate loop_rate(2);

    // prefetch the param
    std::string ns;
    ns=ros::this_node::getName();

    ros::Subscriber sub1= n.subscribe("/robot1/move_base/global_costmap/costmap", 10 ,mapCallBack1);	
    float robot1_x, robot1_y; 
    ros::param::param<float>(ns + "/robot1_x", robot1_x, 0.0);
    ros::param::param<float>(ns + "/robot1_y", robot1_y, 0.0); 
    ROS_INFO("robot1_location: (%3.1f, %3.1f)", robot1_x, robot1_y);

    #ifdef SECOND_ROBOT 
    ros::Subscriber sub2= n.subscribe("/robot2/move_base/global_costmap/costmap", 10 ,mapCallBack2);
    float robot2_x, robot2_y;
    ros::param::param<float>(ns + "/robot2_x", robot2_x, 0.0); 
    ros::param::param<float>(ns + "/robot2_y", robot2_y, -0.8);\
    ROS_INFO("robot2_location: (%3.1f, %3.1f)", robot2_x, robot2_y);
    #endif

    #ifdef THIRD_ROBOT 
    ros::Subscriber sub3= n.subscribe("/robot3/move_base_node/global_costmap/costmap", 10 ,mapCallBack3);
    float  robot3_x, robot3_y;
    ros::param::param<float>(ns + "/robot3_x", robot3_x, 0.0);
    ros::param::param<float>(ns + "/robot3_y", robot3_y, 0.8); 
    ROS_INFO("robot3_location: (%3.1f, %3.1f)", robot3_x, robot3_y);
    #endif



    nav_msgs::OccupancyGrid mergedMap;
    nav_msgs::OccupancyGrid blankMap;
    mergedMap.header.frame_id="robot1/map";
    mergedMap.header.stamp = ros::Time::now(); 

    // mergedMap.info.resolution = mapData1.info.resolution;  
    float resolution, map_width, map_height;
    ros::param::param<float>(ns + "/resolution", resolution, 0.05);
    ros::param::param<float>(ns + "/map_width" , map_width , 3000);
    ros::param::param<float>(ns + "/map_height", map_height, 3000);
    mergedMap.info.resolution = resolution;         // float32   // rosparam
    mergedMap.info.width      = map_width;        //  uint32   // rosparam
    mergedMap.info.height     = map_height;                     // rosparam
    // int p[mergedMap.info.width*mergedMap.info.height];
    
    // for(int i=0;i<mergedMap.info.width*mergedMap.info.height;i++)
    // {
    // p[i]=-1;
    // }
    // std::vector<signed char> a(p, p+mergedMap.info.width*mergedMap.info.height);
    // mergedMap.data = a;
    mergedMap.data.resize(map_height*map_width);
    for(int i=0;i<mergedMap.info.width*mergedMap.info.height;i++)
    {
        mergedMap.data[i]=-1;
    }
    mergedMap.info.origin.position.x=(-1.0)*mergedMap.info.width*mergedMap.info.resolution/2.0;
    mergedMap.info.origin.position.y=(-1.0)*mergedMap.info.height*mergedMap.info.resolution/2.0;
    blankMap = mergedMap;
    std::vector<float> transform;

    transform.push_back(robot1_x);
    transform.push_back(robot1_y);

    #ifdef SECOND_ROBOT 
    transform.push_back(robot2_x);
    transform.push_back(robot2_y);
    #endif

    #ifdef THIRD_ROBOT 
    transform.push_back(robot3_x);
    transform.push_back(robot3_y);   // 缺少旋转。
    #endif

    while(ros::ok())
    {

        mergedMap.data = blankMap.data;
        ros::spinOnce();
        std::vector<nav_msgs::OccupancyGrid> robotMaps;

        robotMaps.push_back(mapData1);

        #ifdef SECOND_ROBOT 
        robotMaps.push_back(mapData2);
        #endif

        #ifdef THIRD_ROBOT 
        robotMaps.push_back(mapData3);     
        #endif

        #ifdef THIRD_ROBOT 
        if(mapData1.data.size()<1 || mapData2.data.size()<1 || mapData3.data.size()<1){continue;} 
        #elif defined SECOND_ROBOT
        if(mapData1.data.size()<1 || mapData2.data.size()<1 ){continue;}
        #else
        if(mapData1.data.size()<1){continue;}
        #endif

        // #ifdef SINGLE_ROBOT
        // mergedMap = mapData1;
        ROS_INFO("merge start robot");
        // #else
        merge(mergedMap, robotMaps, transform);
        // #endif
        pub.publish(mergedMap);
        loop_rate.sleep();
    }

    return 0;
}


