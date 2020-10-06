# Overview
This repo consists of:
- icp_registration: merge cartographer submaps using ICP method
- m-explore: multiple robots' maps merge only based on submaps without using images.
- cartographer_map_discrete: nodes to convert continous value into discrete value.

Before running this repo this repo could only run if and only if the cartographer node offers the correct output map name.  

## Preparation
Please reach me to get the gem.pth file.

## interface modification

To merge cartographer submaps from two robots, follow the following steps:

### merge submaps using image from camera

luanch file /m-explore/map_merge/xtark_mapping_multi_sim_gem_jianming.launch

1. change line 4 of <...>/m-explore/map_merge/launch/xtark_mapping_multi_sim_gem_jianming.launch to <...>/m-explore/map_merge/include/weights/gem.pth
2. change line 46: robot_map_topic value to the name of cartographer output map.
3. change line 48: merged_map_topic value to the expected suffix name of output map. e.g. original value is "map" -- name of output map is "robot1/map".
4. change the map topic of cartographer outputs to "robot?/cartographerMap"
5. change line 4 of <...>/m-explore/map_merge/include/weights/gem.pth  to <...>/m-explore/map_merge/include/weights/gem.pth
6. change line 5 of <...>/m-explore/map_merge/launch/xtark_mapping_2robot_sim_wo_gem.launch to  <...>/m-explore/map_merge/include/xtark_mapping_multi_sim_gem_jianming.launch
7. change line 10 of <...>/m-explore/map_merge/launch/xtark_mapping_2robot_sim_wo_gem.launch to  <...>/m-explore/map_merge/include/xtark_mapping_multi_sim_gem_jianming.launch

```
roslaunch multirobot_map_merge xtark_mapping_2robot_sim.launch
```


### merge submaps only using cartographer submaps


luanch file /m-explore/map_merge/xtark_mapping_multi_sim_wo_gem.launch

1. change line 4 of <...>/m-explore/map_merge/launch/xtark_mapping_multi_sim_wo_gem.launch to <...>/m-explore/map_merge/include/weights/gem.pth
2. change line 36: robot_map_topic value to the name of cartographer output map.
3. change line 37: merged_map_topic value to the expected suffix name of output map. e.g. original value is "map" -- name of output map is "robot1/map".
4. change the map topic of cartographer outputs to "robot?/cartographerMap"
5. change line 4 of <...>/m-explore/map_merge/include/weights/gem.pth  to <...>/m-explore/map_merge/include/weights/gem.pth
6. change line 5 of <...>/m-explore/map_merge/launch/xtark_mapping_2robot_sim_wo_gem.launch to  <...>/m-explore/map_merge/include/xtark_mapping_multi_sim_wo_gem.launch
7. change line 10 of <...>/m-explore/map_merge/launch/xtark_mapping_2robot_sim_wo_gem.launch to  <...>/m-explore/map_merge/include/xtark_mapping_multi_sim_wo_gem.launch

```
roslaunch multirobot_map_merge xtark_mapping_2robot_sim_wo_gem.launch
```



Have Fun :)
