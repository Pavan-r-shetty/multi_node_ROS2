
# multi_node_ROS2_SLAM
This repository showcases implementation of a mutli-node system in ROS2 using Perception algorithm like ORBSLAM3, Stereo Visual Odometry. It has 3 nodes. Image loader node, Visual odometry node, Point cloud stitcher node. 



## Acknowledgements

 - [Ubuntu using WSL](https://www.youtube.com/watch?v=AMlaEFaKG88)
 - [ORBSLAM3 official repo](https://github.com/UZ-SLAMLab/ORB_SLAM3)
 - [ORBSLAM3 on Raspberry Pi](https://medium.com/@antonioconsiglio/integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361)
- [ORBSLAM3 demo implementation](https://github.com/kevin-robb/orb_slam_implementation)

## Build Node 1 - Image Loader node

```bash
  cd ~/ros2_ws/
  colcon build --packages-select image_loader_node

```
## Build Node 2 - Visual Odometry node

```bash
  cd ~/ros2_ws/
  colcon build --packages-select visual_odometry_node

```
## Build Node 3 - Point Cloud Stitcher node

```bash
  cd ~/ros2_ws/
  colcon build --packages-select point_cloud_stitcher_node

```
## Run Node 1 - Image Loader node

```bash
  cd ~/ros2_ws/
  ros2 run image_loader_node image_loader_node --ros-args -p left_image_dir:=/home/shetty/ ros2_ws/rainier_labs/assignmentData/left/ -p depth_image_dir:=/home/shetty/ros2_ws/rainier_labs/assignmentData/depth/

```
## Run Node 2 - Visual Odometry node

```bash
  cd ~/ros2_ws/
  ros2 run visual_odometry_node visual_odometry_node

```
## Run Node 3 - Point Cloud Stitcher node

```bash
  cd ~/ros2_ws/
  ros2 run point_cloud_stitcher_node point_cloud_stitcher_node

```

## Deployment

To deploy this project, run the launch file to launch all the 3 nodes

```bash
  cd ~/ros2_ws/
  ros2 launch my_launch_package launch_nodes_in_sequence.launch.py

```

## To View Octomap

Use the octomap.bt and visualize it in octavis package

```bash
  cd ~/ros2_ws/    (directory where you have your octomap.bt)
  octovis octomap.bt

```


## Note

Pass appropriate data in Node 1 when running just node 1 or edit the launch file with the dataset path. I have used straight section where ORBSLAM3 tracking was relatively better. 
## Challenges in Node 2

- You will need to use some kind of synchronization so that the correct depth and left pair are subscribed. 
- I have used a custom function with the help of buffers since I had trouble integrating sync policy using message filters 
- ORBSLAM3 build challenge, use the help of links from Acknowledgement section
- Integrate ORBSLAM3 with node 2 by passing absolute paths of ORBSLAM3 and Pangolin in CMakeLists 
- Pangolin needs to be built seperately along with OpenCV


## Challenges in Node 3

- Downsample the Point Cloud: Use a voxel grid filter to reduce the point cloud size.
- Asynchronous Callbacks: Process incoming messages without blocking.
- Adjust OctoMap Updates: Update OctoMap only at intervals, reducing the frequency of updates.
- Use a Multi-threaded ROS Executor: To ensure smooth execution of different callbacks.
- Integrate Open3d for Fast and Efficient 3D map Implementation


## ORBSLAM3 

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/orb.JPG)

## ORBSLAM3 feature mapping view

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/orbcurrent.JPG)

## Point Cloud Pangolin view 

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/pcl.JPG)
