
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

- Ensure that the correct data is passed into Node 1 when running it independently, or update the launch file with the appropriate dataset path. I used the straight section for testing where ORBSLAM3 tracking performed relatively better
- The overall performance heavily relies on ORBSLAM3's ability to detect features and maintain consistent local map tracking. 
- I tested it on the straight section where the tracking is better compared to other sections, although the performance is still suboptimal in an absolute sense.
## Challenges in Node 2

- Proper synchronization is needed to ensure the correct depth and left image pair is subscribed. 
- I implemented a custom function using buffers due to difficulties in integrating sync policy with message filters.
- There might be challenges in building ORBSLAM3; refer to the links in the Acknowledgements section for assistance.
- To integrate ORBSLAM3 with Node 2, make sure to pass the absolute paths for ORBSLAM3 and Pangolin in the CMakeLists. 
- Pangolin should be built separately, along with OpenCV.


## Challenges in Node 3

- Point Cloud Downsampling: Apply a voxel grid filter to reduce the size of the point cloud for better processing efficiency.
- Asynchronous Callbacks: Ensure that incoming messages are processed asynchronously to avoid blocking other tasks.
- Optimizing OctoMap Updates: Limit the frequency of OctoMap updates by performing them at set intervals, improving overall performance.
- Multi-threaded ROS Executor: Use a multi-threaded ROS executor to handle multiple callbacks simultaneously, ensuring smooth execution.
- Open3D Integration: Integrate Open3D for faster and more efficient 3D map implementations.


## ORBSLAM3 

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/orb.JPG)

## ORBSLAM3 feature mapping view

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/orbcurrent.JPG)

## Point Cloud Pangolin view 

![App Screenshot](https://github.com/Pavan-r-shetty/multi_node_ROS2/blob/main/pcl.JPG)

# nms-web
https://drive.google.com/file/d/1WPAPif6z77XQIE60n0eZV3JEE_J1i6OP/view?usp=sharing 

