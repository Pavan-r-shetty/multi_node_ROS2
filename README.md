# multi_node_ROS2_SLAM
This repository showcases implementation of a mutli-node system in ROS2 using Perception algorithm like ORBSLAM3, Stereo Visual Odometry. It has 3 nodes. Image loader node, Visual odometry node, Point cloud stitcher node. 


https://www.youtube.com/watch?v=AMlaEFaKG88 —Ubuntu using WSL
To install ROS 2 on Ubuntu running through WSL (Windows Subsystem for Linux), follow these steps:
Step 1: Set up WSL for GUI (Optional but Recommended)
Since ROS 2 uses GUI tools (e.g., Rviz), it’s helpful to have a display server set up in WSL. You can use VcXsrv or X410 on Windows to handle GUI.
Install VcXsrv:
Download and install VcXsrv from here.
Launch VcXsrv and configure it for "Multiple windows" mode.
Configure WSL: In your WSL terminal, set the display environment variable by adding this line to your ~/.bashrc or ~/.zshrc file:
bash
Copy code
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
source ~/.bashrc


Step 2: Install ROS 2 in WSL
Now, proceed with the installation of ROS 2.
1. Set up the locale
Configure your Ubuntu locale settings to UTF-8 (which is required by ROS 2):
bash
Copy code
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

2. Add the ROS 2 apt repository
First, install some necessary dependencies:
bash
Copy code
sudo apt update && sudo apt install curl gnupg lsb-release

Add the ROS 2 repository key:
bash
Copy code
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/trusted.gpg.d/ros.asc

Now, add the ROS 2 repository to your Ubuntu sources list:
bash
Copy code
sudo sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

3. Install ROS 2
Update your package index and install ROS 2. You can install different versions of ROS 2, such as foxy, galactic, or humble. Here’s an example of installing ROS 2 Humble:
bash
Copy code
sudo apt update
sudo apt install ros-humble-desktop

If you want a lightweight installation (without visualization tools), you can install the ros-humble-base package instead of ros-humble-desktop.
4. Set up the ROS 2 environment
Source the ROS 2 setup script to ensure your environment is configured properly:
bash
Copy code
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

5. Install additional ROS 2 dependencies
ROS 2 packages may have additional dependencies that are not installed by default. You can use rosdep to install them:
bash
Copy code
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

Step 3: Testing the ROS 2 Installation
Launch a ROS 2 node to verify that the installation works:
bash
Copy code
ros2 run demo_nodes_cpp talker
To see VcXsrv in action with your WSL (Ubuntu) setup, follow these steps to run a GUI application such as Rviz or any graphical ROS 2 tool.
Step 1: Install VcXsrv
If you haven't installed VcXsrv yet, follow these instructions:
Download VcXsrv from here.
Install VcXsrv by running the installer and following the prompts.
Step 2: Launch VcXsrv
Launch VcXsrv:
Search for XLaunch in your Windows Start Menu.
When the XLaunch window opens, select the following:
Multiple windows
Start no client
Check the box for Disable access control (this allows connections from WSL).
Click Finish to start the X server.
Step 3: Set Up WSL for VcXsrv
In your WSL Ubuntu terminal, make sure your display environment variable is correctly set:
Open your terminal and add the following line to your ~/.bashrc (or ~/.zshrc if you use Zsh):
bash
Copy code
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
source ~/.bashrc
Then, source the file to apply the changes immediately:
bash
Copy code
source ~/.bashrc


This will set the DISPLAY environment variable, which tells Linux applications where to render the GUI (in this case, to VcXsrv on your Windows system).
Step 4: Install a GUI Program (Example: Rviz from ROS 2)
If you have already installed ROS 2 desktop version (which includes Rviz), you can now try running Rviz. If not, follow the steps above to install ROS 2 desktop.
Once you have ROS 2 installed, test VcXsrv with the following:
Run Rviz in WSL Ubuntu:
bash
Copy code
rviz2
If everything is set up correctly, Rviz should now open in a new window on your Windows desktop.
Step 5: Try Another GUI Program (Optional)
You can also try other GUI programs to ensure VcXsrv is working properly. For example:
Install a basic GUI app like xclock in WSL:
bash
Copy code
sudo apt install x11-apps


Run xclock to test the X server:
bash
Copy code
xclock
If the clock application appears on your Windows desktop, VcXsrv is working!


Node 1: Image Loader Node
Objective:
Load and publish left and depth images from directories.
Setup the Node: Start by creating a ROS2 node that will read images sequentially from two directories: one for left camera images and one for depth images.
Image Loading: Use OpenCV’s cv::imread() to load the images. Assume they are sequentially numbered, so you can iterate over the filenames.
Convert to ROS2 Messages: Utilize cv_bridge to convert OpenCV images to ROS2 sensor messages. Convert them into sensor_msgs::Image format.
Publish the Images: Publish the left camera images to /camera/left/image_raw and the depth images to /camera/depth/image_raw.
Publishing Rate: Publish the images at a fixed rate using ROS2’s rclcpp::Rate or a timer.
Basic structure for the node:
cv::imread() for reading images.
cv_bridge::CvImage for converting OpenCV images to ROS2 images.
rclcpp::Publisher for publishing messages.
Node 2: Visual Odometry Node (ORB-SLAM3 Integration)
Objective:
Integrate ORB-SLAM3 for visual odometry.
Setup ORB-SLAM3: Download and integrate ORB-SLAM3 into your ROS2 workspace. Modify it so it can be wrapped into a ROS2 node.
Subscribe to Image Topics: The node should subscribe to /camera/left/image_raw and /camera/depth/image_raw topics. You will use these images for visual odometry. You will need to use some kind of synchronization so that the right depth and left pair are subscribed. I have used a custom function with the help of buffers since I had trouble integrating sync policy using message filters 
Run Visual Odometry: Feed the images into ORB-SLAM3’s visual odometry system to estimate the robot’s position and orientation.
Publish Pose: After obtaining the pose estimates from ORB-SLAM3, publish the robot’s pose as geometry_msgs::PoseStamped messages to /robot/pose.
Tips: You will likely need to modify ORB-SLAM3 to ensure it integrates properly with ROS2. Carefully ensure that the pose output is in the correct ROS2 format.
Node 3: Point Cloud Stitcher & Octomap Generator Node
Objective:
Construct a 3D map using depth images and pose data.
Subscribe to Depth and Pose: This node subscribes to the depth images from /camera/depth/image_raw and the pose estimates from /robot/pose. Use synchronization like the second node.
Unproject Depth Images: Convert the depth images into 3D point clouds. This involves transforming 2D depth data into 3D points using the camera’s intrinsics.
Accumulate Point Clouds: For every N frames, accumulate the point clouds, transforming them into the correct world frame using the pose estimates.
Stitch Point Clouds: Stitch the point clouds together to create a denser, larger point cloud representing the environment.
Generate Octomap: Convert the stitched point cloud into an Octomap (or similar voxelized map). You can use the Octomap library in ROS2 for this purpose.
Display the Map: Use Pangolin (or another visualizer) to display the 3D map.
You will need to use optimization techniques like:
Downsample the Point Cloud: Use a voxel grid filter to reduce the point cloud size.
Asynchronous Callbacks: Process incoming messages without blocking.
Adjust OctoMap Updates: Update OctoMap only at intervals, reducing the frequency of updates.
Use a Multi-threaded ROS Executor: To ensure smooth execution of different callbacks.
Integrate Open3d for Fast and Efficient Implementation
Tips:
Use the pcl library to handle point cloud operations.
The Octomap ROS2 package can be used for converting point clouds into a voxel-based representation.


Tools
OpenCV: For image handling.
ORB-SLAM3: For visual odometry.
Pangolin: For visualization.
Octomap: For 3D mapping.
Open3d: for 3D data processing


NODE 1:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake image_loader_node --dependencies rclcpp sensor_msgs cv_bridge image_transport
Create a C++ file in the src folder called image_loader_node.cpp and start implementing the node logic.

Build the Package:

cd ~/ros2_ws
colcon build --packages-select image_loader_node
source ~/ros2_ws/install/setup.bash




Testing the Node
Ensure that you have ROS2 topics /camera/left/image_raw and /camera/depth/image_raw being published.
cd /ros2_ws/src/image_loader_node/src
 ros2 run image_loader_node image_loader_node --ros-args -p left_image_dir:=/home/shetty/ros2_ws/rainier_labs/assignmentData/left/ -p depth_image_dir:=/home/shetty/ros2_ws/rainier_labs/assignmentData/depth/
(Pass appropriate paths)


Use rqt_image_view to verify if the images are being published correctly:
bash
Copy code
ros2 run rqt_image_view rqt_image_view






NODE 2:


sudo apt-get install libopencv-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-dev
sudo apt-get install libopencv-contrib-dev
sudo apt-get install libpcl-dev
sudo apt-get install libgl1-mesa-dev

sudo apt install ros-foxy-cv-bridge ros-foxy-rclcpp ros-foxy-geometry-msgs

cd ~/ros2_ws/src git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git

git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
sudo make install
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake visual_odometry_node --dependencies rclcpp sensor_msgs geometry_msgs cv_bridge
cd ~/ros2_ws
colcon build --packages-select visual_odometry_node

There will be so many errors related to ORBSLAM3 packages (OBSLAM3, Pangolin, Sophus)
You might need to individually build and provide a reference in CMakelists and xml of visual odometry node. I had to pass the absolute paths of ORBSLAM3 and Pangolin in the CMakeLists of the second node.

I have used the below links as reference to build ORBSLAM3:
https://github.com/UZ-SLAMLab/ORB_SLAM3
https://medium.com/@antonioconsiglio/integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361
https://github.com/kevin-robb/orb_slam_implementation


In ORBSLAM3 directory pass yaml file related to your camera as: ORB_SLAM3/Examples/RGB-D/custom_camera.yaml (It takes a few trial and error to get the best ORB parameters that produces the best tracking overall)
Run Node 1: ros2 run image_loader_node image_loader_node --ros-args -p left_image_dir:=/home/shetty/ros2_ws/rainier_labs/assignmentData/left/ -p depth_image_dir:=/home/shetty/ros2_ws/rainier_labs/assignmentData/depth/

Run Node 2: ros2 run visual_odometry_node visual_odometry_node 




NODE 3:


cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake point_cloud_stitcher_node --dependencies rclcpp sensor_msgs geometry_msgs cv_bridge
Create a node point_cloud_stitcher_node.cpp 
In src you will need to build open3d for 3D data processing
# Install dependencies 
sudo apt update 
sudo apt install -y git cmake build-essential libeigen3-dev 
# Clone Open3D repository 
cd ~ 
git clone --recursive https://github.com/isl-org/Open3D.git
cd Open3D
 # Build Open3D 
mkdir build 
cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local .. 
make -j$(nproc)
sudo make install
cd ~/ros2_ws
colcon build --packages-select point_cloud_stitcher_node






Now to Launch all the nodes together create a launch package
cd ~/ros2_ws/src
ros2 pkg create my_launch_package
cd ~/ros2_ws/src/my_launch_package 
mkdir -p launch
Create a file named launch_nodes_in_sequence.launch.py
cd ~/ros2_ws 
colcon build --packages-select my_launch_package

Now run the file using ros2 launch my_launch_package launch_nodes_in_sequence.launch.py












