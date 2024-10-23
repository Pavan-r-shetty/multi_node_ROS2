// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/io/pcd_io.h"
// #include "pcl_ros/transforms.hpp"
// #include "pcl/visualization/cloud_viewer.h"
// #include "octomap/octomap.h"
// #include "octomap/ColorOcTree.h"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <mutex>
// #include <thread>
// #include <pangolin/pangolin.h>
// #include <vector>
// #include <cmath>

// class PointCloudStitcherNode : public rclcpp::Node
// {
// public:
//     PointCloudStitcherNode() : Node("point_cloud_stitcher_node"), pose_initialized_(false)
//     {
//         // Subscribers
//         depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/depth/image_raw", 10, std::bind(&PointCloudStitcherNode::depthImageCallback, this, std::placeholders::_1));
//         pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/robot/pose", 10, std::bind(&PointCloudStitcherNode::poseCallback, this, std::placeholders::_1));

//         // Initialize Octomap
//         octree_ = std::make_shared<octomap::ColorOcTree>(0.1);  // Set resolution to 0.1m

//         // Timer for saving Octomap
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(50), std::bind(&PointCloudStitcherNode::saveOctomap, this));

//         // Buffers for synchronization
//         depth_image_buffer_.reserve(10);
//         pose_buffer_.reserve(10);

//         // Run Pangolin in a separate thread for 3D visualization
//         pangolin_thread_ = std::thread(&PointCloudStitcherNode::runPangolinVisualizer, this);
//     }

//     ~PointCloudStitcherNode()
//     {
//         if (pangolin_thread_.joinable()) {
//             pangolin_thread_.join();
//         }
//     }

// private:
//     // Buffers to store messages temporarily
//     std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;
//     std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

//     std::mutex cloud_mutex_;

//     // Pose initialization flag
//     bool pose_initialized_;

//     // Current pose
//     geometry_msgs::msg::PoseStamped current_pose_;

//     // Octomap object
//     std::shared_ptr<octomap::ColorOcTree> octree_;

//     // Accumulated point cloud
//     pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;

//     // Camera intrinsics
//     float fx_ = 348.925;
//     float fy_ = 351.135;
//     float cx_ = 339.075;
//     float cy_ = 177.45;
//     float depth_scale_ = 0.001;

//     // Callback for depth image
//     void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         if (depth_image_buffer_.size() >= 10)
//         {
//             depth_image_buffer_.erase(depth_image_buffer_.begin()); // Maintain queue size
//         }
//         depth_image_buffer_.push_back(msg);

//         // Try to find a matching pose
//         matchAndPublish();
//     }

//     // Callback for pose
//     void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//     {
//         if (pose_buffer_.size() >= 10)
//         {
//             pose_buffer_.erase(pose_buffer_.begin()); // Maintain queue size
//         }
//         pose_buffer_.push_back(msg);

//         // Try to find a matching depth image
//         matchAndPublish();
//     }

//     // Matching function for synchronizing depth image and pose based on timestamps
//     void matchAndPublish()
//     {
//         if (pose_buffer_.empty() || depth_image_buffer_.empty())
//         {
//             return;
//         }

//         auto pose = pose_buffer_.front();
//         rclcpp::Time pose_time(pose->header.stamp);

//         // Find the depth image with the closest timestamp to the pose
//         sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
//         rclcpp::Time closest_time;
//         double min_time_diff = std::numeric_limits<double>::max();

//         for (const auto &depth_image : depth_image_buffer_)
//         {
//             rclcpp::Time depth_time(depth_image->header.stamp);
//             double time_diff = std::abs((pose_time - depth_time).seconds());

//             if (time_diff < min_time_diff)
//             {
//                 min_time_diff = time_diff;
//                 closest_depth_image = depth_image;
//                 closest_time = depth_time;
//             }
//         }

//         // If matching pair found, process the data
//         if (closest_depth_image != nullptr)
//             // RCLCPP_INFO(this->get_logger(), "Processing....");

//         {
//             processDepthImageAndPose(closest_depth_image, pose);
//         }
//     }

//     // Process the matched depth image and pose
//     void processDepthImageAndPose(const sensor_msgs::msg::Image::SharedPtr &depth_image, const geometry_msgs::msg::PoseStamped::SharedPtr &pose)
//     {
//         // RCLCPP_INFO(this->get_logger(), "Processing synchronized depth image and pose.");

//         // Convert depth image to OpenCV format
//         cv::Mat depth_cv_image = cv_bridge::toCvCopy(depth_image, "mono16")->image;

//         if (depth_cv_image.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "Depth image is empty.");
//             return;
//         }

//         // Unproject depth image into 3D point cloud
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         unprojectDepthImageToPointCloud(depth_cv_image, cloud);

//         // Transform point cloud to world frame using current pose
//         Eigen::Affine3d transform = poseToTransform(*pose);
//         pcl::transformPointCloud(*cloud, *cloud, transform);

//         // Lock the cloud while updating it to ensure thread safety
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         // Accumulate point clouds
//         accumulated_cloud_ += *cloud;
//     }

//     // Unproject depth image to 3D point cloud
//     void unprojectDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//     {
//         for (int v = 0; v < depth_image.rows; ++v)
//         {
//             for (int u = 0; u < depth_image.cols; ++u)
//             {
//                 uint16_t depth = depth_image.at<uint16_t>(v, u);
//                 if (depth > 0)
//                 {
//                     float z = depth * depth_scale_;
//                     float x = (u - cx_) * z / fx_;
//                     float y = (v - cy_) * z / fy_;
//                     cloud->points.push_back(pcl::PointXYZ(x, y, z));
//                 }
//             }
//         }
//         cloud->width = cloud->points.size();
//         cloud->height = 1;
//         cloud->is_dense = false;
//     }

//     // Convert pose to Eigen transform
//     Eigen::Affine3d poseToTransform(const geometry_msgs::msg::PoseStamped &pose)
//     {
//         Eigen::Translation3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
//         Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
//         return translation * rotation;
//     }

//     // Save the accumulated point cloud to Octomap
//     void saveOctomap()
//     {
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         if (accumulated_cloud_.points.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "No points in accumulated cloud.");
//             return;
//         }

//         for (const auto &point : accumulated_cloud_.points)
//         {
//             octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
//         }

//         octree_->write("octomap.bt");
//         RCLCPP_INFO(this->get_logger(), "Octomap saved.");
//     }

//     // Run Pangolin visualization with zoom functionality
//     void runPangolinVisualizer()
//     {
//         pangolin::CreateWindowAndBind("Octomap Viewer", 640, 480);
//         glEnable(GL_DEPTH_TEST);

//         // Camera setup with zoom functionality
//         pangolin::OpenGlRenderState s_cam(
//             pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 1000),  // Adjust far clipping plane to allow more zooming out
//             pangolin::ModelViewLookAt(0, -1, -10, 0, 0, 0, pangolin::AxisZ)        // Set initial zoom distance
//         );

//         // Create display and set up handler with zoom capability
//         pangolin::View &d_cam = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
//             .SetHandler(new pangolin::Handler3D(s_cam));  // Enable mouse control including zoom

//         while (!pangolin::ShouldQuit())
//         {
//             glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//             d_cam.Activate(s_cam);

//             renderOctomap();

//             pangolin::FinishFrame();
//         }
//     }

//     // Render the Octomap in Pangolin
//     void renderOctomap()
//     {
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         glColor3f(1.0, 0.0, 0.0); // Red color for visual distinction

//         // Render the Octomap using the octree
//         for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
//         {
//             if (octree_->isNodeOccupied(*it))
//             {
//                 const octomap::point3d &coord = it.getCoordinate();
//                 glPushMatrix();
//                 glTranslatef(coord.x(), coord.y(), coord.z());
//                 pangolin::glDrawColouredCube();
//                 glPopMatrix();
//             }
//         }
//     }

//     // Subscribers
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

//     // Timer for saving Octomap
//     rclcpp::TimerBase::SharedPtr timer_;

//     // Thread for running Pangolin
//     std::thread pangolin_thread_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointCloudStitcherNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/io/pcd_io.h"
// #include "pcl_ros/transforms.hpp"
// #include "octomap/octomap.h"
// #include "octomap/ColorOcTree.h"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <mutex>
// #include <chrono>
// #include <thread>

// class PointCloudStitcherNode : public rclcpp::Node
// {
// public:
//     PointCloudStitcherNode() : Node("point_cloud_stitcher_node"), pose_initialized_(false), batch_size_(5), batch_count_(0)
//     {
//         // Subscribers
//         depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/depth/image_raw", 10, std::bind(&PointCloudStitcherNode::depthImageCallback, this, std::placeholders::_1));
//         pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/robot/pose", 10, std::bind(&PointCloudStitcherNode::poseCallback, this, std::placeholders::_1));

//         // Initialize Octomap
//         octree_ = std::make_shared<octomap::ColorOcTree>(1);  // Set resolution to 0.1m

//         // Timer to periodically save Octomap
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(10), std::bind(&PointCloudStitcherNode::saveOctomap, this));
//     }

// private:
//     // Buffers to store messages temporarily
//     std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;
//     std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

//     std::mutex cloud_mutex_;

//     // Pose initialization flag
//     bool pose_initialized_;
//     int batch_size_;   // Number of messages to batch before processing
//     int batch_count_;  // Current batch count

//     // Current pose
//     geometry_msgs::msg::PoseStamped current_pose_;

//     // Octomap object
//     std::shared_ptr<octomap::ColorOcTree> octree_;

//     // Accumulated point cloud
//     pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;

//     // Camera intrinsics
//     float fx_ = 348.925;
//     float fy_ = 351.135;
//     float cx_ = 339.075;
//     float cy_ = 177.45;
//     float depth_scale_ = 0.001;

//     // Floor level and threshold (z-axis considered as "up")
//     float floor_level_ = 0.0;  // Define the floor at z = 0
//     float height_limit_ = 1.0; // Limit points within 1 meter of the floor

//     // Callback for depth image
//     void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         if (depth_image_buffer_.size() >= 10)
//         {
//             depth_image_buffer_.erase(depth_image_buffer_.begin()); // Maintain queue size
//         }
//         depth_image_buffer_.push_back(msg);

//         // Try to find a matching pose
//         if (++batch_count_ >= batch_size_)
//         {
//             batch_count_ = 0;
//             matchAndPublish();
//         }
//     }

//     // Callback for pose
//     void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//     {
//         if (pose_buffer_.size() >= 10)
//         {
//             pose_buffer_.erase(pose_buffer_.begin()); // Maintain queue size
//         }
//         pose_buffer_.push_back(msg);

//         // Try to find a matching depth image
//         if (++batch_count_ >= batch_size_)
//         {
//             batch_count_ = 0;
//             matchAndPublish();
//         }
//     }

//     // Matching function for synchronizing depth image and pose based on timestamps
//     void matchAndPublish()
//     {
//         if (pose_buffer_.empty() || depth_image_buffer_.empty())
//         {
//             return;
//         }

//         auto pose = pose_buffer_.front();
//         rclcpp::Time pose_time(pose->header.stamp);

//         // Find the depth image with the closest timestamp to the pose
//         sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
//         rclcpp::Time closest_time;
//         double min_time_diff = std::numeric_limits<double>::max();

//         for (const auto &depth_image : depth_image_buffer_)
//         {
//             rclcpp::Time depth_time(depth_image->header.stamp);
//             double time_diff = std::abs((pose_time - depth_time).seconds());

//             if (time_diff < min_time_diff)
//             {
//                 min_time_diff = time_diff;
//                 closest_depth_image = depth_image;
//                 closest_time = depth_time;
//             }
//         }

//         // If matching pair found, process the data
//         if (closest_depth_image != nullptr)
//         {
//             processDepthImageAndPose(closest_depth_image, pose);
//         }
//     }

//     // Process the matched depth image and pose
//     void processDepthImageAndPose(const sensor_msgs::msg::Image::SharedPtr &depth_image, const geometry_msgs::msg::PoseStamped::SharedPtr &pose)
//     {
//         // Convert depth image to OpenCV format
//         cv::Mat depth_cv_image = cv_bridge::toCvCopy(depth_image, "mono16")->image;

//         if (depth_cv_image.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "Depth image is empty.");
//             return;
//         }

//         // Unproject depth image into 3D point cloud
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         unprojectDepthImageToPointCloud(depth_cv_image, cloud);

//         // Transform point cloud to world frame using current pose
//         Eigen::Affine3d transform = poseToTransform(*pose);
//         pcl::transformPointCloud(*cloud, *cloud, transform);

//         // Lock the cloud while updating it to ensure thread safety
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         // Accumulate point clouds within 1 meter of the floor
//         for (const auto &point : cloud->points)
//         {
//             if (point.z >= floor_level_ && point.z <= (floor_level_ + height_limit_))
//             {
//                 accumulated_cloud_.points.push_back(point);
//             }
//         }
//     }

//     // Unproject depth image to 3D point cloud
//     void unprojectDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//     {
//         for (int v = 0; v < depth_image.rows; ++v)
//         {
//             for (int u = 0; u < depth_image.cols; ++u)
//             {
//                 uint16_t depth = depth_image.at<uint16_t>(v, u);
//                 if (depth > 0)
//                 {
//                     float z = depth * depth_scale_;
//                     float x = (u - cx_) * z / fx_;
//                     float y = (v - cy_) * z / fy_;
//                     cloud->points.push_back(pcl::PointXYZ(x, y, z));
//                 }
//             }
//         }
//         cloud->width = cloud->points.size();
//         cloud->height = 1;
//         cloud->is_dense = false;
//     }

//     // Convert pose to Eigen transform
//     Eigen::Affine3d poseToTransform(const geometry_msgs::msg::PoseStamped &pose)
//     {
//         Eigen::Translation3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
//         Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
//         return translation * rotation;
//     }

//     // Save the accumulated point cloud to Octomap
//     void saveOctomap()
//     {
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         if (accumulated_cloud_.points.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "No points in accumulated cloud.");
//             return;
//         }

//         for (const auto &point : accumulated_cloud_.points)
//         {
//             octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
//         }

//         // Save the Octomap in binary format
//         octree_->writeBinary("octomap.bt");

//         RCLCPP_INFO(this->get_logger(), "Octomap1 saved.");
//     }

//     // Subscribers
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

//     // Timer for periodically saving the Octomap
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointCloudStitcherNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/io/pcd_io.h"
// #include "pcl_ros/transforms.hpp"
// #include "octomap/octomap.h"
// #include "octomap/ColorOcTree.h"
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <mutex>
// #include <thread>
// #include <open3d/Open3D.h>
// #include <vector>
// #include <cmath>

// class PointCloudStitcherNode : public rclcpp::Node
// {
// public:
//     PointCloudStitcherNode() : Node("point_cloud_stitcher_node"), pose_initialized_(false)
//     {
//         // Subscribers
//         depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/depth/image_raw", 10, std::bind(&PointCloudStitcherNode::depthImageCallback, this, std::placeholders::_1));
//         pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/robot/pose", 10, std::bind(&PointCloudStitcherNode::poseCallback, this, std::placeholders::_1));

//         // Initialize Octomap
//         octree_ = std::make_shared<octomap::ColorOcTree>(0.1);  // Set resolution to 0.1m

//         // Timer for saving Octomap
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(50), std::bind(&PointCloudStitcherNode::saveOctomap, this));

//         // Buffers for synchronization
//         depth_image_buffer_.reserve(10);
//         pose_buffer_.reserve(10);

//         // Run Open3D visualization in a separate thread
//         open3d_thread_ = std::thread(&PointCloudStitcherNode::runOpen3DVisualizer, this);
//     }

//     ~PointCloudStitcherNode()
//     {
//         if (open3d_thread_.joinable()) {
//             open3d_thread_.join();
//         }
//     }

// private:
//     // Buffers to store messages temporarily
//     std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;
//     std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

//     std::mutex cloud_mutex_;

//     // Pose initialization flag
//     bool pose_initialized_;

//     // Current pose
//     geometry_msgs::msg::PoseStamped current_pose_;

//     // Octomap object
//     std::shared_ptr<octomap::ColorOcTree> octree_;

//     // Accumulated point cloud
//     pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;

//     // Camera intrinsics
//     float fx_ = 348.925;
//     float fy_ = 351.135;
//     float cx_ = 339.075;
//     float cy_ = 177.45;
//     float depth_scale_ = 0.001;

//     // Callback for depth image
//     void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         if (depth_image_buffer_.size() >= 10)
//         {
//             depth_image_buffer_.erase(depth_image_buffer_.begin()); // Maintain queue size
//         }
//         depth_image_buffer_.push_back(msg);

//         // Try to find a matching pose
//         matchAndPublish();
//     }

//     // Callback for pose
//     void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//     {
//         if (pose_buffer_.size() >= 10)
//         {
//             pose_buffer_.erase(pose_buffer_.begin()); // Maintain queue size
//         }
//         pose_buffer_.push_back(msg);

//         // Try to find a matching depth image
//         matchAndPublish();
//     }

//     // Matching function for synchronizing depth image and pose based on timestamps
//     void matchAndPublish()
//     {
//         if (pose_buffer_.empty() || depth_image_buffer_.empty())
//         {
//             return;
//         }

//         auto pose = pose_buffer_.front();
//         rclcpp::Time pose_time(pose->header.stamp);

//         // Find the depth image with the closest timestamp to the pose
//         sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
//         rclcpp::Time closest_time;
//         double min_time_diff = std::numeric_limits<double>::max();

//         for (const auto &depth_image : depth_image_buffer_)
//         {
//             rclcpp::Time depth_time(depth_image->header.stamp);
//             double time_diff = std::abs((pose_time - depth_time).seconds());

//             if (time_diff < min_time_diff)
//             {
//                 min_time_diff = time_diff;
//                 closest_depth_image = depth_image;
//                 closest_time = depth_time;
//             }
//         }

//         // If matching pair found, process the data
//         if (closest_depth_image != nullptr)
//         {
//             processDepthImageAndPose(closest_depth_image, pose);
//         }
//     }

//     // Process the matched depth image and pose
//     void processDepthImageAndPose(const sensor_msgs::msg::Image::SharedPtr &depth_image, const geometry_msgs::msg::PoseStamped::SharedPtr &pose)
//     {
//         // Convert depth image to OpenCV format
//         cv::Mat depth_cv_image = cv_bridge::toCvCopy(depth_image, "mono16")->image;

//         if (depth_cv_image.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "Depth image is empty.");
//             return;
//         }

//         // Unproject depth image into 3D point cloud
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         unprojectDepthImageToPointCloud(depth_cv_image, cloud);

//         // Transform point cloud to world frame using current pose
//         Eigen::Affine3d transform = poseToTransform(*pose);
//         pcl::transformPointCloud(*cloud, *cloud, transform);

//         // Lock the cloud while updating it to ensure thread safety
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         // Accumulate point clouds
//         accumulated_cloud_ += *cloud;
//     }

//     // Unproject depth image to 3D point cloud
//     void unprojectDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//     {
//         for (int v = 0; v < depth_image.rows; ++v)
//         {
//             for (int u = 0; u < depth_image.cols; ++u)
//             {
//                 uint16_t depth = depth_image.at<uint16_t>(v, u);
//                 if (depth > 0)
//                 {
//                     float z = depth * depth_scale_;
//                     float x = (u - cx_) * z / fx_;
//                     float y = (v - cy_) * z / fy_;
//                     cloud->points.push_back(pcl::PointXYZ(x, y, z));
//                 }
//             }
//         }
//         cloud->width = cloud->points.size();
//         cloud->height = 1;
//         cloud->is_dense = false;
//     }

//     // Convert pose to Eigen transform
//     Eigen::Affine3d poseToTransform(const geometry_msgs::msg::PoseStamped &pose)
//     {
//         Eigen::Translation3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
//         Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
//         return translation * rotation;
//     }

//     // Save the accumulated point cloud to Octomap
//     void saveOctomap()
//     {
//         std::lock_guard<std::mutex> lock(cloud_mutex_);

//         if (accumulated_cloud_.points.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "No points in accumulated cloud.");
//             return;
//         }

//         for (const auto &point : accumulated_cloud_.points)
//         {
//             octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
//         }

//         octree_->write("octomap.bt");
//         RCLCPP_INFO(this->get_logger(), "Octomap saved.");
//     }

//     // Run Open3D visualization
//     void runOpen3DVisualizer()
//     {
//         open3d::visualization::Visualizer visualizer;
//         visualizer.CreateVisualizerWindow("PointCloud Visualizer", 640, 480);
//         open3d::geometry::PointCloud open3d_cloud;

//         while (rclcpp::ok())
//         {
//             {
//                 std::lock_guard<std::mutex> lock(cloud_mutex_);

//                 // Convert PCL point cloud to Open3D point cloud
//                 open3d_cloud.Clear();
//                 for (const auto &point : accumulated_cloud_.points)
//                 {
//                     open3d_cloud.points_.emplace_back(point.x, point.y, point.z);
//                 }
//             }

//             // Update the visualization
//             visualizer.ClearGeometries();
//             visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(open3d_cloud));
//             visualizer.PollEvents();
//             visualizer.UpdateRender();
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }

//         visualizer.DestroyVisualizerWindow();
//     }

//     // Subscribers
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

//     // Timer for saving Octomap
//     rclcpp::TimerBase::SharedPtr timer_;

//     // Thread for running Open3D visualization
//     std::thread open3d_thread_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointCloudStitcherNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_ros/transforms.hpp"
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <open3d/Open3D.h>
#include <vector>
#include <chrono>

class PointCloudStitcherNode : public rclcpp::Node
{
public:
    PointCloudStitcherNode() : Node("point_cloud_stitcher_node"), pose_initialized_(false)
    {
        // Subscribers
        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudStitcherNode::depthImageCallback, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudStitcherNode::poseCallback, this, std::placeholders::_1));

        // Initialize Octomap
        octree_ = std::make_shared<octomap::ColorOcTree>(0.1);  // Set resolution to 0.1m

        // Timer for saving Octomap
        timer_ = this->create_wall_timer(
            std::chrono::seconds(50), std::bind(&PointCloudStitcherNode::saveOctomap, this));

        // Buffers for synchronization
        depth_image_buffer_.reserve(10);
        pose_buffer_.reserve(10);

        // Run Open3D visualization in a separate thread
        open3d_thread_ = std::thread(&PointCloudStitcherNode::runOpen3DVisualizer, this);
    }

    ~PointCloudStitcherNode()
    {
        if (open3d_thread_.joinable()) {
            open3d_thread_.join();
        }
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;
    std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;
    std::mutex cloud_mutex_;
    bool pose_initialized_;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::shared_ptr<octomap::ColorOcTree> octree_;
    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;
    float fx_ = 348.925, fy_ = 351.135, cx_ = 339.075, cy_ = 177.45, depth_scale_ = 0.001;
    float max_depth_threshold_ = 5.0;  // Maximum allowed depth for unprojection

    // Callback for depth image
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (depth_image_buffer_.size() >= 10) {
            depth_image_buffer_.erase(depth_image_buffer_.begin());
        }
        depth_image_buffer_.push_back(msg);
        matchAndPublish();
    }

    // Callback for pose
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (pose_buffer_.size() >= 10) {
            pose_buffer_.erase(pose_buffer_.begin());
        }
        pose_buffer_.push_back(msg);
        matchAndPublish();
    }

    // Synchronize depth image and pose
    void matchAndPublish()
    {
        if (pose_buffer_.empty() || depth_image_buffer_.empty()) {
            return;
        }

        auto pose = pose_buffer_.front();
        rclcpp::Time pose_time(pose->header.stamp);

        sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
        double min_time_diff = std::numeric_limits<double>::max();

        for (const auto &depth_image : depth_image_buffer_) {
            rclcpp::Time depth_time(depth_image->header.stamp);
            double time_diff = std::abs((pose_time - depth_time).seconds());

            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_depth_image = depth_image;
            }
        }

        if (closest_depth_image != nullptr) {
            processDepthImageAndPose(closest_depth_image, pose);
        }
    }

    // Process the matched depth image and pose
    void processDepthImageAndPose(const sensor_msgs::msg::Image::SharedPtr &depth_image, const geometry_msgs::msg::PoseStamped::SharedPtr &pose)
    {
        cv::Mat depth_cv_image = cv_bridge::toCvCopy(depth_image, "mono16")->image;

        if (depth_cv_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Depth image is empty.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        unprojectDepthImageToPointCloud(depth_cv_image, cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Unprojected point cloud is empty.");
            return;
        }

        Eigen::Affine3d transform = poseToTransform(*pose);
        pcl::transformPointCloud(*cloud, *cloud, transform);

        // Downsample the point cloud
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjusted leaf size
        voxel_filter.filter(*cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Downsampled point cloud is empty.");
            return;
        }

        std::lock_guard<std::mutex> lock(cloud_mutex_);
        accumulated_cloud_ += *cloud;
    }

    // Unproject depth image to 3D point cloud
    void unprojectDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                uint16_t depth = depth_image.at<uint16_t>(v, u);
                if (depth > 0 && depth < max_depth_threshold_ / depth_scale_) {
                    float z = depth * depth_scale_;
                    float x = (u - cx_) * z / fx_;
                    float y = (v - cy_) * z / fy_;
                    cloud->points.push_back(pcl::PointXYZ(x, y, z));
                }
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
    }

    // Convert pose to Eigen transform
    Eigen::Affine3d poseToTransform(const geometry_msgs::msg::PoseStamped &pose)
    {
        Eigen::Translation3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Quaterniond rotation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        return translation * rotation;
    }

    // Save the accumulated point cloud to Octomap
    void saveOctomap()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        if (accumulated_cloud_.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points in accumulated cloud.");
            return;
        }

        for (const auto &point : accumulated_cloud_.points) {
            octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }

        // Save the OctoMap as a binary file
        std::string filename = "octomap.bt";
        if (octree_->writeBinary(filename)) {
            RCLCPP_INFO(this->get_logger(), "Octomap saved as binary file: %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save Octomap as binary.");
        }
    }

    // Run Open3D visualization
    void runOpen3DVisualizer()
    {
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("PointCloud Visualizer", 640, 480);
        open3d::geometry::PointCloud open3d_cloud;

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);

                open3d_cloud.Clear();
                for (const auto &point : accumulated_cloud_.points) {
                    open3d_cloud.points_.emplace_back(point.x, point.y, point.z);
                }
            }

            if (!open3d_cloud.points_.empty()) {
                visualizer.ClearGeometries();
                visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(open3d_cloud));
            }

            visualizer.PollEvents();
            visualizer.UpdateRender();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        visualizer.DestroyVisualizerWindow();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread open3d_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudStitcherNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
