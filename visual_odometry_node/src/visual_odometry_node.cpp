
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
// #include <ORB_SLAM3/System.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Include ORB-SLAM3 headers (adjust paths as needed)
#include "System.h"  // Ensure ORB-SLAM3 is properly integrated with ROS2

class VisualOdometryNode : public rclcpp::Node
{
public:
    VisualOdometryNode()
    : Node("visual_odometry_node")
    {
        RCLCPP_INFO(this->get_logger(), "VisualOdometryNode starting...");

        // Create subscribers for the left and depth camera images
        left_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/left/image_raw", 10, std::bind(&VisualOdometryNode::leftImageCallback, this, std::placeholders::_1));

        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&VisualOdometryNode::depthImageCallback, this, std::placeholders::_1));

        // Publisher for robot pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot/pose", 10);

        // Initialize ORB-SLAM3 system
        slam_system_ = new ORB_SLAM3::System(
            "/home/shetty/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt", 
            "/home/shetty/ros2_ws/src/ORB_SLAM3/Examples/RGB-D/custom_camera.yaml", 
            ORB_SLAM3::System::RGBD, true);

        // Reserve space for buffers
        left_image_buffer_.reserve(10);
        depth_image_buffer_.reserve(10);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    ORB_SLAM3::System* slam_system_;
    
    // Buffers for image synchronization
    std::vector<sensor_msgs::msg::Image::SharedPtr> left_image_buffer_;
    std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

    // Callback for left camera images
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Add left image to buffer
        if (left_image_buffer_.size() >= 10) {
            left_image_buffer_.erase(left_image_buffer_.begin());
        }
        left_image_buffer_.push_back(msg);

        // Try to find a matching depth image and process
        matchAndProcess();
    }

    // Callback for depth camera images
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Add depth image to buffer
        if (depth_image_buffer_.size() >= 10) {
            depth_image_buffer_.erase(depth_image_buffer_.begin());
        }
        depth_image_buffer_.push_back(msg);

        // Try to find a matching left image and process
        matchAndProcess();
    }

    void matchAndProcess()
    {
        if (left_image_buffer_.empty() || depth_image_buffer_.empty()) {
            return;
        }

        // Get the first left image and its timestamp
        auto left_image = left_image_buffer_.front();
        rclcpp::Time left_time(left_image->header.stamp);

        // Find the depth image with the closest timestamp
        sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
        rclcpp::Time closest_time;
        double min_time_diff = std::numeric_limits<double>::max();

        for (const auto &depth_image : depth_image_buffer_) {
            rclcpp::Time depth_time(depth_image->header.stamp);
            double time_diff = std::abs((left_time - depth_time).seconds());

            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_depth_image = depth_image;
                closest_time = depth_time;
            }
        }

        // If we found a matching depth image, proceed with ORB-SLAM3 processing
        if (closest_depth_image != nullptr) {
            processFrame(left_image, closest_depth_image);

            // Remove matched frames from buffers
            left_image_buffer_.erase(left_image_buffer_.begin());
            depth_image_buffer_.erase(std::remove(depth_image_buffer_.begin(), depth_image_buffer_.end(), closest_depth_image), depth_image_buffer_.end());
        }
    }

    void processFrame(const sensor_msgs::msg::Image::SharedPtr& left_image_msg, const sensor_msgs::msg::Image::SharedPtr& depth_image_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing synchronized frames.");

        // Convert ROS2 messages to OpenCV images
        cv::Mat left_image, depth_image;
        try {
            left_image = cv_bridge::toCvCopy(left_image_msg, "bgr8")->image;
            depth_image = cv_bridge::toCvCopy(depth_image_msg, "mono16")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Feed synchronized frames to ORB-SLAM3 for tracking
        Sophus::SE3f pose;
        try {
            pose = slam_system_->TrackRGBD(left_image, depth_image, this->get_clock()->now().seconds());
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 tracking exception: %s", e.what());
            return;
        }

        // Publish the pose if tracking is successful
        if (slam_system_->GetTrackingState() == ORB_SLAM3::Tracking::OK) {
            publishPose(pose);
        } else {
            RCLCPP_WARN(this->get_logger(), "Tracking failed.");
        }
    }

    void publishPose(Sophus::SE3f& pose)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "world";

        // Set position and orientation
        pose_msg.pose.position.x = pose.translation()[0];
        pose_msg.pose.position.y = pose.translation()[1];
        pose_msg.pose.position.z = pose.translation()[2];

        Eigen::Quaternionf q(pose.unit_quaternion());
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish the pose
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Pose published.");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
