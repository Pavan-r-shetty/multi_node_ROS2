#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

class ImageLoaderNode : public rclcpp::Node
{
public:
    ImageLoaderNode() : Node("image_loader_node")
    {
        // Publishers for the left and depth images
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/left/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);

        // Load directories from parameters or hardcode them for now
        left_image_dir_ = this->declare_parameter<std::string>("left_image_dir", "/path/to/left_images");
        depth_image_dir_ = this->declare_parameter<std::string>("depth_image_dir", "/path/to/depth_images");

        // Start the timer to publish images at a fixed rate
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImageLoaderNode::publishImages, this));
        
        // Load image paths
        loadImages();
    }

private:
    void loadImages()
    {
        // Lambda function to extract the numerical part of the file name
        auto extractImageNumber = [](const std::string &path) -> int {
            std::string filename = fs::path(path).stem().string(); // Get file name without extension
            return std::stoi(filename); // Convert the numeric part to an integer
        };

        // Load left and depth image file paths from the directories
        for (const auto &entry : fs::directory_iterator(left_image_dir_)) {
            left_image_paths_.push_back(entry.path().string());
        }
        for (const auto &entry : fs::directory_iterator(depth_image_dir_)) {
            depth_image_paths_.push_back(entry.path().string());
        }

        // Sort based on the extracted numerical value from file names
        std::sort(left_image_paths_.begin(), left_image_paths_.end(), [&extractImageNumber](const std::string &a, const std::string &b) {
            return extractImageNumber(a) < extractImageNumber(b);
        });

        std::sort(depth_image_paths_.begin(), depth_image_paths_.end(), [&extractImageNumber](const std::string &a, const std::string &b) {
            return extractImageNumber(a) < extractImageNumber(b);
        });

        if (left_image_paths_.size() != depth_image_paths_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Number of left and depth images do not match!");
        }
    }

    void publishImages()
    {
        if (image_index_ >= left_image_paths_.size()) {
            RCLCPP_INFO(this->get_logger(), "All images published");
            rclcpp::shutdown();
            return;
        }

        // Load left and depth images using OpenCV
        cv::Mat left_image = cv::imread(left_image_paths_[image_index_], cv::IMREAD_COLOR);
        cv::Mat depth_image = cv::imread(depth_image_paths_[image_index_], cv::IMREAD_UNCHANGED);

        // Convert them to ROS2 sensor_msgs using cv_bridge
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_image).toImageMsg();
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();

        // Set headers (timestamps can be added if needed)
        auto time_now = this->now();
        left_msg->header.stamp = time_now;
        depth_msg->header.stamp = time_now;
        left_msg->header.frame_id = left_image_paths_[image_index_];
        depth_msg->header.frame_id = depth_image_paths_[image_index_];

        // Publish the images
        left_image_pub_->publish(*left_msg);
        depth_image_pub_->publish(*depth_msg);

        RCLCPP_INFO(this->get_logger(), "Published image pair: %zu", image_index_);
        image_index_++;
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Directories and image paths
    std::string left_image_dir_;
    std::string depth_image_dir_;
    std::vector<std::string> left_image_paths_;
    std::vector<std::string> depth_image_paths_;

    // Image index
    size_t image_index_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageLoaderNode>());
    rclcpp::shutdown();
    return 0;
}
