// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <vector>
// #include <cmath>
// #include <std_msgs/msg/string.hpp>

// class PointCloudStitcherNode : public rclcpp::Node
// {
// public:
//     PointCloudStitcherNode() : Node("PointCloudStitcherNode")
//     {
//         // Create subscribers for the left and depth images
//         pose_sub_ = this->create_subscription<sensor_msgs::msg::PoseStamped>(
//             "/robot/pose", 10, std::bind(&PointCloudStitcherNode::poseCallback, this, std::placeholders::_1));
//         depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/depth/image_raw", 10, std::bind(&PointCloudStitcherNode::depthImageCallback, this, std::placeholders::_1));
        
//         // Initialize Octomap
//         octree_ = std::make_shared<octomap::ColorOcTree>(0.1);  // Set resolution to 0.1m

//         // Create a timer to save Octomap periodically (every 10 seconds)
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(10), std::bind(&PointCloudStitcherNode::saveOctomap, this));

//         // Run Pangolin in a separate thread
//         pangolin_thread_ = std::thread(&PointCloudStitcherNode::runPangolinVisualizer, this);
    
//         // Buffers with a queue size of 10
//         depth_image_buffer_.reserve(10);
//         pose_buffer_.reserve(10);
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pose_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

//     std::vector<sensor_msgs::msg::Image::SharedPtr> pose_buffer_;
//     std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

//     void poseCallback(const sensor_msgs::msg::PoseStamped::SharedPtr msg)
//     {

//         // Add incoming message to buffer
//         if (pose_buffer_.size() >= 10)
//         {
//             posee_buffer_.erase(pose_buffer_.begin()); // Maintain queue size
//         }
//         pose_buffer_.push_back(msg);

//         // Try to find a matching pose based on the timestamp
//         matchAndPublish();
//     }

//     void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         // std::cout << "depth image callback" << std::endl;

//         // Add incoming message to buffer
//         if (depth_image_buffer_.size() >= 10)
//         {
//             depth_image_buffer_.erase(depth_image_buffer_.begin()); // Maintain queue size
//         }
//         depth_image_buffer_.push_back(msg);

//         // Try to find a matching left image based on the timestamp
//         matchAndPublish();
//     }

//     void matchAndPublish()
//     {
//         if (pose_buffer_.empty() || depth_image_buffer_.empty())
//         {
//             return;
//         }

//         // Get the first pose from pose buffer
//         auto pose = pose_buffer_.front();
//         rclcpp::Time pose_time(pose->header.stamp);

//         // Search for the depth image with the closest timestamp
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

//         if (closest_depth_image != nullptr)
//         {
            
//         }
//     }
// };

// int main(int argc, char **argv)
// {
    
//     rclcpp::init(argc, argv);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "starting!!");


//     rclcpp::spin(std::make_shared<PointCloudStitcherNode>());
    
//     rclcpp::shutdown();
//     return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"  // Correct inclusion for std_msgs::msg::String
#include <mutex>
#include <chrono>

class SyncNode : public rclcpp::Node
{
public:
    SyncNode() : Node("sync_node"), batch_size_(5), batch_count_(0)
    {
        // Subscribers
        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&SyncNode::depthImageCallback, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose", 10, std::bind(&SyncNode::poseCallback, this, std::placeholders::_1));

        // Publisher for synchronized frame IDs
        frame_id_pub_ = this->create_publisher<std_msgs::msg::String>(
            "synced_frame_ids", rclcpp::QoS(10));  // Corrected create_publisher call with QoS
    }

private:
    // Buffers to store messages temporarily
    std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> pose_buffer_;
    std::vector<sensor_msgs::msg::Image::SharedPtr> depth_image_buffer_;

    std::mutex sync_mutex_;

    int batch_size_;   // Number of messages to batch before processing
    int batch_count_;  // Current batch count

    // Callback for depth image
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        if (depth_image_buffer_.size() >= 10)
        {
            depth_image_buffer_.erase(depth_image_buffer_.begin()); // Maintain queue size
        }
        depth_image_buffer_.push_back(msg);

        // Try to find a matching pose
        if (++batch_count_ >= batch_size_)
        {
            batch_count_ = 0;
            matchAndPublish();
        }
    }

    // Callback for pose
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        if (pose_buffer_.size() >= 10)
        {
            pose_buffer_.erase(pose_buffer_.begin()); // Maintain queue size
        }
        pose_buffer_.push_back(msg);

        // Try to find a matching depth image
        if (++batch_count_ >= batch_size_)
        {
            batch_count_ = 0;
            matchAndPublish();
        }
    }

    // Matching function for synchronizing depth image and pose based on timestamps and frame IDs
    void matchAndPublish()
    {
        if (pose_buffer_.empty() || depth_image_buffer_.empty())
        {
            return;
        }

        auto pose = pose_buffer_.front();
        rclcpp::Time pose_time(pose->header.stamp);

        // Find the depth image with the closest timestamp to the pose
        sensor_msgs::msg::Image::SharedPtr closest_depth_image = nullptr;
        rclcpp::Time closest_time;
        double min_time_diff = std::numeric_limits<double>::max();

        for (const auto &depth_image : depth_image_buffer_)
        {
            rclcpp::Time depth_time(depth_image->header.stamp);
            double time_diff = std::abs((pose_time - depth_time).seconds());

            if (time_diff < min_time_diff)
            {
                min_time_diff = time_diff;
                closest_depth_image = depth_image;
                closest_time = depth_time;
            }
        }

        // If matching pair found, publish frame IDs and time difference
        if (closest_depth_image != nullptr)
        {
            double time_difference = (pose_time - closest_time).seconds();
            std::string frame_ids = "Pose Frame ID: " + pose->header.frame_id + 
                                    ", Depth Image Frame ID: " + closest_depth_image->header.frame_id +
                                    ", Time Difference: " + std::to_string(time_difference) + " seconds";

            RCLCPP_INFO(this->get_logger(), "%s", frame_ids.c_str());

            // Publish the frame IDs and time difference
            std_msgs::msg::String msg;
            msg.data = frame_ids;
            frame_id_pub_->publish(msg);
        }
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Publisher for frame IDs
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_id_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
