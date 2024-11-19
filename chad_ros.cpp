#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;

struct ChadRos: public rclcpp::Node {
    ChadRos(): Node("minimal_subscriber") {
        _sub_deskewed = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/robot/dlio/odom_node/pointcloud/deskewed", queue_size, std::bind(&ChadRos::callback_deskewed, this, _1));
        _sub_keyframe = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/robot/dlio/odom_node/pointcloud/keyframe", queue_size, std::bind(&ChadRos::callback_keyframe, this, _1));
        _sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/dlio/odom_node/pose", queue_size, std::bind(&ChadRos::callback_pose, this, _1));
    }

    void callback_deskewed(const sensor_msgs::msg::PointCloud2&) {
        // TODO
    }
    void callback_keyframe(const sensor_msgs::msg::PointCloud2&) {
        // TODO
    }
    void callback_pose(const geometry_msgs::msg::PoseStamped&) {
        // TODO
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_deskewed;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_keyframe;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_pose;
    uint32_t queue_size = 10;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChadRos>());
    rclcpp::shutdown();
    return 0;
}