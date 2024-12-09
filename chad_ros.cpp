#include "chad_ros.hpp"
#include "chad/chad.hpp"

struct ChadRos: public rclcpp::Node {
    ChadRos(): Node("minimal_subscriber") {
        // _sub_deskewed = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/dlio/odom_node/pointcloud/deskewed", queue_size, std::bind(&ChadRos::callback_points, this, std::placeholders::_1));
        _sub_keyframe = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dlio/odom_node/pointcloud/keyframe", queue_size, std::bind(&ChadRos::callback_points, this, std::placeholders::_1));
        _sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/dlio/odom_node/pose", queue_size, std::bind(&ChadRos::callback_pose, this, std::placeholders::_1));
    }
    ~ChadRos() {
        chad.merge_all_subtrees();
        chad.print_stats();
    }

    void callback_points(const sensor_msgs::msg::PointCloud2& msg) {
        // extract pointcloud from message
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(msg, pointcloud);
        std::vector<Eigen::Vector3f> points;
        points.reserve(pointcloud.points.size());
        for (const auto& point: pointcloud.points) {
            points.push_back({point.x, point.y, point.z});
        }

        // insert points into TSDF CHAD
        std::cout << "Inserting " << points.size() << " points into CHAD" << std::endl;
        chad.insert(points, _cur_pos, _cur_rot);
    }
    void callback_pose(const geometry_msgs::msg::PoseStamped& msg) {
        // extract position
        _cur_pos = {
            (float)msg.pose.position.x,
            (float)msg.pose.position.y,
            (float)msg.pose.position.z
        };
        // extract rotation
        _cur_rot = {
            (float)msg.pose.orientation.w,
            (float)msg.pose.orientation.x,
            (float)msg.pose.orientation.y,
            (float)msg.pose.orientation.z
        };
    }

    uint32_t queue_size = 10;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_deskewed;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_keyframe;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_pose;
    Eigen::Vector3f _cur_pos = { 0, 0, 0 };
    Eigen::Quaternionf _cur_rot = { 1, 0, 0, 0 };
    Chad chad;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChadRos>());
    rclcpp::shutdown();
    return 0;
}