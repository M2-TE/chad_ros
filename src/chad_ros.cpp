#include "chad_ros.hpp"

struct ChadRos: public rclcpp::Node {
    ChadRos(): Node("minimal_subscriber") {
        // std::string pointcloud_topic = "/dlio/odom_node/pointcloud/keyframe";
        // std::string pointcloud_topic = "/dlio/odom_node/pointcloud/deskewed";
        // std::string pose_topic = "/dlio/odom_node/pose";
        std::string pointcloud_topic = "/robot/dlio/odom_node/pointcloud/deskewed";
        std::string pose_topic = "/robot/dlio/odom_node/pose";
        _sub_deskewed = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, queue_size, std::bind(&ChadRos::callback_points, this, std::placeholders::_1));
        _sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, queue_size, std::bind(&ChadRos::callback_pose, this, std::placeholders::_1));

        // init mapping backends
        float voxel_resolution = VOXEL_RESOLUTION;
        float sdf_truncation = VOXEL_RESOLUTION * 2;
        bool space_carving = false;
        #if MAPPING_BACKEND == 0
            chad_map_p = new chad::TSDFMap( voxel_resolution, sdf_truncation, 100.0f );
            chad_map_p->_debug_outputs = false;
        #elif MAPPING_BACKEND == 1
            openvdb::initialize();
            vdb_volume_p = new vdbfusion::VDBVolume{ voxel_resolution, sdf_truncation, space_carving };
        #elif MAPPING_BACKEND == 2
            // TODO
        #endif
    }
    ~ChadRos() {
        #if MAPPING_BACKEND == 0
            chad_map_p->print_memory_usage();
            chad_map_p->reconstruct("meshes", 1, true);
        #elif MAPPING_BACKEND == 1
            // generate mesh as per example in repo
            auto [vertices, triangles] = vdb_volume_p->ExtractTriangleMesh(true);
            Eigen::MatrixXd V(vertices.size(), 3);
            for (size_t i = 0; i < vertices.size(); i++) {
                V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
            }
            Eigen::MatrixXi F(triangles.size(), 3);
            for (size_t i = 0; i < triangles.size(); i++) {
                F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
            }
            std::string filename = "mesh.ply";
            igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
        #elif MAPPING_BACKEND == 2
            // TODO
        #endif
        
        // std::ofstream ofs("points.asc");
        // for (const auto& point: POINTS_DEBUG) {
        //     ofs << point.x() << ',' << point.y() << ',' << point.z() << '\n';
        // }
        // ofs.close();
    }

    void callback_points(const sensor_msgs::msg::PointCloud2& msg) {
        // extract pointcloud from message
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(msg, pointcloud);
        std::vector<Eigen::Vector3f> points;
        std::vector<Eigen::Vector3d> pointsd;
        #if MAPPING_BACKEND == 1
            pointsd.reserve(pointcloud.points.size());
            for (const auto& point: pointcloud.points) {
                pointsd.push_back({(double)point.x, (double)point.y, (double)point.z});
            }
        #else
            points.reserve(pointcloud.points.size());
            for (const auto& point: pointcloud.points) {
                points.push_back({point.x, point.y, point.z});
            }
        #endif

        // DEBUG
        // for (const auto& point: pointcloud.points) {
        //     POINTS_DEBUG.push_back({point.x, point.y, point.z});
        // }

        raw_points_bytes += pointcloud.points.size() * sizeof(float) * 3;
        // std::cout << "total bytes: " << raw_points_bytes << std::endl;

        // insert points into TSDF data structure
        #if MAPPING_BACKEND == 0 // CHAD TSDF
            chad_map_p->insert(points, _cur_pos);
        #elif MAPPING_BACKEND == 1 // VDBFusion
            std::cout << "inserting points into VDBFusion" << std::endl;
            Eigen::Vector3d pos = _cur_pos.cast<double>();
            vdb_volume_p->Integrate(pointsd, pos, [](float weighting_input) { return 1.0f; });
        #elif MAPPING_BACKEND == 2 // nvblox
            // TODO
        #endif
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

    uint32_t queue_size = 1000;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_deskewed;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_keyframe;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_pose;
    Eigen::Vector3f _cur_pos = { 0, 0, 0 };
    Eigen::Quaternionf _cur_rot = { 1, 0, 0, 0 };

    #if MAPPING_BACKEND == 0
        chad::TSDFMap* chad_map_p;
    #elif MAPPING_BACKEND == 1
        vdbfusion::VDBVolume* vdb_volume_p;
    #elif MAPPING_BACKEND == 2
        // TODO
    #endif

    // benchmarking vars
    std::vector<Eigen::Vector3f> POINTS_DEBUG;
    std::ofstream measurements;
    size_t frame_count = 0;
    size_t raw_points_bytes = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChadRos>());
    rclcpp::shutdown();

    return 0;
}