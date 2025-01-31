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
        #if MAPPING_BACKEND == 1
            openvdb::initialize();
            vdb_volume_p = new vdbfusion::VDBVolume{ voxel_resolution, sdf_truncation, space_carving };
        #elif MAPPING_BACKEND == 2
            octomap_tree_p = new octomap::OcTree{ voxel_resolution };
        #endif

        // create a new measurement file
        #ifdef BENCHMARKING
            measurements = std::ofstream{ "measurements.csv" };
            measurements << "frame,mem,dur\n";
        #endif
    }
    ~ChadRos() {
        #ifdef BENCHMARKING
            measurements.close();
        #endif

        return; // DISABLING MAP CONSTUCTION

        #if MAPPING_BACKEND == 0
            chad.merge_all_subtrees();
            chad.print_stats();
            reconstruct(chad, 1, "mesh.ply", true);
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
            // octomap_tree_p->writeBinary("maps/octomap.bt");
        #endif
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

        #ifdef BENCHMARKING
            auto beg = std::chrono::high_resolution_clock::now();
        #endif

        // insert points into TSDF data structure
        #if MAPPING_BACKEND == 0 // CHAD TSDF
            std::cout << "Inserting " << points.size() << " points into CHAD" << std::endl;
            chad.insert(points, _cur_pos, _cur_rot);
        #elif MAPPING_BACKEND == 1 // VDBFusion
            Eigen::Vector3d pos = _cur_pos.cast<double>();
            vdb_volume_p->Integrate(pointsd, pos, [](float weighting_input) { return 1.0f; });
        #elif MAPPING_BACKEND == 2 // octomap
            octomap::Pointcloud cloud;
            for (auto& point: points) {
                cloud.push_back({ point.x(), point.y(), point.z() });
            }
            octomap_tree_p->insertPointCloud(cloud, { _cur_pos.x(), _cur_pos.y(), _cur_pos.z() });
        #endif

        #ifdef BENCHMARKING
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::milliseconds dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
            std::cout << "TSDF update took " << dur.count() << "ms" << std::endl;

            // manually reset pointcloud buffers before reading memory
            points = {};
            pointsd = {};
            pointcloud = {};

            // measure physical memory footprint
            double mb = (double)read_phys_mem_kb() / 1024.0;
            #if MAPPING_BACKEND == 0
                // double mb_read = chad.get_readonly_size(); // memory leak?
                // double mb_hash = chad.get_hash_size(); // memory leak?
            #endif

            // write measurements to csv
            measurements
                << frame_count++ << ','
                << mb << ','
                << dur.count()
                << std::endl;
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

    uint32_t queue_size = 100;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_deskewed;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_keyframe;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_pose;
    Eigen::Vector3f _cur_pos = { 0, 0, 0 };
    Eigen::Quaternionf _cur_rot = { 1, 0, 0, 0 };

    #if MAPPING_BACKEND == 0
        Chad chad;
    #elif MAPPING_BACKEND == 1
        vdbfusion::VDBVolume* vdb_volume_p;
    #elif MAPPING_BACKEND == 2
        octomap::OcTree* octomap_tree_p;
    #endif

    // benchmarking vars
    std::ofstream measurements;
    size_t frame_count = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChadRos>());
    rclcpp::shutdown();
    return 0;
}