#include "chad_ros.hpp"

struct ChadRos: public rclcpp::Node {
    ChadRos(): Node("minimal_subscriber") {
        // std::string pointcloud_topic = "/dlio/odom_node/pointcloud/keyframe";
        std::string pointcloud_topic = "/dlio/odom_node/pointcloud/deskewed";
        std::string pose_topic = "/dlio/odom_node/pose";
        _sub_deskewed = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, queue_size, std::bind(&ChadRos::callback_points, this, std::placeholders::_1));
        _sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, queue_size, std::bind(&ChadRos::callback_pose, this, std::placeholders::_1));

        float voxel_resolution = VOXEL_RESOLUTION;
        float sdf_truncation = VOXEL_RESOLUTION * 2;
        bool space_carving = false;
        #if MAPPING_BACKEND == 1
            openvdb::initialize();
            vdb_volume_p = new vdbfusion::VDBVolume{ voxel_resolution, sdf_truncation, space_carving };
        #elif MAPPING_BACKEND == 2
            nvblox_mapper_p = new nvblox::Mapper(voxel_resolution, nvblox::MemoryType::kDevice);
        #endif
    }
    ~ChadRos() {
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
            std::string filename = "maps/mesh.ply";
            igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
        #elif MAPPING_BACKEND == 2
            nvblox_mapper_p->saveMeshAsPly("maps/nvblox");
        #endif
    }

    void callback_points(const sensor_msgs::msg::PointCloud2& msg) {
        // extract pointcloud from message
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(msg, pointcloud);
        std::vector<std::array<float, 3>> points;
        std::vector<Eigen::Vector3d> pointsd;
        points.reserve(pointcloud.points.size());
        for (const auto& point: pointcloud.points) {
            points.push_back({point.x, point.y, point.z});
            pointsd.push_back({(double)point.x, (double)point.y, (double)point.z});
        }

        #ifdef BENCHMARKING
            auto beg = std::chrono::high_resolution_clock::now();
        #endif

        // insert points into TSDF data structure
        #if MAPPING_BACKEND == 0 // CHAD TSDF
            std::cout << "Inserting " << points.size() << " points into CHAD" << std::endl;
            chad.insert(points, _cur_pos, _cur_rot);
        #elif MAPPING_BACKEND == 1 // VDBFusion
            Eigen::Vector3d pos = _cur_pos.cast<double>();
            vdb_volume_p->Integrate(pointsd, pos, [](float weighting_input) { return weighting_input; });
        #elif MAPPING_BACKEND == 2 // NVBlox
        #endif

        #ifdef BENCHMARKING
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::milliseconds dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
            total += dur;
            min = std::min(min, dur);
            max = std::max(max, dur);

            // manually reset pointcloud buffers before reading memory
            points = {};
            pointsd = {};
            pointcloud = {};

            // measure physical memory footprint
            #if MAPPING_BACKEND == 0
                double mb = (double)read_phys_mem_kb() / 1024.0;
                double mb_read = chad.get_readonly_size();
                double mb_hash = chad.get_hash_size();
            #elif MAPPING_BACKEND > 0
                double mb = (double)read_phys_mem_kb() / 1024.0;
            #endif

            // write measurements to csv
            std::ofstream file { "measurements.csv", std::ofstream::app };
            file << frame_count++ << ',' 
                << mb << ',' 
                << dur.count()
                << '\n';
            file.close();
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
        Chad chad;
    #elif MAPPING_BACKEND == 1
        vdbfusion::VDBVolume* vdb_volume_p;
    #elif MAPPING_BACKEND == 2
        nvblox::Mapper* nvblox_mapper_p;
    #endif

    // benchmarking vars
    std::chrono::milliseconds min = std::chrono::milliseconds::max();
    std::chrono::milliseconds max = std::chrono::milliseconds::min();
    std::chrono::milliseconds total = std::chrono::milliseconds::zero();
    size_t frame_count = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChadRos>());
    rclcpp::shutdown();
    return 0;
}