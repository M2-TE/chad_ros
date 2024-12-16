#include "chad_grid.hpp"
#include "chad_reconstruction.hpp"
#include "chad_lvr2.hpp"
#include "chad/chad.hpp"

void reconstruct(Chad& chad, uint32_t root_addr, std::string_view mesh_name, bool save_grid) {
    typedef lvr2::BaseVector<float> VecT;
    typedef lvr2::BilinearFastBox<VecT> BoxT;
    
    // create hash grid from entire tree
    // generate mesh from hash grid
    lvr2::PMPMesh<VecT> mesh{};
    std::string decomp_type = "PMC";
    if (decomp_type == "MC") {
    }
    else if (decomp_type == "PMC") {
        auto node_levels = chad.get_node_levels();
        auto leaf_level = chad.get_leaf_level();
        auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, leaf_level, root_addr, LEAF_RESOLUTION);
        if (save_grid) grid_p->saveGrid("hashgrid.grid");
        
        ChadReconstruction<VecT, BoxT> reconstruction { grid_p };
        reconstruction.getMesh(mesh);
    }
    
    // generate mesh buffer from reconstructed mesh
    auto norm_face = lvr2::calcFaceNormals(mesh);
    auto norm_vert = lvr2::calcVertexNormals(mesh, norm_face);
    lvr2::MeshBufferPtr mesh_buffer_p;
    if (false) {
        // coloring
        auto cluster_map = lvr2::planarClusterGrowing(mesh, norm_face, 0.85);
        lvr2::ClusterPainter cluster_painter { cluster_map };
        lvr2::ColorGradient::GradientType t = lvr2::ColorGradient::gradientFromString("GREY");
        auto cluster_colors = boost::optional<lvr2::DenseClusterMap<lvr2::RGB8Color>>(cluster_painter.colorize(mesh, t));
        lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer { cluster_map };
        finalizer.setClusterColors(*cluster_colors);
        finalizer.setVertexNormals(norm_vert);
        mesh_buffer_p = finalizer.apply(mesh);
    }
    else {
        // calc normals for vertices
        lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
        finalizer.setNormalData(norm_vert);
        mesh_buffer_p = finalizer.apply(mesh);
    }

    // save to disk
    auto model_p = std::make_shared<lvr2::Model>(mesh_buffer_p);
    lvr2::ModelFactory::saveModel(model_p, mesh_name.data());
    std::cout << "Saved mesh to " << mesh_name << std::endl;
}