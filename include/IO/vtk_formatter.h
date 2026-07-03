#if !defined(VTK_FORMATTER_H)
#define VTK_FORMATTER_H

#include "vtu11/vtu11.hpp"
#include "bladePanel.h"
#include "mesher.h"
#include "wake.h"
#include <Eigen/Dense>

namespace VLM::IO {
    void vtu_write(Eigen::MatrixXd *mesh, VLM::mesh::bladePanel *panels, const VLM::IO::helicopter &h, const VLM::aerodynamics::Wake& wake, std::string file_name, const Eigen::Vector3d& V_inf, const Eigen::Vector3d& Omega_vec) {

    std::vector<double> points;
    std::vector<vtu11::VtkIndexType> connectivity;
    std::vector<vtu11::VtkIndexType> offsets;
    std::vector<vtu11::VtkCellType> types;

    std::vector<double> pointData;
    std::vector<double> cellData;
    std::vector<double> cellGamma;
    std::vector<double> pointVelocity;
    std::vector<double> cellForce;

    int current_offset = 0;
    
    for(int i=0;i<h.n_blades;i++) {
        for(int j=0; j<h.n_panels;j++) {
            types.push_back(9);
            current_offset += 4;
            offsets.push_back(current_offset);
            cellData.push_back(0.0); // 0 = Blade
            cellGamma.push_back(panels[i*h.n_panels+j].gamma);
            cellForce.push_back(panels[i*h.n_panels+j].force.x());
            cellForce.push_back(panels[i*h.n_panels+j].force.y());
            cellForce.push_back(panels[i*h.n_panels+j].force.z());

            int c0offset = i*(2*h.n_panels+2);
            int quad_order[4] = {0, 1, 2, 3};
            for(int k=0; k<4;k++) {
                connectivity.push_back(c0offset+panels[i*h.n_panels+j].globalDoF[quad_order[k]]);
            }

            const Eigen::MatrixXd &point_arr = mesh[i];

            for(int k=0;k<2;k++) {
                pointData.push_back(k);
                int dof = panels[i*h.n_panels+j].globalDoF[k];
                const Eigen::VectorXd point = point_arr.col(dof);
                for (int l=0;l<3;l++) {
                    points.push_back(point[l]);
                }
                Eigen::Vector3d pt(point[0], point[1], point[2]);
                Eigen::Vector3d V_kin = Omega_vec.cross(pt);
                Eigen::Vector3d vel = V_inf - V_kin;
                pointVelocity.push_back(vel.x());
                pointVelocity.push_back(vel.y());
                pointVelocity.push_back(vel.z());
            }

            if (j==h.n_panels-1) {
                for(auto k : std::vector<int>({3,2})) {
                    pointData.push_back(k-2);
                    int dof = panels[i*h.n_panels+j].globalDoF[k];
                    const Eigen::VectorXd point = point_arr.col(dof);
                    for (int l=0;l<3;l++) {
                        points.push_back(point[l]);
                    }
                    Eigen::Vector3d pt(point[0], point[1], point[2]);
                    Eigen::Vector3d V_kin = Omega_vec.cross(pt);
                    Eigen::Vector3d vel = V_inf - V_kin;
                    pointVelocity.push_back(vel.x());
                    pointVelocity.push_back(vel.y());
                    pointVelocity.push_back(vel.z());
                }
            }
        }
    }

    int current_point_idx = h.n_blades * (2*h.n_panels + 2);
    
    // Add Wake Panels
    for (const auto& wp : wake.panels) {
        types.push_back(9); // VTK_QUAD
        current_offset += 4;
        offsets.push_back(current_offset);
        cellData.push_back(1.0); // 1 = Wake Panel
        cellGamma.push_back(wp.gamma);
        cellForce.push_back(0.0); cellForce.push_back(0.0); cellForce.push_back(0.0);
        
        int quad_order[4] = {0, 1, 2, 3};
        for (int k = 0; k < 4; k++) {
            int qk = quad_order[k];
            connectivity.push_back(current_point_idx++);
            points.push_back(wp.points[qk].x());
            points.push_back(wp.points[qk].y());
            points.push_back(wp.points[qk].z());
            pointData.push_back(1.0);
            pointVelocity.push_back(wp.point_velocities[qk].x());
            pointVelocity.push_back(wp.point_velocities[qk].y());
            pointVelocity.push_back(wp.point_velocities[qk].z());
        }
    }
    
    // Add Vortons
    for (const auto& vorton : wake.vortons) {
        types.push_back(1); // VTK_VERTEX
        current_offset += 1;
        offsets.push_back(current_offset);
        cellData.push_back(2.0); // 2 = Vorton
        cellGamma.push_back(vorton.alpha.norm());
        cellForce.push_back(0.0); cellForce.push_back(0.0); cellForce.push_back(0.0);
        
        connectivity.push_back(current_point_idx++);
        points.push_back(vorton.position.x());
        points.push_back(vorton.position.y());
        points.push_back(vorton.position.z());
        pointData.push_back(2.0);
        pointVelocity.push_back(vorton.velocity.x());
        pointVelocity.push_back(vorton.velocity.y());
        pointVelocity.push_back(vorton.velocity.z());
    }

    // Create small proxy mesh type
    vtu11::Vtu11UnstructuredMesh mesh_vtu { points, connectivity, offsets, types };

    // Create tuples with (name, association, number of components) for each data set
    std::vector<vtu11::DataSetInfo> dataSetInfo
    {
        { "PointData", vtu11::DataSetType::PointData, 1 },
        { "Velocity", vtu11::DataSetType::PointData, 3 },
        { "EntityType", vtu11::DataSetType::CellData, 1 },
        { "Gamma", vtu11::DataSetType::CellData, 1 },
        { "Force", vtu11::DataSetType::CellData, 3 },
    };

    // Write data to .vtu file using Base64Inline format
    vtu11::writeVtu( file_name, mesh_vtu, dataSetInfo, { pointData, pointVelocity, cellData, cellGamma, cellForce }, "Base64Inline" );
}
}

#endif // VTK_FORMATTER_H
