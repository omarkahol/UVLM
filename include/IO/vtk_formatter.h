#if !defined(VTK_FORMATTER_H)
#define VTK_FORMATTER_H

#include "vtu11.hpp"
#include "vortex_panel.h"
#include "mesher.h"

namespace VLM::IO {
    void vtu_write(arma::mat *mesh,VLM::mesh::vortex_panel *panels, const VLM::IO::helicopter &h, std::string file_name) {

    std::vector<double> points;
    std::vector<vtu11::VtkIndexType> connectivity;
    std::vector<vtu11::VtkIndexType> offsets;
    std::vector<vtu11::VtkCellType> types;

    std::vector<double> pointData;
    std::vector<double> cellData;

    for(int i=0;i<h.n_blades;i++) {

        for(int j=0; j<h.n_panels;j++) {
            types.push_back(9);
            int offset = i*(h.n_panels)*4 + 4*(j+1);
            offsets.push_back(offset);
            cellData.push_back(0);

            int c0offset = i*(2*h.n_panels+2);
            for(int k=0; k<4;k++) {
                connectivity.push_back(c0offset+panels[i*h.n_panels+j].globalDoF[k]);
            }

            const arma::mat &point_arr = mesh[i];

            for(int k=0;k<2;k++) {
                pointData.push_back(k);
                int dof = panels[i*h.n_panels+j].globalDoF[k];
                const arma::vec &point = point_arr.col(dof);
                for (int l=0;l<3;l++) {
                    points.push_back(point[l]);
                }
            }

            if (j==h.n_panels-1) {
                for(auto k : std::vector<int>({3,2})) {
                    pointData.push_back(k-2);
                    int dof = panels[i*h.n_panels+j].globalDoF[k];
                    const arma::vec &point = point_arr.col(dof);
                    for (int l=0;l<3;l++) {
                        points.push_back(point[l]);
                    }
            }

            }

        }

    }
    // Create small proxy mesh type
    vtu11::Vtu11UnstructuredMesh mesh_vtu { points, connectivity, offsets, types };

    // Create tuples with (name, association, number of components) for each data set
    std::vector<vtu11::DataSetInfo> dataSetInfo
    {
        { "PointData", vtu11::DataSetType::PointData, 1 },
        { "CellData", vtu11::DataSetType::CellData, 1 },
    };

    // Write data to .vtu file using Ascii format
    vtu11::writeVtu( file_name, mesh_vtu, dataSetInfo, { pointData, cellData }, "Ascii" );
}
}

#endif // VTK_FORMATTER_H
