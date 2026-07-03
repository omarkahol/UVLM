#if !defined(MESHER_H)
#define MESHER_H

#include <memory>
#include <iostream>
#include "bladePanel.h"
#include "build_configuration.h"
#include "rotation.h"
#include <Eigen/Dense>

namespace VLM::mesh {

    void create_blade_mesh(Eigen::MatrixXd *blademesh, const VLM::IO::helicopter &h) {
        double dr = (h.r-h.r0) / h.n_panels;
        for (int i=0; i<h.n_blades; i++) {
            blademesh[i] = Eigen::MatrixXd(3,2*h.n_panels+2);

            Eigen::Matrix3d R = VLM::tensor::R3(i*2*M_PI/h.n_blades)*VLM::tensor::R2(h.alfa_shaft)*VLM::tensor::R1(h.theta);
            
            for (int j=0; j<h.n_panels+1;j++) {
                blademesh[i].col(2*j) = Eigen::Vector3d(h.r0+j*dr,h.c/2,0);
                blademesh[i].col(2*j+1) = Eigen::Vector3d(h.r0+j*dr,-h.c/2,0);
            }
            blademesh[i] = R*blademesh[i];
        }
    };

    void create_blade_panels(VLM::mesh::bladePanel *panels, Eigen::MatrixXd *mesh, const VLM::IO::helicopter &h) {
        for(int i=0; i<h.n_blades; i++) {
            for (int j=0; j<h.n_panels;j++) {
                panels[i*h.n_panels+j].mesh = &mesh[i];
                panels[i*h.n_panels+j].globalDoF[0]=2*j;
                panels[i*h.n_panels+j].globalDoF[1]=2*j+1;
                panels[i*h.n_panels+j].globalDoF[2]=2*j+3;
                panels[i*h.n_panels+j].globalDoF[3]=2*j+2;

                panels[i*h.n_panels+j].computeControlPoint();
                panels[i*h.n_panels+j].computeNormal();
            }
        }
    };
};

#endif // MESHER_H