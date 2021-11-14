#if !defined(MESHER_H)
#define MESHER_H

#include <memory>
#include <iostream>
#include "vortex_panel.h"
#include "build_configuration.h"
#include "rotation.h"

namespace VLM::mesh {

    arma::subview_col<double> get(VLM::mesh::vortex_panel *panel, const int &point) {
        return panel->mesh->col(panel->globalDoF[point-1]);
    };

    void compute_normal(VLM::mesh::vortex_panel *panel) {
        panel->n=arma::cross(get(panel,1)-get(panel,2),get(panel,3)-get(panel,2));
        panel->n = panel->n/arma::norm(panel->n);
    };

    void compute_control_point(VLM::mesh::vortex_panel *panel) {
        panel->cp = 0.5*( 0.5*(get(panel,1)-get(panel,2)) + 0.5*(get(panel,4)-get(panel,3)));  
    };

    void create_blade_mesh(arma::mat *blademesh, const VLM::IO::helicopter &h) {
        double dr = (h.r-h.r0) / h.n_panels;
        for (int i=0; i<h.n_blades; i++) {
            blademesh[i] = arma::mat(3,2*h.n_panels+2);

            arma::mat R = VLM::tensor::R3(i*2*M_PI/h.n_blades)*VLM::tensor::R2(h.alfa_shaft)*VLM::tensor::R1(h.theta);
            
            for (int j=0; j<h.n_panels+1;j++) {
                blademesh[i].col(2*j) = arma::vec({h.r0+j*dr,h.c/2,0});
                blademesh[i].col(2*j+1) = arma::vec({h.r0+j*dr,-h.c/2,0});
            }
            blademesh[i] = R*blademesh[i];
        }
    };

    void create_blade_panels(VLM::mesh::vortex_panel *panels, arma::mat *mesh, const VLM::IO::helicopter &h) {
        for(int i=0; i<h.n_blades; i++) {
            for (int j=0; j<h.n_panels;j++) {
                panels[i*h.n_panels+j].mesh = &mesh[i];
                panels[i*h.n_panels+j].globalDoF[0]=2*j;
                panels[i*h.n_panels+j].globalDoF[1]=2*j+1;
                panels[i*h.n_panels+j].globalDoF[2]=2*j+3;
                panels[i*h.n_panels+j].globalDoF[3]=2*j+2;

                compute_normal(&panels[i*h.n_panels+j]);
                compute_control_point(&panels[i*h.n_panels+j]);
            }
        }
    };
};

#endif // MESHER_H