#if !defined(BIOT_SAVART_H)
#define BIOT_SAVART_H
#include "mesh_movement.h"

namespace VLM::aerodynamics{
double induced_velocity(VLM::mesh::vortex_panel *panel_1, VLM::mesh::vortex_panel *panel_2) {
    
    const arma::vec &cp = panel_2->get_control_point();
    double v{0.0};
    /*
        1--<--4
        |     |
              ^
        |     |
        2-->--3
    */

    const arma::mat &p = panel_1->get_mesh();
    
    for (int i=0; i<4; i++) {
        int inext = (i+1)%4;
        arma::vec l = p.col(inext)-p.col(i);
        arma::vec r1 = cp-p.col(i);
        arma::vec r2 = cp-p.col(inext);
        double l_norm = arma::norm(l);
        double r1_norm = arma::norm(r1);
        double r2_norm = arma::norm(r2);

        double cosb1 = M_PI-arma::dot(l,r1)/(l_norm*r1_norm);
        double cosb2 = arma::dot(l,r2)/(l_norm*r2_norm);

        arma::vec ev = arma::cross(l,r1);
        double ev_norm = arma::norm(ev);
        double proj = arma::dot(ev,panel_2->get_normal())/ev_norm;
        v += (cosb1+cosb2)*proj/(4*M_PI*r1_norm*cosb1);
    }

    return v;
}
}

#endif // BIOT_SAVART_H
