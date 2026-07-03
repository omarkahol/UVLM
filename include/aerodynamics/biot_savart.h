#if !defined(BIOT_SAVART_H)
#define BIOT_SAVART_H

#include "bladePanel.h"
#include <Eigen/Dense>

namespace VLM::aerodynamics {
double induced_velocity(VLM::mesh::bladePanel *panel_1, VLM::mesh::bladePanel *panel_2) {
    
    const Eigen::Vector3d &cp = panel_2->getControlPoint();
    double v{0.0};
    /*
        1--<--4
        |     |
              ^
        |     |
        2-->--3
    */

    for (int i=0; i<4; i++) {
        int inext = (i+1)%4;
        Eigen::Vector3d p_i = panel_1->getPoint(i+1);
        Eigen::Vector3d p_inext = panel_1->getPoint(inext+1);
        
        Eigen::Vector3d l = p_inext - p_i;
        Eigen::Vector3d r1 = cp - p_i;
        Eigen::Vector3d r2 = cp - p_inext;
        double l_norm = l.norm();
        double r1_norm = r1.norm();
        double r2_norm = r2.norm();

        double cosb1 = M_PI - l.dot(r1)/(l_norm*r1_norm);
        double cosb2 = l.dot(r2)/(l_norm*r2_norm);

        Eigen::Vector3d ev = l.cross(r1);
        double ev_norm = ev.norm();
        double proj = ev.dot(panel_2->getNormal())/ev_norm;
        v += (cosb1+cosb2)*proj/(4*M_PI*r1_norm*cosb1);
    }

    return v;
}
}

#endif // BIOT_SAVART_H
