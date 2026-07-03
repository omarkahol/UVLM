#if !defined(BUILD_CONFIGURTION_H)
#define BUILD_CONFIGURTION_H

#include <cmath>
#include "parser.h"

namespace VLM::IO {

    struct helicopter {
        int n_blades;
        double r;
        double r0;
        double c;
        double theta;
        int n_panels;
        double alfa_shaft;
        int npoints;
    };
    inline helicopter helicopter_configuration(parser &p){
        auto data = p.get();
        helicopter h;
        h.n_blades = data["BLADES"].first;
        h.r = data["RADIUS"].first;
        h.r0 = data["CUTOFF"].first;
        h.c = data["CHORD"].first;
        h.n_panels = data["NPANELS"].first;
        h.theta = data["PITCH"].first * M_PI/180;
        h.alfa_shaft = data["ASHAFT"].first * M_PI/180;
        h.npoints = data["BLADES"].first*data["NPANELS"].first;
        return h;
    }

//FLIGHT MECHANICS PART
    struct mechanics {
        double omega;
        double v_inf_x;
        double v_inf_y;
        double v_inf_z;
    };
    inline mechanics mechanics_configuration(parser &p) {
        mechanics m;
        auto data = p.get();
        m.omega = data["OMEGA"].first * M_PI/30;
        m.v_inf_x = data["V_INF_X"].first;
        m.v_inf_y = data["V_INF_Y"].first;
        m.v_inf_z = data["V_INF_Z"].first;
        return m;
    }

//SIMULATION PART
    struct simulation {
        double t_max;
        int nt;
        int itprint;
        double dt;
        int n_collapse;
        double n_destroy;
    };
    inline simulation simulation_configuration(parser &p) {
        simulation s;
        auto data = p.get();
        s.t_max = data["TFINAL"].first;
        s.itprint = data["ITPRINT"].first;
        s.nt = data["NT"].first;
        s.dt = data["TFINAL"].first/static_cast<double>(data["NT"].first);
        s.n_collapse = data["NCOLLAPSE"].first;
        s.n_destroy = data["NDESTROY"].first;
        return s;
    }
}

#endif // BUILD_CONFIGURTION_H