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
    helicopter helicopter_configuration(parser &p){
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
    };
    mechanics mechanics_configuration(parser &p) {
        mechanics m;
        auto data = p.get();
        m.omega = data["OMEGA"].first * M_PI/30;
        return m;
    }

//SIMULATION PART
    struct simulation {
        double t_max;
        int nt;
        int itprint;
        double dt;
    };
    simulation simulation_configuration(parser &p) {
        simulation s;
        auto data = p.get();
        s.t_max = data["TFINAL"].first;
        s.itprint = data["ITPRINT"].first;
        s.nt = data["NT"].first;
        s.dt = data["TFINAL"].first/static_cast<double>(data["NT"].first);
        return s;
    }
}

#endif // BUILD_CONFIGURTION_H