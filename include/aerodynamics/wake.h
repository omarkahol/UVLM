#if !defined(WAKE_H)
#define WAKE_H

#include <vector>
#include <list>
#include <Eigen/Dense>
#include "bladePanel.h"
#include "build_configuration.h"

namespace VLM::aerodynamics {

struct Vorton {
    Eigen::Vector3d position;
    Eigen::Vector3d alpha; // strength vector: Gamma * Area * normal
    Eigen::Vector3d velocity; // Advection velocity
    int age;
};

class WakePanel {
public:
    Eigen::Vector3d points[4];
    Eigen::Vector3d point_velocities[4]; // Advection velocity at each point
    double gamma;
    int age;

    WakePanel() : gamma(0.0), age(0) {
        for (int i=0; i<4; i++) point_velocities[i] = Eigen::Vector3d::Zero();
    }
    
    Eigen::Vector3d getNormal() const {
        Eigen::Vector3d n = (points[2] - points[1]).cross(points[0] - points[1]);
        n.normalize();
        return n;
    }

    Eigen::Vector3d getControlPoint() const {
        return 0.25 * (points[0] + points[1] + points[2] + points[3]);
    }
    
    double getArea() const {
        // Approximate area as sum of two triangles: 0,1,2 and 0,2,3
        double a1 = 0.5 * ((points[1] - points[0]).cross(points[2] - points[0])).norm();
        double a2 = 0.5 * ((points[2] - points[0]).cross(points[3] - points[0])).norm();
        return a1 + a2;
    }
};

class Wake {
public:
    std::list<WakePanel> panels;
    std::list<Vorton> vortons;

    int n_collapse;
    double destroy_distance;

    Wake(int collapse_iters, double destroy_dist) 
        : n_collapse(collapse_iters), destroy_distance(destroy_dist) {}

    // Shed new panels from the trailing edge of the blade
    void shed_from_blade(VLM::mesh::bladePanel* bladePanels, int n_blades, int n_panels_per_blade);

    // Advect all wake elements
    void advect(double dt, VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf);

    // Convert old panels to vortons
    void collapse_panels();

    // Destroy vortons far from the rotor
    void destroy_vortons();

    // Solve the aerodynamic circulation (Gamma)
    void solve_circulation(VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf, const Eigen::Vector3d& Omega);
    
    // Compute the forces on the blade
    Eigen::Vector3d compute_forces(VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf, const Eigen::Vector3d& Omega, double rho);
};

// Compute induced velocity from a WakePanel on a point
Eigen::Vector3d induced_velocity(const WakePanel& panel, const Eigen::Vector3d& p);

// Compute induced velocity from a Vorton on a point
Eigen::Vector3d induced_velocity(const Vorton& vorton, const Eigen::Vector3d& p);

} // namespace VLM::aerodynamics

#endif // WAKE_H
