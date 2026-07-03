#include "wake.h"
#include <omp.h>
#include <iostream>

namespace VLM::aerodynamics {

Eigen::Vector3d induced_velocity_segment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p, double gamma) {
    Eigen::Vector3d r1 = p - p1;
    Eigen::Vector3d r2 = p - p2;
    double r1_norm = r1.norm();
    double r2_norm = r2.norm();
    
    // Check for singularity (point is on the vortex segment)
    double r1xr2_sq = (r1.cross(r2)).squaredNorm();
    double h_sq = r1xr2_sq / (p2 - p1).squaredNorm();
    
    double dot = r1.dot(r2);
    Eigen::Vector3d cross = r1.cross(r2);
    
    double factor = (gamma / (4.0 * M_PI)) * ((r1_norm + r2_norm) / (r1_norm * r2_norm * (r1_norm * r2_norm + dot) + 1e-10));
    
    double core_radius = 0.05; // 5cm core
    if (h_sq < core_radius * core_radius) {
        factor *= (h_sq / (core_radius * core_radius));
    }
    
    return factor * cross;
}

Eigen::Vector3d induced_velocity(const WakePanel& panel, const Eigen::Vector3d& p) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; i++) {
        v += induced_velocity_segment(panel.points[i], panel.points[(i+1)%4], p, panel.gamma);
    }
    return v;
}

Eigen::Vector3d induced_velocity(const Vorton& vorton, const Eigen::Vector3d& p) {
    Eigen::Vector3d r = p - vorton.position;
    double r_norm = r.norm();
    double core_radius = 0.05; // Typical desingularization core radius
    
    if (r_norm < 1e-8) {
        return Eigen::Vector3d::Zero();
    }
    
    // Biot-Savart for a vortex particle with a smoothing kernel
    double r3 = r_norm * r_norm * r_norm;
    double core3 = core_radius * core_radius * core_radius;
    double factor = 1.0 / (4.0 * M_PI * (r3 + core3));
    
    return factor * vorton.alpha.cross(r);
}

void Wake::shed_from_blade(VLM::mesh::bladePanel* bladePanels, int n_blades, int n_panels_per_blade) {
    int n_total = n_blades * n_panels_per_blade;
    bool has_prev = panels.size() >= (size_t)n_total;
    auto prev_it = panels.end();
    if (has_prev) {
        for (int k = 0; k < n_total; ++k) prev_it--;
    }

    for (int i = 0; i < n_blades; i++) {
        for (int j = 0; j < n_panels_per_blade; j++) {
            VLM::mesh::bladePanel& bp = bladePanels[i * n_panels_per_blade + j];
            
            WakePanel wp;
            Eigen::Vector3d pTE1 = bp.getPoint(2);
            Eigen::Vector3d pTE2 = bp.getPoint(3);
            
            wp.points[0] = pTE1;
            wp.points[1] = pTE2;
            
            if (has_prev) {
                wp.points[3] = prev_it->points[0];
                wp.points[2] = prev_it->points[1];
                prev_it++;
            } else {
                Eigen::Vector3d offset(0, 0, -0.05); 
                wp.points[3] = pTE1 + offset;
                wp.points[2] = pTE2 + offset;
            }
            wp.gamma = bp.gamma; // Inherit circulation
            wp.age = 0;
            
            panels.push_back(wp);
        }
    }
}

void Wake::advect(double dt, VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf) {
    // 1. Compute velocity at all wake points
    // Since WakePanel points 2 and 3 are the downstream ones, we only need to move them.
    // However, points 0 and 1 are attached to the TE or the previous panel.
    // A better approach is to advect all points, but keep TE points attached during shedding.
    
    std::vector<Eigen::Vector3d*> all_points;
    for (auto& wp : panels) {
        all_points.push_back(&wp.points[0]);
        all_points.push_back(&wp.points[1]);
        all_points.push_back(&wp.points[2]);
        all_points.push_back(&wp.points[3]);
    }
    for (auto& v : vortons) {
        all_points.push_back(&v.position);
    }
    
    int n_pts = all_points.size();
    std::vector<Eigen::Vector3d> vels(n_pts, Eigen::Vector3d::Zero());
    
    // OpenMP parallelized Biot-Savart interactions
    #pragma omp parallel for
    for (int i = 0; i < n_pts; i++) {
        Eigen::Vector3d p = *all_points[i];
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        
        // Induced by blade
        for (int j = 0; j < num_blade_panels; j++) {
            for (int k = 0; k < 4; k++) {
                vel += induced_velocity_segment(bladePanels[j].getPoint(k+1), bladePanels[j].getPoint((k+1)%4 + 1), p, bladePanels[j].gamma);
            }
        }
        
        // Induced by wake panels
        for (const auto& wp : panels) {
            vel += induced_velocity(wp, p);
        }
        
        // Induced by vortons
        for (const auto& vorton : vortons) {
            vel += induced_velocity(vorton, p);
        }
        // Add freestream velocity
        vel += V_inf;
        
        vels[i] = vel;
    }
    
    // Update positions (Explicit Euler) and store velocities for VTK export
    int pt_idx = 0;
    for (auto& wp : panels) {
        for (int k = 0; k < 4; k++) {
            wp.point_velocities[k] = vels[pt_idx];
            *all_points[pt_idx] += vels[pt_idx] * dt;
            pt_idx++;
        }
        wp.age++;
    }
    for (auto& v : vortons) {
        v.velocity = vels[pt_idx];
        *all_points[pt_idx] += vels[pt_idx] * dt;
        pt_idx++;
        v.age++;
    }
}

void Wake::collapse_panels() {
    auto it = panels.begin();
    while (it != panels.end()) {
        if (it->age >= n_collapse) {
            Vorton v;
            v.position = it->getControlPoint();
            v.alpha = it->gamma * it->getArea() * it->getNormal();
            v.age = it->age;
            vortons.push_back(v);
            
            it = panels.erase(it);
        } else {
            ++it;
        }
    }
}

void Wake::destroy_vortons() {
    auto it = vortons.begin();
    while (it != vortons.end()) {
        if (it->position.norm() > destroy_distance) {
            it = vortons.erase(it);
        } else {
            ++it;
        }
    }
}

void Wake::solve_circulation(VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf, const Eigen::Vector3d& Omega) {
    Eigen::MatrixXd AIC = Eigen::MatrixXd::Zero(num_blade_panels, num_blade_panels);
    Eigen::VectorXd RHS = Eigen::VectorXd::Zero(num_blade_panels);

    // Compute AIC and RHS
    #pragma omp parallel for
    for (int i = 0; i < num_blade_panels; i++) {
        Eigen::Vector3d cp = bladePanels[i].getControlPoint();
        Eigen::Vector3d n = bladePanels[i].getNormal();
        Eigen::Vector3d V_kin = Omega.cross(cp);

        // Downwash from existing wake on this control point
        Eigen::Vector3d V_ind_wake = Eigen::Vector3d::Zero();
        for (const auto& wp : panels) {
            V_ind_wake += induced_velocity(wp, cp);
        }
        for (const auto& vorton : vortons) {
            V_ind_wake += induced_velocity(vorton, cp);
        }

        RHS(i) = -(V_inf - V_kin + V_ind_wake).dot(n); // Note V_kin is subtracted because blade is moving

        for (int j = 0; j < num_blade_panels; j++) {
            // Induced velocity from panel j with gamma=1 on control point i
            Eigen::Vector3d v_ind_j = Eigen::Vector3d::Zero();
            for (int k = 0; k < 4; k++) {
                v_ind_j += induced_velocity_segment(bladePanels[j].getPoint(k+1), bladePanels[j].getPoint((k+1)%4 + 1), cp, 1.0);
            }
            AIC(i, j) = v_ind_j.dot(n);
        }
    }

    // Solve for Gamma
    Eigen::VectorXd Gamma = AIC.fullPivLu().solve(RHS);

    // Update gamma in blade panels
    for (int i = 0; i < num_blade_panels; i++) {
        bladePanels[i].gamma = Gamma(i);
    }
}

Eigen::Vector3d Wake::compute_forces(VLM::mesh::bladePanel* bladePanels, int num_blade_panels, const Eigen::Vector3d& V_inf, const Eigen::Vector3d& Omega, double rho) {
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    
    #pragma omp parallel
    {
        Eigen::Vector3d local_force = Eigen::Vector3d::Zero();
        
        #pragma omp for
        for (int i = 0; i < num_blade_panels; i++) {
            double gamma = bladePanels[i].gamma;
            Eigen::Vector3d panel_force = Eigen::Vector3d::Zero();
            
            for (int k = 0; k < 4; k++) {
                // In VLM, the trailing edge segment (k=1) of the bound ring is cancelled 
                // by the shed wake. We only integrate forces on the LE and side segments.
                if (k == 1) continue;

                Eigen::Vector3d p1 = bladePanels[i].getPoint(k+1);
                Eigen::Vector3d p2 = bladePanels[i].getPoint((k+1)%4 + 1);
                Eigen::Vector3d mid = 0.5 * (p1 + p2);
                Eigen::Vector3d dl = p2 - p1;
                
                // Compute local velocity at the midpoint
                Eigen::Vector3d V_ind = Eigen::Vector3d::Zero();
                
                // Induced by blade
                for (int j = 0; j < num_blade_panels; j++) {
                    for (int kj = 0; kj < 4; kj++) {
                        V_ind += induced_velocity_segment(bladePanels[j].getPoint(kj+1), bladePanels[j].getPoint((kj+1)%4 + 1), mid, bladePanels[j].gamma);
                    }
                }
                
                // Induced by wake
                for (const auto& wp : panels) {
                    V_ind += induced_velocity(wp, mid);
                }
                for (const auto& vorton : vortons) {
                    V_ind += induced_velocity(vorton, mid);
                }
                
                Eigen::Vector3d V_kin = Omega.cross(mid);
                Eigen::Vector3d V_local = V_inf - V_kin + V_ind;
                
                // dF = rho * V_local x (gamma * dl)
                panel_force += rho * V_local.cross(gamma * dl);
            }
            bladePanels[i].force = panel_force;
            local_force += panel_force;
        }
        
        #pragma omp critical
        {
            total_force += local_force;
        }
    }
    
    return total_force;
}

} // namespace VLM::aerodynamics
