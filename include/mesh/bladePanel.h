#if !defined(BLADE_PANEL_H)
#define BLADE_PANEL_H

#include <vector>
#include <Eigen/Dense>

namespace VLM::mesh {
class bladePanel {
    public: 
        Eigen::MatrixXd *mesh;
        int globalDoF[4];
        double gamma;
        Eigen::Vector3d cp;
        Eigen::Vector3d n;
        Eigen::Vector3d force;
    
    public:
        bladePanel() : mesh(nullptr), gamma(0.0) {}
        bladePanel(Eigen::MatrixXd *mesh, int globalDoF[4]);
        ~bladePanel();
    
    public:
        Eigen::Vector3d getPoint(const int &p);
        void computeNormal();
        void computeControlPoint();
        Eigen::Vector3d getControlPoint();
        Eigen::Vector3d getNormal();
};
}

#endif // BLADE_PANEL_H
