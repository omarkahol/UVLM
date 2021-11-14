#if !defined(BLADE_PANEL_H)
#define BLADE_PANEL_H

#include <vector>
#include <armadillo>

namespace VLM::mesh {
class bladePanel {
    private: 
        arma::mat *mesh;
        int globalDoF[4];
        double gamma;
        arma::vec cp;
        arma::vec n;
    
    public:
        bladePanel(arma::mat *mesh, int globalDoF[4]);
        ~bladePanel();
    
    public:
        arma::subview_col<double> getPoint(const int &p);
        void computeNormal();
        void computeControlPoint();
        arma::vec getControlPoint();
        arma::vec getNormal();
};
}

#endif // VORTEX_PANEL_H
