#if !defined(ROTATION_H)
#define ROTATION_H

#include <math.h>
#include "armadillo"

namespace VLM::tensor {

    arma::mat R3 (double t) {
        arma::mat33 r = {
                {std::cos(t),-std::sin(t),0}, 
                {std::sin(t), std::cos(t),0},
                {0,0,1}
                };
        return r;
    }

    arma::mat R2 (double t) {
        arma::mat33 r = {
                {std::cos(t),0,std::sin(t)},
                {0,1,0},
                {-std::sin(t),0, std::cos(t)}
                };
        return r;
    }

    arma::mat R1 (double t) {
        arma::mat33 r = {
                {1,0,0},
                {0,std::cos(t),-std::sin(t)}, 
                {0,std::sin(t), std::cos(t)}
            };
        return r;
    }
 
}

#endif // ROTATION_H