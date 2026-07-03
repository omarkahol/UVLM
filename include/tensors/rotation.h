#if !defined(ROTATION_H)
#define ROTATION_H

#include <math.h>
#include <Eigen/Dense>

namespace VLM::tensor {

    Eigen::Matrix3d R3 (double t) {
        Eigen::Matrix3d r;
        r << std::cos(t), -std::sin(t), 0,
             std::sin(t),  std::cos(t), 0,
             0,            0,           1;
        return r;
    }

    Eigen::Matrix3d R2 (double t) {
        Eigen::Matrix3d r;
        r << std::cos(t), 0, std::sin(t),
             0,           1, 0,
            -std::sin(t), 0, std::cos(t);
        return r;
    }

    Eigen::Matrix3d R1 (double t) {
        Eigen::Matrix3d r;
        r << 1, 0,            0,
             0, std::cos(t), -std::sin(t),
             0, std::sin(t),  std::cos(t);
        return r;
    }
 
}

#endif // ROTATION_H