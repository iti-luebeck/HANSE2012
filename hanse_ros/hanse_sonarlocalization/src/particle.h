#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Geometry>

class Particle {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Affine2f position;
    float weight;
};

#endif
