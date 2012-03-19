#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class Particle {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2f velocity;
    Eigen::Affine2f position;
    float weight;
};

#endif
