#include "math.h"
#ifndef MATRIXBASE_PLUGIN_H
#define MATRIXBASE_PLUGIN_H

inline Scalar getR() const{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
    return norm();
}
inline Scalar getTheta() const{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
    return std::atan2(this->operator()(1), this->operator()(0));
}
inline void setPolar(Scalar r, Scalar theta){
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
    this->operator()(0) = r * std::cos(theta);
    this->operator()(1) = r * std::sin(theta);
}
inline void setR(Scalar r){
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
    setPolar(r, getTheta());
}
inline void setTheta(Scalar theta){
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
    setPolar(getR(), theta);
}

#endif // MATRIXBASE_PLUGIN_H
