#ifndef HANSE_SONARLOCALIZATION_UTIL_H
#define HANSE_SONARLOCALIZATION_UTIL_H

#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

namespace localization {
    Eigen::Affine2f positionFromPose(const geometry_msgs::Pose &pose);
    Eigen::Affine2f positionFromAffine3(Eigen::Affine3f affine);
}

#endif
