#include "eigen_conversions/eigen_msg.h"
#include "util.h"

namespace localization {

Eigen::Affine2f positionFromPose(const geometry_msgs::Pose &pose)
{
    Eigen::Affine3d poseEigen;
    tf::poseMsgToEigen(pose, poseEigen);
    Eigen::Vector3d translation3d = poseEigen.translation();
    Eigen::Vector2f translation((float)translation3d.x(), (float)translation3d.y());
    Eigen::Vector3d direction = poseEigen.rotation() * Eigen::Vector3d(1,0,0);
    float angle = atan2f(direction.y(), direction.x());
    Eigen::Affine2f position = Eigen::Translation2f(translation) * Eigen::Rotation2D<float>(angle);
    return position;
}

Eigen::Affine2f positionFromAffine3(Eigen::Affine3f affine)
{
    Eigen::Vector3f translation3f = affine.translation();
    Eigen::Vector2f translation((float)translation3f.x(), (float)translation3f.y());
    Eigen::Vector3f direction = affine.rotation() * Eigen::Vector3f(1,0,0);
    float angle = atan2f(direction.y(), direction.x());
    Eigen::Affine2f position = Eigen::Translation2f(translation) * Eigen::Rotation2D<float>(angle);
    return position;
}

}
