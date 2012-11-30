#include "wall_follow_shift_algo.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class CompPointsY
{
public:
bool operator()(Vector2d v1, Vector2d v2)
{
    if(v1.coeff(1) < v2.coeff(1))
        return true;
    else
        return false;
}
};

void wall_follow_shift_algo::sonar_laser_update(
        const sensor_msgs::LaserScan::ConstPtr& msg,
        geometry_msgs::PoseStamped &goal)
{
    set ＜Vector2d, CompPointsY＞ shifted_sonar_points;
    Vector2d shift_distance(1,0);

    for (unsigned int i = 0; i < msg->ranges.size(); i++){
        double angle = msg->angle_min + msg->angle_increment * i;
        //calculating x, y coordinates of laser scan
        Vector2d p(msg->ranges.at(i) * cos(angle),msg->ranges.at(i) * sin(angle));

        //shifting scan
        p -= shift_distance;

        shifted_sonar_points.insert(p);
    }


}
