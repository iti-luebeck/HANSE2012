#include "wall_follow_shift_algo.h"


using namespace Eigen;
using std::set;

class CompPointsX
{
public:
    bool operator()(Vector3d v1, Vector3d v2)
    {
        if(v1(0) < v2(0))
            return true;
        else
            return false;
    }
};

void wall_follow_shift_algo::sonar_laser_update(
        const sensor_msgs::LaserScan::ConstPtr& msg,
        Vector3d &goal,
        Quaterniond &orientation) throw (std::runtime_error)
{

    Vector3d shift_distance(0, -3, 0);

    //searching for nearest scan
    double min_dist = DBL_MAX;
    unsigned int start_index = -1;
    for(unsigned int i = 0; i<msg->ranges.size(); i++){
        if (msg->ranges[i] <= min_dist && msg->ranges[i] > 0){
            min_dist = msg->ranges[i];
            start_index = i;
        }
    }

    Vector3d v_sum(0, 0, 0);
    double d_sum = 0;

    unsigned int k = 0;
    //while (k <= 10){
        unsigned int index = (start_index - k + msg->ranges.size()-5) % msg->ranges.size();
        if(msg->ranges.at(index) > 0){
            double angle = msg->angle_min + msg->angle_increment * index;
            //calculating x, y coordinates of laser scan
            Vector3d p(msg->ranges.at( index ), 0, 0);
            AngleAxis<double> rotation(angle, Vector3d(0, 0, -1));
            p = rotation * p;

            //shifting scan
            p -= shift_distance;

            v_sum = p;

            d_sum = p.norm();

        }
       // k++;
    //}

    goal = v_sum ;
}
