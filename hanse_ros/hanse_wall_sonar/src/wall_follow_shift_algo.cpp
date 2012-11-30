#include "wall_follow_shift_algo.h"


using namespace Eigen;
using std::set;

class CompPointsX
{
public:
    bool operator()(Vector2d v1, Vector2d v2)
    {
        if(v1(0) < v2(0))
            return true;
        else
            return false;
    }
};

void wall_follow_shift_algo::sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg,
        Eigen::Vector2d &goal)
{
    set<Vector2d, CompPointsX> shifted_sonar_points;
    Vector2d shift_distance(0,0);

    for (unsigned int i = 0; i < msg->ranges.size(); i++){
        double angle = msg->angle_min + msg->angle_increment * i;
        if(msg->ranges.at(i) > 0){
            //calculating x, y coordinates of laser scan
            Vector2d p(0, 0);
            p.setPolar(msg->ranges.at(i), angle);

            //shifting scan
            p -= shift_distance;
            //only add vectors with positive x coordinate
            //follow right wall
            if(p(0) > 0 && p(1) < 0){
                shifted_sonar_points.insert(p);
            }
        }
    }

    Vector2d sum(0,0);
    set<Vector2d>::iterator iterator = shifted_sonar_points.begin();
    double last_r;
    for(unsigned int i = 0; i < 5; i++){
        sum += *iterator;
        last_r = (*iterator).getR();
        iterator++;
    }


    sum.setR(last_r);

    std::cout << "debug result:\n";
    std::cout << sum;
    std::cout << "\n\n";

    goal = sum;

}
