#include "wall_follow_fancy_algo.h"

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


wall_follow_fancy_algo::wall_follow_fancy_algo(){
#ifdef DEBUG
    // init wall follow node
    char *argv[] = {}; int argc = 0;
    ros::init(argc, argv, "debug_shift");

    //create NodeHandle
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PolygonStamped>("/debug_shift_poly", 1000);
#endif //DEBUG
}

void wall_follow_fancy_algo::sonar_laser_update(
        const geometry_msgs::PolygonStamped::ConstPtr& msg,
        const geometry_msgs::Pose& pose,
        Vector3d &goal,
        Quaterniond &orientation) throw (std::runtime_error)
{

    //create
    std::vector<Vector3d> points;
    for(geometry_msgs::Point32 p : msg->polygon.points){
        Vector3d vp(p.x, p.y, p.z);
        points.push_back(vp);
    }


    // create point array for circle
    unsigned int step = 10;
    double distance = 3;
    std::vector<Vector3d> circ;
    for (unsigned int i = 0; i < 360 / step; i++) {
        const Vector3d dVector = distance * Vector3d::UnitX();
        AngleAxisd rotate(2 * M_PI * ((double) i*10)/360, Vector3d::UnitZ());
        circ.push_back(rotate * dVector);
    }

    double tolerance = 0;
    std::vector<Vector3d> result;
    for (unsigned int i = 0; i < points.size(); i++) {
        std::vector<Vector3d> pCirc;
        for (unsigned int j = 0; j < circ.size(); j++) {
//            // if (pSonar.get(i).getR() > n.getR())
            pCirc.push_back(circ[j] + points[i]);

        }


        // remove all points that lie inside other circles
        for (std::vector<Vector3d>::iterator it = pCirc.begin(); it != pCirc.end(); ) {
            Vector3d p = *it;
            for (Vector3d q : points) {
                if ((q - p).norm() < distance - tolerance) {
                    pCirc.erase(it);
                    goto next_point;
                }
            }
            it++;
            next_point:;
        }
        result.insert(result.end(), pCirc.begin(), pCirc.end());

    }

    orientation = Quaterniond(1,0,0,0);
    goal = Vector3d();

#ifdef DEBUG
    publish_debug_info(result);
#endif //DEBUG

}

#ifdef DEBUG
void wall_follow_fancy_algo::publish_debug_info(const std::vector<Vector3d> &shifted_points){
    //create polygon from valid points
    std::vector<geometry_msgs::Point32> debug_points;
    for(unsigned int i = 0; i < shifted_points.size(); i++){
        geometry_msgs::Point32 p;
        p.x = shifted_points[i](0);
        p.y = shifted_points[i](1);
        p.z = shifted_points[i](2);

        debug_points.push_back(p);
    }

    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = debug_points;
    pub.publish(spolygon);
}
#endif //DEBUG
