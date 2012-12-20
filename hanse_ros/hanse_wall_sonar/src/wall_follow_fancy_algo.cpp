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

    Vector3d robot_position(pose.position.x, pose.position.y, pose.position.z);

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

    double tolerance = 0.5;
    std::list<Vector3d> result;
    for (unsigned int i = 0; i < points.size(); i++) {
        std::list<Vector3d> pCirc;
        for (unsigned int j = 0; j < circ.size(); j++) {
            Vector3d circ_point = circ[j] + points[i];
            //too greedy filter
            //if((robot_position - points[i]).norm() > (robot_position - circ_point).norm())
                pCirc.push_back(circ_point);
        }


        // remove all points that lie inside other circles
        for (std::list<Vector3d>::iterator it = pCirc.begin(); it != pCirc.end(); ) {
            Vector3d p = *it;
            for (Vector3d q : points) {
                if ((q - p).norm() < distance - tolerance) {
                    it = pCirc.erase(it);
                    goto next_point1;
                }
            }
            it++;
            next_point1:;
        }
        result.insert(result.end(), pCirc.begin(), pCirc.end());

    }


    std::list<Vector3d> all_points;
    all_points.insert(all_points.end(), result.begin(), result.end());
    all_points.insert(all_points.end(), points.begin(), points.end());

    double output_stepsize = angles::from_degrees(3);
    for (std::list<Vector3d>::iterator it = result.begin(); it != result.end(); ) {
        Vector3d p = *it;
        p -= robot_position;
        int p_angle = (int) round(atan2(p(1), p(0)) / output_stepsize);


        for (Vector3d q : all_points) {
            q -= robot_position;
            int q_angle = (int) round(atan2(q(1), q(0)) / output_stepsize);
            if(q_angle == p_angle){
                if((p - robot_position).norm() > (q - robot_position).norm()){
                    it = result.erase(it);
                    goto next_point2;
                }
            }
        }
        it++;
        next_point2:;
    }


    orientation = Quaterniond(1,0,0,0);
    goal = Vector3d();

#ifdef DEBUG
    publish_debug_info(result);
#endif //DEBUG

}

#ifdef DEBUG
void wall_follow_fancy_algo::publish_debug_info(const std::list<Vector3d> &point_list){
    //create polygon from valid points
    std::vector<geometry_msgs::Point32> debug_points;
    for(Vector3d point: point_list){
        geometry_msgs::Point32 p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);

        debug_points.push_back(p);
    }

    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = debug_points;
    pub.publish(spolygon);
}
#endif //DEBUG
