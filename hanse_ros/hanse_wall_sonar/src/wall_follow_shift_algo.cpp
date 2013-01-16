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


wall_follow_shift_algo::wall_follow_shift_algo(){
#ifdef DEBUG
    // init wall follow node
    char *argv[] = {}; int argc = 0;
    ros::init(argc, argv, "debug_shift");

    //create NodeHandle
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PolygonStamped>("/debug_shift_poly", 1000);
#endif //DEBUG
}

void wall_follow_shift_algo::sonar_laser_update(
        const geometry_msgs::PolygonStamped::ConstPtr& msg,
        const geometry_msgs::Pose& pose,
        Vector3d &goal,
        Quaterniond &orientation) throw (std::runtime_error)
{

    Vector3d shift_distance(0, -3, 0);
    Quaterniond robot_orientation(pose.orientation.w,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z);
    Vector3d global_shift;
    global_shift = robot_orientation * shift_distance;

    //create vector containing all shifted points
    std::vector<Vector3d> shifted_points;
    for(unsigned int i = 0; i < msg->polygon.points.size(); i++){
        Vector3d p(msg->polygon.points[i].x,
                   msg->polygon.points[i].y,
                   msg->polygon.points[i].z);
        shifted_points.push_back(p + global_shift);
    }
#ifdef DEBUG
    publish_debug_info(shifted_points);
#endif //DEBUG

    //searching for nearest point
    double min_dist = DBL_MAX;
    unsigned int nearest_point_index = -1;
    for(unsigned int i = 0; i < shifted_points.size(); i++){
        double square_distance = pow(pose.position.x - shifted_points[i](0), 2) + pow(pose.position.y - shifted_points[i](1), 2);
        if (square_distance <= min_dist){
            min_dist = square_distance;
            nearest_point_index = i;
        }
    }

    //find last point to look at
    //limited by a lookahead distance and lookahead distance
    double limit_lookahead_distance = 10;
    unsigned int limit_lookahead_index_delta = 20;

    unsigned int last_point_index = nearest_point_index;
    for(unsigned int q = 0; q <= limit_lookahead_index_delta; q++){
        double distance = (shifted_points[nearest_point_index] - shifted_points[last_point_index + 1]).norm();
        if(distance < limit_lookahead_distance){
            last_point_index = (last_point_index + 1) % shifted_points.size();
        } else {
            break;
        }
    }


    //sum up points
    for(unsigned int q = nearest_point_index; q <= last_point_index; q++){
       goal += shifted_points[q];
    }
    goal /= goal.norm();
    goal *= shifted_points[last_point_index].norm();


    //calculate orientation
    Vector3d path;
    path = shifted_points[last_point_index] - shifted_points[nearest_point_index];
    orientation = AngleAxisd(atan2(path(1), path(0)), Vector3d::UnitZ());
}

#ifdef DEBUG
void wall_follow_shift_algo::publish_debug_info(const std::vector<Vector3d> &shifted_points){
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
