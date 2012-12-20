#include "wall_follow_fancy_algo.h"

using namespace Eigen;


wall_follow_fancy_algo::wall_follow_fancy_algo(){
#ifdef DEBUG
    // init wall follow node
    char *argv[] = {}; int argc = 0;
    ros::init(argc, argv, "debug_shift");

    //create NodeHandle
    ros::NodeHandle n;
    pub_all = n.advertise<geometry_msgs::PolygonStamped>("/debug_fancy_poly", 1000);
    pub_path = n.advertise<geometry_msgs::PolygonStamped>("/debug_fancy_path", 1000);

#endif //DEBUG
}

void wall_follow_fancy_algo::sonar_laser_update(
        const geometry_msgs::PolygonStamped::ConstPtr& msg,
        const geometry_msgs::Pose& pose,
        Vector3d &goal,
        Quaterniond &orientation) throw (std::runtime_error)
{

    Vector3d robot_position(pose.position.x, pose.position.y, pose.position.z);
    Quaterniond robot_oriantation(pose.orientation.w,
                                  pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z);
    Vector3d robot_rot;
    robot_rot = robot_oriantation * Vector3d::UnitX();
    double robot_yaw_angle = atan2(robot_rot(1), robot_rot(0));


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



        for (std::list<Vector3d>::iterator it = pCirc.begin(); it != pCirc.end(); ) {
            // remove all points that lie inside other circles
            Vector3d p = *it;
            for (Vector3d q : points) {
                if ((q - p).norm() < distance - tolerance) {
                    it = pCirc.erase(it);
                    goto next_point;
                }
            }

            //remove all points that are behind the robot
            {
                //calculate position relative to robot
                p -= robot_position;
                double p_angle = atan2(p(1), p(0));
                double diff = robot_yaw_angle - p_angle;
                //throw point away if it is behind
                if(fabs(diff) > M_PI/2 ){
                    it = result.erase(it);
                    goto next_point;
                }
            }

            //go to next element
            it++;
            next_point:;
        }
        result.insert(result.end(), pCirc.begin(), pCirc.end());

    }

    //searching for nearest point
    std::vector<Vector3d> nearest_points;
    Vector3d nearest_point_to_robot = get_nearest(result, robot_position, nearest_points);
    nearest_points.push_back(nearest_point_to_robot);
    for(unsigned int i = 0; i < 150; i++){
        nearest_points.push_back(
            get_nearest(result, *(--nearest_points.end()), nearest_points));
    }


    orientation = Quaterniond(1,0,0,0);
    goal = Vector3d();

#ifdef DEBUG
    publish_debug_info(result, nearest_points);
#endif //DEBUG

}


Vector3d wall_follow_fancy_algo::get_nearest(const std::list<Vector3d> &list, const Vector3d &ref_point, std::vector<Vector3d> &nearest_points){
    Vector3d nearestPoint(DBL_MAX, DBL_MAX, DBL_MAX);
    for(Vector3d p: list){
        if((nearestPoint - ref_point).squaredNorm() > (p - ref_point).squaredNorm()
                && ref_point != p
                && std::find(nearest_points.begin(),nearest_points.end(),p)==nearest_points.end()){
            nearestPoint = p;
        }
    }
    return nearestPoint;
}


#ifdef DEBUG
void wall_follow_fancy_algo::publish_debug_info(const std::list<Vector3d> &all,const std::vector<Vector3d> &path){
{
    //create polygon from valid points
    std::vector<geometry_msgs::Point32> debug_points;
    for(Vector3d point: all){
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
    pub_all.publish(spolygon);
}

    //create polygon from valid points
    std::vector<geometry_msgs::Point32> debug_points;
    for(Vector3d point: path){
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
    pub_path.publish(spolygon);
}

//std::vector<geometry_msgs::Point32> wall_follow_fancy_algo::to_point32_vector
//                                                (const std::iterator &begin, const std::iterator &end){
//    std::vector<geometry_msgs::Point32> points;
//    for(std::iterator it = begin; it!=end; it++){
//        points.push_back(*it);
//    }
//    return points;
//}

#endif //DEBUG
