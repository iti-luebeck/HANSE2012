#include "wall_follow_fancy_algo.h"

using namespace Eigen;


WallFollowFancyAlgo::WallFollowFancyAlgo(){
#ifdef DEBUG
    // init wall follow node
    char *argv[] = {}; int argc = 0;
    ros::init(argc, argv, "debug_fancy");

    //create NodeHandle
    ros::NodeHandle n;
    pub_all_ = n.advertise<geometry_msgs::PolygonStamped>("/debug_fancy_poly", 1000);
    pub_path_ = n.advertise<geometry_msgs::PolygonStamped>("/debug_fancy_path", 1000);

#endif //DEBUG
}

void WallFollowFancyAlgo::sonarLaserUpdate(
        const geometry_msgs::PolygonStamped::ConstPtr& msg,
        const geometry_msgs::Pose& pose,
        Vector3d &goal,
        Quaterniond &orientation) throw (std::runtime_error)
{

    const Vector3d robot_position(pose.position.x, pose.position.y, pose.position.z);
    const Quaterniond robot_oriantation(pose.orientation.w,
                                  pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z);
    Vector3d robot_rot;
    robot_rot = robot_oriantation * Vector3d::UnitX();
    const double robot_yaw_angle = atan2(robot_rot(1), robot_rot(0));


    //create vector of Vector3d for global sonar points that are in front of the robot
    std::vector<Vector3d> global_front_sonar_points;
    for(const geometry_msgs::Point32 &p : msg->polygon.points){
        Vector3d vp(p.x, p.y, p.z);
        if(!isBehindRobot(vp, robot_yaw_angle, robot_position)){
            //add point if it isn't behind the robot
            global_front_sonar_points.push_back(vp);
        }
    }


    // create std::vector<Vector3d> of points in a circle
    const unsigned int step = 2; //steps of point in circe (degree)
    const double distance = 7; //radius of the circle
    std::vector<Vector3d> circ;
    for (unsigned int i = 0; i < 360; i+=step) {
        const Vector3d dVector = distance * Vector3d::UnitX();
        AngleAxisd rotate(angles::from_degrees((double) i), Vector3d::UnitZ());
        circ.push_back(rotate * dVector);
    }

    double tolerance = 0; //to check if points lie within other circles
    std::list<Vector3d> result;
    for (const auto &p: global_front_sonar_points) {
        //create point circle for current point
        std::list<Vector3d> pCirc;
        for (const auto &c:circ) {
            Vector3d circ_point = c + p;
            pCirc.push_back(circ_point);
        }

        //filter unwanted points
        for (std::list<Vector3d>::iterator it = pCirc.begin(); it != pCirc.end(); ) {
            // remove all points that lie inside other circles
            const Vector3d &pc(*it);
            if(isInsideOtherCircle(distance, global_front_sonar_points, pc, tolerance)){
                //remove point if it lies inside another circle
                it = pCirc.erase(it);
            } else if(isBehindRobot(pc, robot_yaw_angle, robot_position)){
                //remove point if it is behind a robot
                it = pCirc.erase(it);
            } else {
                //keep point and continue with next point
                it++;
            }
        }
        result.insert(result.end(), pCirc.begin(), pCirc.end());
    }


    //Greedy Algorithm to calculate goal point and orientation

    //getting a list of points by looking at the next points that are close to each other
    //limit by a limit_lookahead_distancee, limit_lookahead_index_delta and limit_point_sdistance
    //TODO ensure that this algorithm won't go backward!
    const double limit_lookahead_sdistance = DBL_MAX;
    const unsigned int limit_lookahead_index_delta = 40;
    const double limit_point_sdistance = pow(6, 2);

    std::vector<Vector3d> nearest_point_list = {robot_position};
    for(unsigned int i = 0; i < limit_lookahead_index_delta; i++){
        //use last point in nearest_point_list as reference point
        const Vector3d &ref_point(*(--nearest_point_list.end()));
        //initialize
        Vector3d const *nearestPoint = NULL;
        double nearest_sdistance = DBL_MAX;
        //search nearest point
        for(const Vector3d &p: result){
            const double p_sdistance = (p - ref_point).squaredNorm();
            if(p_sdistance < nearest_sdistance //is nearer
                    && (p-robot_position).squaredNorm() < limit_lookahead_sdistance //check lookahead distance limit
                    && std::find(nearest_point_list.begin(),nearest_point_list.end(),p)==nearest_point_list.end() //isn't already the nearest point list
                    && p_sdistance < limit_point_sdistance //check if distance between the two points is small enough
                    ){
                nearestPoint = &p;
                nearest_sdistance = (*nearestPoint - ref_point).squaredNorm();
            }
        }
        //check if we found a new Point
        if(nearest_sdistance != DBL_MAX){
            nearest_point_list.push_back(*nearestPoint);
        } else {
            //give up because there aren't any closer points
            break;
        }
    }
    //remove robot position
    nearest_point_list.erase(nearest_point_list.begin());

    //
    Vector3d sum(0, 0, 0);
    for(const auto &p:nearest_point_list){
        sum += p - robot_position;
    }


    goal = sum / sum.norm() + robot_position;


    orientation = AngleAxisd(atan2(sum(1), sum(0)), Vector3d::UnitZ());


#ifdef DEBUG
    publishDebugInfo(result, nearest_point_list);
#endif //DEBUG

}

bool WallFollowFancyAlgo::isBehindRobot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position)
{
    //calculate position relative to robot
    Vector3d p_reltative = p - robot_position;
    double p_angle = atan2(p_reltative(1), p_reltative(0));
    double diff = robot_yaw_angle - p_angle;
    //return true if the point p is behind
    return (fabs(diff) > M_PI/2 );
}

bool WallFollowFancyAlgo::isInsideOtherCircle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &pc, const double &tolerance)
{
    for (const Vector3d &q : global_sonar_points) {
        if ((q - pc).norm() < distance - tolerance) {
            return true;
        }
    }
    return false;
}


#ifdef DEBUG
void WallFollowFancyAlgo::publishDebugInfo(const std::list<Vector3d> &all,const std::vector<Vector3d> &path){
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();

    //create polygon from valid points
    for(const Vector3d &point: all){
        geometry_msgs::Point32 p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        spolygon.polygon.points.push_back(p);
    }
    pub_all_.publish(spolygon);


    //create polygon from valid points
    spolygon.polygon.points.clear();
    for(const Vector3d &point: path){
        geometry_msgs::Point32 p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);

        spolygon.polygon.points.push_back(p);
    }
    pub_path_.publish(spolygon);
}

#endif //DEBUG
