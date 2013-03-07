#include "wall_follow_shift_node.h"

using namespace Eigen;

WallFollowShiftNode::WallFollowShiftNode(ros::NodeHandle n):node_(n){

    pub_goal_ = node_.advertise<geometry_msgs::PoseStamped>("sonar/wall_follow/goal", 1000);
    pub_poly_ = node_.advertise<geometry_msgs::PolygonStamped>("sonar/shift/poly", 1000);

    setupSubscribers();

}

void WallFollowShiftNode::configCallback(hanse_wall_sonar::wall_follow_shift_node_paramsConfig &config, uint32_t level){
    bool old_sim_mode = this->config_.simulation_mode_;
    this->config_ = config;
    if (config.simulation_mode_ != old_sim_mode){
        setupSubscribers();
    }
}

void WallFollowShiftNode::sonarLaserUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error)
{

    Vector3d shift_distance(0, config_.wall_distance_right_, 0);
    Quaterniond robot_orientation(last_pose_.orientation.w,
                            last_pose_.orientation.x,
                            last_pose_.orientation.y,
                            last_pose_.orientation.z);
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

    publishDebugInfo(shifted_points);

    //searching for nearest point
    double min_dist = DBL_MAX;
    unsigned int nearest_point_index = -1;
    for(unsigned int i = 0; i < shifted_points.size(); i++){
        double square_distance = pow(last_pose_.position.x - shifted_points[i](0), 2) + pow(last_pose_.position.y - shifted_points[i](1), 2);
        if (square_distance <= min_dist){
            min_dist = square_distance;
            nearest_point_index = i;
        }
    }

    //find last point to look at
    //limited by a lookahead distance and lookahead index delta
    unsigned int last_point_index = nearest_point_index;
    for(int q = 0; q <= config_.limit_lookahead_index_delta_; q++){
        double distance = (shifted_points[nearest_point_index] - shifted_points[last_point_index + 1]).norm();
        if(distance < config_.limit_lookahead_distance_){
            last_point_index = (last_point_index + 1) % shifted_points.size();
        } else {
            break;
        }
    }

    Vector3d goal(0,0,0);

    //sum up points
    for(unsigned int q = nearest_point_index; q <= last_point_index; q++){
       goal += shifted_points[q];
    }
    goal /= goal.norm();
    goal *= shifted_points[last_point_index].norm();


    //calculate orientation
    Quaterniond goal_orientation;
    Vector3d path;
    path = shifted_points[last_point_index] - shifted_points[nearest_point_index];
    goal_orientation = AngleAxisd(atan2(path(1), path(0)), Vector3d::UnitZ());

    geometry_msgs::PoseStamped spose;
    spose.header.frame_id = "/map";
    spose.header.stamp = ros::Time::now();

    spose.pose.position.x = goal(0);
    spose.pose.position.y = goal(1);
    spose.pose.position.z = goal(2);
    spose.pose.orientation.x = goal_orientation.x();
    spose.pose.orientation.y = goal_orientation.y();
    spose.pose.orientation.z = goal_orientation.z();
    spose.pose.orientation.w = goal_orientation.w();



    pub_goal_.publish(spose);
}

void WallFollowShiftNode::publishDebugInfo(const std::vector<Vector3d> &shifted_points){
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
    pub_poly_.publish(spolygon);
}

void WallFollowShiftNode::setupSubscribers(){
    sub_pos_.shutdown();
    sub_global_sonar_.shutdown();

    //Subscribe to topic laser_scan (from sonar)
    sub_global_sonar_ = node_.subscribe<geometry_msgs::PolygonStamped>("sonar/global_sonar/polygon", 1000, &WallFollowShiftNode::sonarLaserUpdate, this);

    if(config_.simulation_mode_){
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("posemeter", 1000, &WallFollowShiftNode::posUpdate, this);
    }else {
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("position/estimate", 1000, &WallFollowShiftNode::posUpdate, this);
    }
}

void WallFollowShiftNode::posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last pose
    last_pose_ = msg->pose;
}

int main(int argc, char **argv)
{

    // init wall follow node
    ros::init(argc, argv, "wall_follow_shift_node");

    //create NodeHandle
    ros::NodeHandle n;

    WallFollowShiftNode wall_follow_shift_node(n);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_shift_node_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_shift_node_paramsConfig>::CallbackType cb;
    cb = boost::bind(&WallFollowShiftNode::configCallback, &wall_follow_shift_node, _1, _2);
    dr_srv.setCallback(cb);

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
