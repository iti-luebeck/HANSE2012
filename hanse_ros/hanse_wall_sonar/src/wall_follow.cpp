#include "wall_follow.h"
//! \todo{TODO allow reconfiguration to use without mars}
//! \todo{TODO subscriber in klasse auslagern?}
//! \todo{TODO Memory management?}

WallFollowNode::WallFollowNode(ros::NodeHandle n) : node_(n) {
    WallFollowNode::setupSubscribers();

   //advertise to navigation goal
    pub_ = node_.advertise<geometry_msgs::PoseStamped>("/goal", 1000);

    debug_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/debug/goal", 1000);

    debug_laser_pub_ = node_.advertise<geometry_msgs::PolygonStamped>("/debug/sonarPolygon", 1000);

    //TODO: think about memory ;)
    //algo = new wall_follow_shift_algo();
    algo_ = new WallFollowFancyAlgo();



    ROS_INFO("Wall follow node initialized");

}



void WallFollowNode::gSonarUpdate(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    Vector3d goal;
    Quaterniond goal_orientation;
    try {
       algo_->sonarLaserUpdate(msg, last_pose_, goal, goal_orientation);
    } catch (std::runtime_error e) {
        ROS_ERROR_STREAM(e.what() << " Skipping...");
        return;
    }


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

    //limit publish rate
    if(ros::Time::now().sec > config_.goal_publish_rate_ + last_goal_update_){
        pub_.publish(spose);
        debug_pub_.publish(spose);

        last_goal_update_ = ros::Time::now().sec;
    }
}

void WallFollowNode::posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last pose
    last_pose_ = msg->pose;
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow");

    //create NodeHandle
    ros::NodeHandle n;
    
    WallFollowNode follow(n);


    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_paramsConfig>::CallbackType cb;
    cb = boost::bind(&WallFollowNode::configCallback, &follow, _1, _2);
    dr_srv.setCallback(cb);

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}

void WallFollowNode::configCallback(hanse_wall_sonar::wall_follow_paramsConfig &config, uint32_t level){
    bool old_sim_mode = this->config_.simulation_mode_;
    this->config_ = config;
    if (config.simulation_mode_ != old_sim_mode){
        setupSubscribers();
    }
}

void WallFollowNode::setupSubscribers(){
    sub_pos_.shutdown();
    sub_laser_.shutdown();

    //Subscribe to topic laser_scan (from sonar)
    sub_laser_ = node_.subscribe<geometry_msgs::PolygonStamped>("sonar/global_sonar/polygon", 1000, &WallFollowNode::gSonarUpdate, this);

    if(config_.simulation_mode_){
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("posemeter", 1000, &WallFollowNode::posUpdate, this);
    }else {
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("position/estimate", 1000, &WallFollowNode::posUpdate, this);
    }
}


