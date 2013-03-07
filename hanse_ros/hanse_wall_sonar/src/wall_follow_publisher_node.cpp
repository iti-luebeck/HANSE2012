#include "wall_follow_publisher_node.h"

WallFollowPublisherNode::WallFollowPublisherNode(ros::NodeHandle n):node_(n),last_publish_time_(0)
{
    //Setup publisher and subscriber
    sub_goal_ = node_.subscribe<geometry_msgs::PoseStamped>("sonar/wall_follow/goal", 1000, &WallFollowPublisherNode::updateGoal, this);
    pub_goal_ = node_.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
}

void WallFollowPublisherNode::configCallback(hanse_wall_sonar::wall_follow_publisher_node_paramsConfig &config, uint32_t level){
    //overwrite old config
    this->config_ = config;
}

void WallFollowPublisherNode::updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store received pose in goal list
    last_goals_.push_front(msg->pose);

    //check if it is time to publish a new goal, and if enough points are available
    if((ros::Time::now().sec > last_publish_time_ + config_.publish_time_)
            &&(last_goals_.size() >= (unsigned int) config_.number_of_last_points_)){
        std::list<geometry_msgs::Pose>::iterator it = last_goals_.begin();
        //create stamped pose
        geometry_msgs::PoseStamped spose;
        spose.header.frame_id = "/map";
        spose.header.stamp = ros::Time::now();
        //sum up points
        spose.pose = *(it++);
        for(int i=1; i<config_.number_of_last_points_; i++){
            spose.pose.position.x += (*it).position.x;
            spose.pose.position.y += (*it).position.y;
            spose.pose.position.z += (*it).position.z;

            spose.pose.orientation.w += (*it).orientation.w;
            spose.pose.orientation.x += (*it).orientation.x;
            spose.pose.orientation.y += (*it).orientation.y;
            spose.pose.orientation.z += (*it).orientation.z;
            it++;
        }
        //calculate average point
        spose.pose.position.x /= config_.number_of_last_points_;
        spose.pose.position.y /= config_.number_of_last_points_;
        spose.pose.position.z /= config_.number_of_last_points_;

        spose.pose.orientation.w /= config_.number_of_last_points_;
        spose.pose.orientation.x /= config_.number_of_last_points_;
        spose.pose.orientation.y /= config_.number_of_last_points_;
        spose.pose.orientation.z /= config_.number_of_last_points_;

        //publish goal
        pub_goal_.publish(spose);

        //clear list of last goals
        last_goals_.clear();
        //update last publish time
        last_publish_time_ = ros::Time::now().sec;
    }
}

int main(int argc, char **argv)
{

    // init wall follow node
    ros::init(argc, argv, "wall_follow_publisher_node");

    //create NodeHandle
    ros::NodeHandle n;

    //create node object
    WallFollowPublisherNode wall_follow_publisher_node(n);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_publisher_node_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_publisher_node_paramsConfig>::CallbackType cb;
    cb = boost::bind(&WallFollowPublisherNode::configCallback, &wall_follow_publisher_node, _1, _2);
    dr_srv.setCallback(cb);


    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
