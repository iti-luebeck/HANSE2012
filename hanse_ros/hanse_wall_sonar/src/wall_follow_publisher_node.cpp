#include "wall_follow_publisher_node.h"

WallFollowPublisherNode::WallFollowPublisherNode(ros::NodeHandle n):node_(n),last_publish_time_(0)
{
    sub_goal_ = node_.subscribe<geometry_msgs::PoseStamped>("sonar/wall_follow/goal", 1000, &WallFollowPublisherNode::updateGoal, this);

    pub_goal_ = node_.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
}

void WallFollowPublisherNode::configCallback(hanse_wall_sonar::wall_follow_publisher_node_paramsConfig &config, uint32_t level){
    this->config_ = config;
}

void WallFollowPublisherNode::updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_goals_.push_front(msg->pose);

    if((ros::Time::now().sec > last_publish_time_ + config_.publish_time_)
            &&(last_goals_.size() >= (unsigned int) config_.number_of_last_points_)){
        //average of the last points
        std::list<geometry_msgs::Pose>::iterator it = last_goals_.begin();
        geometry_msgs::Pose avg_pose;
        avg_pose = *(it++);
        for(int i=1; i<config_.number_of_last_points_; i++){
            avg_pose.position.x += (*it).position.x;
            avg_pose.position.y += (*it).position.y;
            avg_pose.position.z += (*it).position.z;

            avg_pose.orientation.w += (*it).orientation.w;
            avg_pose.orientation.x += (*it).orientation.x;
            avg_pose.orientation.y += (*it).orientation.y;
            avg_pose.orientation.z += (*it).orientation.z;
            it++;
        }

        avg_pose.position.x /= config_.number_of_last_points_;
        avg_pose.position.y /= config_.number_of_last_points_;
        avg_pose.position.z /= config_.number_of_last_points_;

        avg_pose.orientation.w /= config_.number_of_last_points_;
        avg_pose.orientation.x /= config_.number_of_last_points_;
        avg_pose.orientation.y /= config_.number_of_last_points_;
        avg_pose.orientation.z /= config_.number_of_last_points_;


        geometry_msgs::PoseStamped spose;
        spose.header.frame_id = "/map";
        spose.header.stamp = ros::Time::now();

        spose.pose = avg_pose;

        pub_goal_.publish(spose);


        last_goals_.clear();

        last_publish_time_ = ros::Time::now().sec;
    }
}

int main(int argc, char **argv)
{

    // init wall follow node
    ros::init(argc, argv, "wall_follow_publisher_node");

    //create NodeHandle
    ros::NodeHandle n;

    WallFollowPublisherNode wall_follow_publisher_node(n);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_publisher_node_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_publisher_node_paramsConfig>::CallbackType cb;
    cb = boost::bind(&WallFollowPublisherNode::configCallback, &wall_follow_publisher_node, _1, _2);
    dr_srv.setCallback(cb);

    //TODO load parameters for dyn reconfigure


    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
