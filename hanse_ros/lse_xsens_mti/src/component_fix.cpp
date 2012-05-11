#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ComponentFix {
public:
    ComponentFix(ros::NodeHandle handle) :
	nh(handle),
	publisher(handle.advertise<sensor_msgs::Imu>("imu/component_fix", 10)),
	subscriber(handle.subscribe("imu", 10, &ComponentFix::callback, this))
    {
	ros::param::param("~unfix", unfix, false);
    }
private:
    bool unfix;
    void callback(const sensor_msgs::Imu &msg) {
	sensor_msgs::Imu newMsg = msg;
	if (unfix) {
	    newMsg.orientation.x = msg.orientation.w;
	    newMsg.orientation.y = msg.orientation.x;
	    newMsg.orientation.z = msg.orientation.y;
	    newMsg.orientation.w = msg.orientation.z;
	} else {
	    newMsg.orientation.w = msg.orientation.x;
	    newMsg.orientation.x = msg.orientation.y;
	    newMsg.orientation.y = msg.orientation.z;
	    newMsg.orientation.z = msg.orientation.w;
	}
	publisher.publish(newMsg);
    }
    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "component_fix");
    ros::NodeHandle n;
    ComponentFix componentFix(n);

    ros::spin();
    return 0;
}
