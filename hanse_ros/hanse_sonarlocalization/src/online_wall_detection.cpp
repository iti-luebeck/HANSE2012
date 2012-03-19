#include "angles/angles.h"
#include "sensor_msgs/LaserScan.h"
#include "online_wall_detection.h"

OnlineWallDetection::OnlineWallDetection(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::LaserScan>("sonar/laser_scan", 1)),
    subscriber(handle.subscribe("sonar/scan", 1, &OnlineWallDetection::callback, this))
{
}

void OnlineWallDetection::callback(const hanse_msgs::ScanningSonar &msg)
{
    WallDistance d = computeWallDistance(msg);
    sensor_msgs::LaserScan laserScan;
    laserScan.header.stamp = msg.header.stamp;
    laserScan.header.frame_id = "/map";
    laserScan.angle_min = d.headPosition;
    laserScan.angle_max = d.headPosition + 0.001;
    laserScan.angle_increment = 0.001;
    laserScan.time_increment = 0;
    laserScan.scan_time = 0;
    laserScan.range_min = 0.15;
    laserScan.range_max = msg.range;
    laserScan.ranges.push_back(d.distance);
    publisher.publish(laserScan);

}

OnlineWallDetection::WallDistance OnlineWallDetection::computeWallDistance(const hanse_msgs::ScanningSonar &msg)
{
    WallDistance distance;
    distance.headPosition = angles::normalize_angle(msg.headPosition);
    int dataPoints = msg.echoData.size();
    distance.distance = -1;
    for (int i = 0; i < dataPoints; i++) {
	if (msg.echoData[i] > 32) {
	    distance.distance = ((double)i / (dataPoints-1)) * (msg.range - 0.15) + 0.15;
	    break;
	}
    }
    return distance;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sonar_online_wall_detection");
    ros::NodeHandle n;

    OnlineWallDetection wallDetection(n);

    ros::spin();

    return 0;
}
