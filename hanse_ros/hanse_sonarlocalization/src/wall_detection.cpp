#include "sensor_msgs/LaserScan.h"
#include "angles/angles.h"
#include "wall_detection.h"


WallDetection::WallDetection(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::LaserScan>("sonar/laser_scan", 1)),
    //publisher for enhanced laser scan (add updated indexes)
    ePublisher(handle.advertise<hanse_msgs::ELaserScan>("sonar/e_laser_scan", 1)),
    subscriber(handle.subscribe("sonar/scan", 1, &WallDetection::callback, this)),
    lastHeadPosition(0),
    movedSincePublish(0),
    isInitialized(false),
    movedSincePublishELaserscan(0)
{
    publishAngle = M_PI / 5;
    stepSize = (2 * M_PI) / 60;

    for (int i = -180; i < 180; i++) {
        float f = angles::from_degrees(i);
        WallDistance distance;
        distance.headPosition = f;
        distance.distance = -1;
        sonarDataMap.insert(std::make_pair(f, distance));
    }

}

void WallDetection::callback(const hanse_msgs::ScanningSonar &msg)
{
    // do initialization on first run
    if (!isInitialized){
        init();
    }
    double headPosition = angles::normalize_angle(msg.headPosition);
    double posDiff = angles::normalize_angle(lastHeadPosition - headPosition);
    double posLow, posHigh;
    if (posDiff < 0) {
        posLow = lastHeadPosition;
        posHigh = headPosition;
    } else {
        posLow = headPosition;
        posHigh = lastHeadPosition;
    }

    if (posHigh > posLow) {
        sonarDataMap.erase(sonarDataMap.upper_bound(posLow), sonarDataMap.lower_bound(posHigh));
    } else {
        sonarDataMap.erase(sonarDataMap.upper_bound(posLow), sonarDataMap.end());
        sonarDataMap.erase(sonarDataMap.begin(), sonarDataMap.lower_bound(posHigh));
    }

    sonarDataMap.erase(headPosition);
    sonarDataMap.insert(std::make_pair(headPosition, computeWallDistance(msg)));

    movedSincePublish += fabs(posDiff);
    lastHeadPosition = headPosition;

    if (movedSincePublish > publishAngle) {
        movedSincePublish -= publishAngle;
        publishLaserScan(msg.header.stamp);
    }

    movedSincePublishELaserscan += fabs(posDiff);
    if (movedSincePublishELaserscan > stepSize) {
        movedSincePublishELaserscan -= stepSize;
        publishELaserScan(msg.header.stamp);
    }
}

WallDetection::WallDistance WallDetection::computeWallDistance(const hanse_msgs::ScanningSonar &msg)
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

void WallDetection::publishLaserScan(ros::Time stamp)
{
    //create LaserScan message
    sensor_msgs::LaserScan laserScan = createLaserScan(stamp);

    ROS_INFO("tick");
    publisher.publish(laserScan);
}

sensor_msgs::LaserScan WallDetection::createLaserScan(ros::Time stamp){
    //create message
    sensor_msgs::LaserScan laserScan;
    
    //create header
    laserScan.header.stamp = stamp;
    laserScan.header.frame_id = "/map";
    
    //create laser scan message
    laserScan.angle_min = -M_PI;
    laserScan.angle_max = M_PI;
    laserScan.angle_increment = stepSize;
    laserScan.time_increment = 0;
    laserScan.scan_time = 0;
    laserScan.range_min = 0.15;
    laserScan.range_max = 50;

    for (double a = -M_PI; a < M_PI; a += stepSize) {
        auto iPos = sonarDataMap.lower_bound(a);
        if (iPos == sonarDataMap.end())
            iPos = sonarDataMap.begin();
        double dPos = angles::normalize_angle_positive(iPos->first - a);
        auto iNeg = iPos;
        if (iNeg == sonarDataMap.begin())
            iNeg = sonarDataMap.end();
        iNeg--;
        double dNeg = angles::normalize_angle_positive(a - iNeg->first);

        auto i = dPos > dNeg ? iNeg : iPos;

        laserScan.ranges.push_back(i->second.distance);
    }
    
    return laserScan;
}

void WallDetection::publishELaserScan(ros::Time stamp)
{
    //create LaserScan message
    sensor_msgs::LaserScan laserScan = createLaserScan(stamp);
    
    //create enhance laser scan message
    hanse_msgs::ELaserScan escan;
    //fill header
    escan.header.stamp = stamp;
    laserScan.header.frame_id = "/map";
    //add lasser scan
    escan.laser_scan = laserScan;
    //add informations about updated index
    double normalized_head_pos = lastHeadPosition + M_PI;
    uint16_t changed = (int) floor(normalized_head_pos / stepSize);
    escan.changed = changed;
    
    //publish created message
    ePublisher.publish(escan);
    

    //debug output
    ROS_DEBUG("head %lf", angles::to_degrees(lastHeadPosition));
    ROS_DEBUG("mormalized head %lf", angles::to_degrees(normalized_head_pos));
    ROS_DEBUG("step size %lf", angles::to_degrees(stepSize));
    ROS_DEBUG(" changed %d", changed);
    ROS_DEBUG("--");
}

void WallDetection::init()
{
    
    isInitialized = true;
    reconfigServer.setCallback(boost::bind(&WallDetection::reconfigure, this, _1, _2));
}

void WallDetection::reconfigure(hanse_sonarlocalization::WallDetectionConfig &newConfig, uint32_t level)
{
    stepSize = angles::from_degrees(newConfig.step_size);
    publishAngle = angles::from_degrees(newConfig.publish_angle);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wall_detection");
    ros::NodeHandle n;

    WallDetection wallDetection(n);

    ros::spin();

    return 0;
}
