#include <geometry_msgs/PoseStamped.h>
#include "util.h"
#include "wall_markers.h"


WallMarkers::WallMarkers(ros::NodeHandle handle) :
    nh(handle),
    markerPublisher(handle.advertise<visualization_msgs::Marker>("localization/viz/wall_markers", 1000)),
    positionSubscriber(handle.subscribe("position/estimate", 1, &WallMarkers::positionCallback, this)),
    wallSubscriber(handle.subscribe("sonar/scan/walls", 1, &WallMarkers::wallCallback, this))
{
    havePosition = false;
    frameID = 0;
}

void WallMarkers::positionCallback(const geometry_msgs::PoseStamped &position)
{
    this->position = position.pose;
}


void WallMarkers::wallCallback(const hanse_msgs::WallDetection &wall)
{
    if (wall.wallDetected) {
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = "/map";
        m.ns = "scan_lines";
        m.id = frameID;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose = position;
        m.scale.x = 0.1;
        m.color.a = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;
        geometry_msgs::Point start, end;
        start.x = 0;
        start.y = 0;
        start.z = 0;

        float maxDistance = 0;

        for (auto d : wall.distances)
            maxDistance = std::max(maxDistance, (float)d);

        end.x = maxDistance * cos(wall.headPosition);
        end.y = maxDistance * sin(wall.headPosition);
        end.z = 0;
        m.points.push_back(start);
        m.points.push_back(end);

        lineMarkers.push_back(m);

        m.ns = "wall_points";
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.points.clear();
        m.scale.x = 0.1;
        m.scale.y = 0.5;
        m.scale.z = 0.5;
        m.color.r = 1;
        for (float d : wall.distances) {
            geometry_msgs::Point p;
            p.x = d * cos(wall.headPosition+M_PI/50);
            p.y = d * sin(wall.headPosition+M_PI/50);
            m.points.push_back(p);
            p.x = d * cos(wall.headPosition-M_PI/50);
            p.y = d * sin(wall.headPosition-M_PI/50);
            m.points.push_back(p);
        }
        wallMarkers.push_back(m);

    }

    frameID++;

    if (!lineMarkers.empty() && lineMarkers.front().id < frameID - 60) {
        visualization_msgs::Marker &m = lineMarkers.front();
        m.action = visualization_msgs::Marker::DELETE;
        markerPublisher.publish(m);
        lineMarkers.pop_front();

        visualization_msgs::Marker &m2 = wallMarkers.front();
        m2.action = visualization_msgs::Marker::DELETE;
        markerPublisher.publish(m2);
        wallMarkers.pop_front();
    }
    for (auto &m : lineMarkers) {
        if (m.id < frameID - 10)
            m.color.b = 0.5;
        markerPublisher.publish(m);
    }
    for (auto &m : wallMarkers) {
        markerPublisher.publish(m);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wall_markers");
    ros::NodeHandle n;
    WallMarkers wallMarkers(n);

    ros::spin();

    return 0;
}
