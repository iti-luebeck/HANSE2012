#include <ros/console.h>
#include <ros/init.h>
#include <iostream>
#include "opencv2/opencv.hpp"
//#include "cv.h"
//#include "highgui.h"

#include "world_map.h"

WorldMap::WorldMap(const std::string &mapFile, float pixelSize, float threshold) :
    pixelSize(pixelSize)
{
    cv::Mat image = cv::imread(mapFile, 0);
    if (image.empty()) {
	ROS_ERROR("Could not load map image %s", mapFile.c_str());
	ros::shutdown();
	return;
    }
    cv::Mat water(image.size(), CV_8U), blocked(image.size(), CV_8U);
    cv::Mat waterDistance(image.size(), CV_32F), blockedDistance(image.size(), CV_32F);
    cv::Mat signedDistance(image.size(), CV_32F);
    cv::threshold(image, water, threshold, 255, cv::THRESH_BINARY_INV);
    cv::threshold(image, blocked, threshold, 255, cv::THRESH_BINARY);

    cv::distanceTransform(water, waterDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::distanceTransform(blocked, blockedDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    cv::addWeighted(waterDistance, pixelSize, blockedDistance, -pixelSize, 0, signedDistance);

    //cv::convertScaleAbs(waterDistance, image, 1.0);
    //cv::convertScaleAbs(blocked, blocked, 255.0);


    //cv::namedWindow("test");
    //cv::imshow("test", image);
    //cv::waitKey(0);
    distanceMap = Eigen::ArrayXXf(image.rows, image.cols);

    for (int y = 0; y < image.rows; y++) {
	for (int x = 0; x < image.cols; x++) {
	    distanceMap(y, x) = signedDistance.at<float>(y, x);
	}
    }
}

float WorldMap::wallDistance(Eigen::Vector2f point)
{
    Eigen::Vector2f pixelPoint = point / pixelSize;

    int x = roundf(pixelPoint(0));
    int y = roundf(pixelPoint(1));

    if (x < 0)
	x = 0;
    if (x >= distanceMap.cols())
	x = distanceMap.cols() - 1;
    if (y < 0)
	y = 0;
    if (y >= distanceMap.rows())
	y = distanceMap.rows() - 1;
    return distanceMap(y, x);
}

float WorldMap::directedWallDistance(Eigen::Vector2f point, Eigen::Vector2f direction, float maximum)
{
    const float eps = 1.5 * pixelSize;
    float min = 0;
    const float max = maximum;

    while (true) {
	float d = wallDistance(point + direction * min);
	if (d < 0)
	    return min;
	min += wallDistance(point + direction * min) + eps;
	if (min >= max)
	    return max;
    }
}

Eigen::Vector2f WorldMap::mapSize()
{
  return Eigen::Vector2f(distanceMap.cols() * pixelSize, distanceMap.rows() * pixelSize);
}