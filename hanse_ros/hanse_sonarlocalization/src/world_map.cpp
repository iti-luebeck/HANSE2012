#include <iostream>
#include "opencv2/opencv.hpp"
//#include "cv.h"
//#include "highgui.h"

#include "world_map.h"

WorldMap::WorldMap(const std::string &mapFile, float resolution) :
    resolution(resolution)
{
    cv::Mat image = cv::imread(mapFile, 0);
    cv::Mat water(image.size(), CV_8U), blocked(image.size(), CV_8U);
    cv::Mat waterDistance(image.size(), CV_32F), blockedDistance(image.size(), CV_32F);
    cv::Mat signedDistance(image.size(), CV_32F);
    cv::threshold(image, water, 127, 255, cv::THRESH_BINARY_INV);
    cv::threshold(image, blocked, 127, 255, cv::THRESH_BINARY);

    cv::distanceTransform(water, waterDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::distanceTransform(blocked, blockedDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    cv::addWeighted(waterDistance, 1 / resolution, blockedDistance, -1 / resolution, 0, signedDistance);

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
    Eigen::Vector2f pixelPoint = point * resolution;

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

Eigen::Vector2f WorldMap::mapSize()
{
  return Eigen::Vector2f(distanceMap.cols() / resolution, distanceMap.rows() / resolution);
}
