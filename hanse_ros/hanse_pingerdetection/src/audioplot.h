#ifndef AUDIOPLOT_H
#define AUDIOPLOT_H

#include <ros/ros.h>
#include <deque>
#include "sensor_msgs/Image.h"

class AudioPlot
{
public:
    AudioPlot(ros::NodeHandle nh, int width, int height, const char *publisherName);
    ~AudioPlot();

    void addSample(float left, float right);

    void setSamplesPerPixel(int samples) { samplesPerPixel = samples; }

    void setCounter(int counterNew) { counter = counterNew; }


private:
    ros::Publisher imgPub;

    sensor_msgs::Image plotData();

    int samplesPerPixel;

    int width, height;

    char *data;
    int counter;
    int counterPixel;
    int shift;
};

#endif // AUDIOPLOT_H
