#ifndef AUDIOPLOT_H
#define AUDIOPLOT_H

#include <ros/ros.h>
#include <deque>
#include "sensor_msgs/Image.h"

class AudioPlot
{
public:
    AudioPlot(ros::NodeHandle nh, unsigned width, unsigned height);
    ~AudioPlot();

    void addSampleRaw(float left, float right);
    void addSampleGoertzel(float left, float right);

    void setSamplesPerPixelRaw(unsigned samples) { samplesPerPixelRaw = samples; }
    void setSamplesPerPixelGoertzel(unsigned samples) { samplesPerPixelGoertzel = samples; }

    void setCounterRaw(unsigned counter) { counterRaw = counter; }
    void setCounterGoertzel(unsigned counter) { counterGoertzel = counter; }

private:
    ros::Publisher imgPubRaw;
    ros::Publisher imgPubGoertzel;

    sensor_msgs::Image plotDataRaw();
    sensor_msgs::Image plotDataGoertzel();

    unsigned samplesPerPixelRaw;
    unsigned samplesPerPixelGoertzel;

    unsigned width, height;

    unsigned char *dataRaw;
    unsigned counterRaw;
    unsigned counterPixelRaw;
    unsigned shiftRaw;

    unsigned char *dataGoertzel;
    unsigned counterGoertzel;
    unsigned counterPixelGoertzel;
    unsigned shiftGoertzel;
};

#endif // AUDIOPLOT_H
