#ifndef AUDIOPLOT_H
#define AUDIOPLOT_H

#include <ros/ros.h>
#include <deque>
#include "sensor_msgs/Image.h"

class AudioPlot
{
public:
    AudioPlot(ros::NodeHandle nh, int width, int height);
    ~AudioPlot();

    void addSampleRaw(float left, float right);
    void addSampleGoertzel(float left, float right);

    void setSamplesPerPixelRaw(int samples) { samplesPerPixelRaw = samples; }
    void setSamplesPerPixelGoertzel(int samples) { samplesPerPixelGoertzel = samples; }

    void setCounterRaw(int counter) { counterRaw = counter; }
    void setCounterGoertzel(int counter) { counterGoertzel = counter; }

private:
    ros::Publisher imgPubRaw;
    ros::Publisher imgPubGoertzel;

    sensor_msgs::Image plotDataRaw();
    sensor_msgs::Image plotDataGoertzel();

    int samplesPerPixelRaw;
    int samplesPerPixelGoertzel;

    int width, height;

    char *dataRaw;
    int counterRaw;
    int counterPixelRaw;
    int shiftRaw;

    char *dataGoertzel;
    int counterGoertzel;
    int counterPixelGoertzel;
    int shiftGoertzel;
};

#endif // AUDIOPLOT_H
