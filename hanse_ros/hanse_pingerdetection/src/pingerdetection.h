#ifndef PINGERDETECTION_H
#define PINGERDETECTION_H

#include "globals.h"

#include <pulse/simple.h>
#include "goertzel.h"
#include "minfilter.h"
#include "audioplot.h"
#include <ros/ros.h>
#include "average.h"

#include "hanse_pingerdetection/PingerDetectionConfig.h"
#include <dynamic_reconfigure/server.h>

class PingerDetection {
public:
    static const int fps = 20;
    static const int bufferFrames = sampleRate / fps;
    PingerDetection();

    ~PingerDetection();

    void tick();



private:
    void processSample(float left, float right);

    void addRight(float sample);
    void addLeft(float sample);

    float calculateAngle(int sampleDifference);

    void reconfigure(hanse_pingerdetection::PingerDetectionConfig &config, uint32_t level);

    enum state {
        WAIT_FOR_PING,
        WAIT_FOR_SECOND_PING_LEFT,
        WAIT_FOR_SECOND_PING_RIGHT,
        WAIT_FOR_NO_PING
    };

    state currentState;
    ros::NodeHandle nh;
    pa_simple *audioInput;
    float *inputBuffer;
    Goertzel leftGoertzel, rightGoertzel;
    AudioPlot rawPlot, goertzelPlot, minPlot, goertzelAveragePlot;
    MinFilter leftMin, rightMin;
    Average goertzelAverage;
    hanse_pingerdetection::PingerDetectionConfig config;
    dynamic_reconfigure::Server<hanse_pingerdetection::PingerDetectionConfig> reconfigureServer;
    ros::Publisher pingerPub;
    int sampleCounter;
    int leftArrival, rightArrival;
    float leftSum, leftWeighted, rightSum, rightWeighted;
    float leftMax, rightMax;
    int timeout;
};

#endif
