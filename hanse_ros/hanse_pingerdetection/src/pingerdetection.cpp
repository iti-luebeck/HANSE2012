#include <ros/ros.h>
#include <pulse/simple.h>
#include "pingerdetection.h"
#include <hanse_msgs/PingerDetection.h>
#include <hanse_msgs/PingerDetectionDebug.h>

PingerDetection::PingerDetection() :
    currentState(WAIT_FOR_PING),
    audioInput(0),
    inputBuffer(0),
    leftGoertzel(240, 15000),
    rightGoertzel(240, 15000),
    rawPlot(nh, 320, 240, "/pingerdetection/plotRaw"),
    goertzelPlot(nh, 320, 240, "/pingerdetection/plotGoertzel"),
    minPlot(nh, 320, 240, "/pingerdetection/plotMin"),
    goertzelAveragePlot(nh, 320, 240,  "/pingerdetection/plotGoertzelAverage"),
    leftMinFilter(240),
    rightMinFilter(240),
    leftAverageGoertzel(500),
    rightAverageGoertzel(500),
    leftAverageMagnitude(10),
    rightAverageMagnitude(10),
    sampleCounter(0)
{

    enabled = false;

    reconfigureServer.setCallback(boost::bind(&PingerDetection::reconfigure, this, _1, _2));
    pingerInput = nh.subscribe("/hanse/pinger/status",1, &PingerDetection::pingerDetectionCallback, this);


    pingerPub = nh.advertise<hanse_msgs::PingerDetection>("/hanse/pinger", 10);
    pingerPubDebug = nh.advertise<hanse_msgs::PingerDetection>("/hanse/pingerDebug", 10);

    pa_sample_spec ss;

    ss.format = PA_SAMPLE_FLOAT32LE;
    ss.channels = 2;
    ss.rate = sampleRate;

    audioInput = pa_simple_new(NULL, "hanse_pingerdetection",
                               PA_STREAM_RECORD, NULL,
                               "pinger",
                               &ss,
                               NULL,
                               NULL,
                               NULL);
    if (!audioInput) {
        ROS_ERROR("Could not initialize audio");
        exit(1);
    }

    inputBuffer = new float[bufferFrames * 2];
}

PingerDetection::~PingerDetection()
{
    if (audioInput)
        pa_simple_free(audioInput);
    delete inputBuffer;
}

void PingerDetection::tick()
{
    int error;
    pa_simple_read(audioInput, (void *)inputBuffer, sizeof(float) * bufferFrames * 2, &error);
    if(enabled){
        for (int i = 0; i < bufferFrames; i++) {
            processSample(inputBuffer[2 * i + 0], inputBuffer[2 * i + 1]);
        }
    }
}

void PingerDetection::processSample(float left, float right)
{

    left *= config.stereoAdjustment;

    if (config.plotRaw){
        rawPlot.addSample(left / 2 + 0.5f, right / 2 + 0.5f);
    }

    // Goertzel berechnen
    float leftGoertzelSample = leftGoertzel.goertzelFilter(left);
    float rightGoertzelSample = rightGoertzel.goertzelFilter(right);

    if (config.plotGoertzel){
        goertzelPlot.addSample(leftGoertzelSample / config.plotScaleGoertzel, rightGoertzelSample / config.plotScaleGoertzel);
    }

    // Durchschnitt über mehrere Goertzel-Werte berechnen
    float leftAverageGoertzelSample = leftAverageGoertzel.averageCalulation(leftGoertzelSample);
    float rightAverageGoertzelSample = rightAverageGoertzel.averageCalulation(rightGoertzelSample);

    if(config.plotGoertzelAverage){
        goertzelAveragePlot.addSample(leftAverageGoertzelSample / config.plotScaleAverageGoertzel, rightAverageGoertzelSample / config.plotScaleAverageGoertzel);
    }

    // Minimum-filter
    float leftMinFilterSample = leftMinFilter.filter(leftGoertzelSample);
    float rightMinFilterSample = rightMinFilter.filter(rightGoertzelSample);

    if (config.plotMin){
        minPlot.addSample(leftMinFilterSample / config.plotScaleMin, rightMinFilterSample / config.plotScaleMin);
    }

    sampleCounter++;

    if (timeout != 0) {
        timeout--;
        if (timeout == 0) {
            currentState = WAIT_FOR_NO_PING;
        }
    }

    float leftSample = 0.0f;
    float rightSample = 0.0f;

    // Je nach Enumeintrag wird eine andere Quelle für die Berechnung eines Peaks verwendet
    if(config.analyseSource == 3){
        // AverageGoertzel
        leftSample = leftAverageGoertzelSample;
        rightSample = rightAverageGoertzelSample;
    } else if(config.analyseSource == 2){
        // Min
        leftSample = leftMinFilterSample;
        rightSample = rightMinFilterSample;
    } else {
        // Goertzel
        leftSample = leftGoertzelSample;
        rightSample = rightGoertzelSample;
    }


    // ROS_INFO("leftSample %f  rightSample %f", leftSample, rightSample);

    switch (currentState) {
    case WAIT_FOR_PING: {

        timeout = (int)(config.timeout * sampleRate);
        leftMax = 0.0f;
        rightMax = 0.0f;
        leftPeakSum = 0.0f;
        leftWeighted = 0.0f;
        rightPeakSum = 0.0f;
        rightWeighted = 0.0f;

        bool leftDetected = leftSample > config.thresholdHigh;
        bool rightDetected = rightSample > config.thresholdHigh;

        if (leftDetected) {
            leftArrival = sampleCounter;
            currentState = WAIT_FOR_SECOND_PING_RIGHT;
            addLeft(leftSample);
        }
        if (rightDetected) {
            rightArrival = sampleCounter;
            currentState = WAIT_FOR_SECOND_PING_LEFT;
            addRight(rightSample);
        }
        if (leftDetected && rightDetected) {
            currentState = WAIT_FOR_NO_PING;
        }
    } break;
    case WAIT_FOR_SECOND_PING_LEFT: {
        addRight(rightSample);
        bool leftDetected = leftSample > config.thresholdHigh;
        if (leftDetected) {
            leftArrival = sampleCounter;
            currentState = WAIT_FOR_NO_PING;
            addLeft(leftSample);
        }
    } break;
    case WAIT_FOR_SECOND_PING_RIGHT: {
        addLeft(leftSample);
        if (leftMax == leftSample)
            leftArrival = sampleCounter;
        bool rightDetected = rightSample > config.thresholdHigh;
        if (rightDetected) {
            rightArrival = sampleCounter;
            currentState = WAIT_FOR_NO_PING;
            addRight(rightSample);
        }
    } break;
    case WAIT_FOR_NO_PING: {

        addRight(rightSample);
        addLeft(leftSample);
        bool leftSilent = leftSample < config.thresholdLow;
        bool rightSilent = rightSample < config.thresholdLow;
        if (leftSilent && rightSilent) {
            //int sampleDifference = (rightArrival + rightWeighted / rightSum) - (leftArrival + leftWeighted / leftSum);
            int sampleDifference = rightArrival - leftArrival;

            float leftAverageMagnitudeResult = leftAverageMagnitude.averageCalulation(leftMax);
            float rightAverageMagnitudeResult = rightAverageMagnitude.averageCalulation(rightMax);

            hanse_msgs::PingerDetection msg;
            msg.header.stamp = ros::Time::now();
            /*msg.leftAmplitude = leftPeakSum;
            msg.rightAmplitude = rightPeakSum;
            msg.timeDifference = sampleDifference / (float)sampleRate;
            msg.leftPeakSum = leftPeakSum;
            msg.rightPeakSum = rightPeakSum;
            msg.leftWeighted = leftWeighted;
            msg.rightWeighted = rightWeighted;
            msg.leftAverageMagnitude = leftAverageMagnitudeResult;
            msg.rightAverageMagnitude = rightAverageMagnitudeResult;*/

            msg.leftAmplitude = leftAverageMagnitudeResult;
            msg.rightAmplitude = rightAverageMagnitudeResult;
            msg.timeDifference = sampleDifference / (float)sampleRate;
            msg.leftPeakSum = leftPeakSum;
            msg.rightPeakSum = rightPeakSum;
            msg.leftWeighted = leftWeighted;
            msg.rightWeighted = rightWeighted;
            currentState = WAIT_FOR_PING;
            pingerPub.publish(msg);
        }
    }
    };


}

void PingerDetection::addRight(float sample)
{
    rightMax = std::max(rightMax, sample);
    rightPeakSum += sample;
    rightWeighted += (sampleCounter - rightArrival) * sample;
}

void PingerDetection::addLeft(float sample)
{
    leftMax = std::max(leftMax, sample);
    leftPeakSum += sample;
    leftWeighted += (sampleCounter - leftArrival) * sample;
}

float PingerDetection::calculateAngle(int sampleDifference)
{
    if(sampleDifference == 0){
        return 0.0;
    } else {
        float delta_t = sampleRate/(float)sampleDifference; // Berechnung des Zeitunterschieds in Sekunden

        float distdiff = config.speedOfSound / delta_t; // Berechnung der zurückgelegten Wegstrecke aus dem Zeitunterschied.

        float angle = distdiff / config.baseline;
        return angle;
    }
}

void PingerDetection::reconfigure(hanse_pingerdetection::PingerDetectionConfig &config, uint32_t level)
{
    this->config = config;

    leftGoertzel.setParameters(config.window, config.frequency) ;
    rightGoertzel.setParameters(config.window, config.frequency);
    leftAverageGoertzel.setWindow(config.averageWindowGoertzelSamples);
    rightAverageGoertzel.setWindow(config.averageWindowGoertzelSamples);
    leftMinFilter.setWindow(config.minWindow);
    rightMinFilter.setWindow(config.minWindow);
    rawPlot.setSamplesPerPixel(config.samplesPerPixel);
    goertzelPlot.setSamplesPerPixel(config.samplesPerPixel);
    minPlot.setSamplesPerPixel(config.samplesPerPixel);
    goertzelAveragePlot.setSamplesPerPixel(config.samplesPerPixel);
    rawPlot.setCounter(config.counter);
    goertzelPlot.setCounter(config.counter);
    minPlot.setCounter(config.counter);
    goertzelAveragePlot.setCounter(config.counter);

    leftAverageMagnitude.setWindow(config.averageMagnitudeWindow);
    rightAverageMagnitude.setWindow(config.averageMagnitudeWindow);

}


void PingerDetection::pingerDetectionCallback(const std_msgs::StringConstPtr& msg){

    if(msg->data == std::string("start")){
        ROS_INFO("Pingerdetection callback enable");
        this->enabled = true;
    } else if(msg->data == std::string("stop")){
        ROS_INFO("Pingerdetection callback disable");
        this->enabled = false;
    } else {
        ROS_INFO("Pingerdetection callback parameter %s is not valid", msg->data.c_str());
    }

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pingerdetection");
    PingerDetection pd;

    while (true) {
        ros::spinOnce();
        pd.tick();
    }
}
