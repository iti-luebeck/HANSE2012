#include <algorithm>
#include "audioplot.h"

AudioPlot::AudioPlot(ros::NodeHandle nh, unsigned width, unsigned height) :
    samplesPerPixelRaw(100),
    width(width),
    height(height),
    imgPubRaw(nh.advertise<sensor_msgs::Image>("pingerdetection/plotRaw", 1)),
    imgPubGoertzel(nh.advertise<sensor_msgs::Image>("pingerdetection/plotGoertzel", 1)),
    counterRaw(0),
    counterPixelRaw(0),
    shiftRaw(0)
{
    dataRaw = new unsigned char[width * height * 3];
    for (unsigned i = 0; i < width * height * 3; i++) {
        dataRaw[i] = 0xFF;
    }

    dataGoertzel = new unsigned char[width * height * 3];
    for (unsigned i = 0; i < width * height * 3; i++) {
        dataGoertzel[i] = 0xFF;
    }


}

AudioPlot::~AudioPlot()
{
    delete dataRaw;
    delete dataGoertzel;
}

void AudioPlot::addSampleRaw(float left, float right)
{
    unsigned px = width-1;
    unsigned ply = height - 1 - std::min(1.f, std::max(0.f, left)) * height;
    unsigned pry = height - 1 - std::min(1.f, std::max(0.f, right)) * height;

    dataRaw[3 * ((px + shiftRaw) % width + width * ply) + 0] = 255;
    dataRaw[3 * ((px + shiftRaw) % width + width * pry) + 1] = 255;

    if (counterPixelRaw == 0) {
        shiftRaw++;
        if (shiftRaw == width)
            shiftRaw = 0;

        counterPixelRaw = samplesPerPixelRaw;
        for (unsigned y = 0; y < height; y++) {
            dataRaw[3 * ((width - 1 + shiftRaw) % width + width * y) + 0] = 0;
            dataRaw[3 * ((width - 1 + shiftRaw) % width + width * y) + 1] = 0;
            dataRaw[3 * ((width - 1 + shiftRaw) % width + width * y) + 2] = 0;
        }

    }
    counterPixelRaw--;

    if (counterRaw == 0) {
        counterRaw = 48000 / 10;
        imgPubRaw.publish(plotDataRaw());
    }
    counterRaw--;
}

sensor_msgs::Image AudioPlot::plotDataRaw()
{
    sensor_msgs::Image plot;
    plot.header.stamp = ros::Time::now();
    plot.width = width;
    plot.height = height;
    plot.step = width * 3;
    plot.encoding = "rgb8";
    plot.is_bigendian = false;
    plot.data.resize(width * height * 3);

    //ROS_INFO("Plot raw");

    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            plot.data[3 * (x + width * y) + 0] = dataRaw[3 * ((x + shiftRaw) % width + width * y) + 0];
            plot.data[3 * (x + width * y) + 1] = dataRaw[3 * ((x + shiftRaw) % width + width * y) + 1];
            plot.data[3 * (x + width * y) + 2] = dataRaw[3 * ((x + shiftRaw) % width + width * y) + 2];

        }
    }
    return plot;
}

void AudioPlot::addSampleGoertzel(float left, float right)
{
    unsigned px = width-1;
    unsigned ply = height - 1 - std::min(1.f, std::max(0.f, left)) * height;
    unsigned pry = height - 1 - std::min(1.f, std::max(0.f, right)) * height;

    dataGoertzel[3 * ((px + shiftGoertzel) % width + width * ply) + 0] = 255;
    dataGoertzel[3 * ((px + shiftGoertzel) % width + width * pry) + 1] = 255;

    if (counterPixelGoertzel == 0) {
        shiftGoertzel++;
        if (shiftGoertzel == width)
            shiftGoertzel = 0;

        counterPixelGoertzel = samplesPerPixelRaw;
        for (unsigned y = 0; y < height; y++) {
            dataGoertzel[3 * ((width - 1 + shiftGoertzel) % width + width * y) + 0] = 0;
            dataGoertzel[3 * ((width - 1 + shiftGoertzel) % width + width * y) + 1] = 0;
            dataGoertzel[3 * ((width - 1 + shiftGoertzel) % width + width * y) + 2] = 0;
        }

    }
    counterPixelGoertzel--;

    if (counterGoertzel == 0) {
        counterGoertzel = 48000 / 10;
        imgPubGoertzel.publish(plotDataGoertzel());
    }
    counterGoertzel--;
}

sensor_msgs::Image AudioPlot::plotDataGoertzel()
{
    sensor_msgs::Image plot;
    plot.header.stamp = ros::Time::now();
    plot.width = width;
    plot.height = height;
    plot.step = width * 3;
    plot.encoding = "rgb8";
    plot.is_bigendian = false;
    plot.data.resize(width * height * 3);

    //ROS_INFO("Plot goertzel");

    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            plot.data[3 * (x + width * y) + 0] = dataGoertzel[3 * ((x + shiftGoertzel) % width + width * y) + 0];
            plot.data[3 * (x + width * y) + 1] = dataGoertzel[3 * ((x + shiftGoertzel) % width + width * y) + 1];
            plot.data[3 * (x + width * y) + 2] = dataGoertzel[3 * ((x + shiftRaw) % width + width * y) + 2];
        }
    }
    return plot;
}
