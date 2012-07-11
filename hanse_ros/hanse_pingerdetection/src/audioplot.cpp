#include <algorithm>
#include "audioplot.h"

AudioPlot::AudioPlot(ros::NodeHandle nh, int width, int height, const char* publisherName)
{
    samplesPerPixel = 100;
    imgPub = nh.advertise<sensor_msgs::Image>(publisherName, 1);
    counter = 0;
    counterPixel = 0;
    shift = 0;

    this->width = width;
    this->height = height;

    data = new char[width * height * 3];
    for (int i = 0; i < width * height * 3; i++) {
        data[i] = 0xFF;
    }
}

AudioPlot::~AudioPlot()
{
    delete data;
}

void AudioPlot::addSample(float left, float right)
{
    int px = width-1;
    int ply = height - 1 - std::min(1.f, std::max(0.f, left)) * height;
    int pry = height - 1 - std::min(1.f, std::max(0.f, right)) * height;

    data[3 * ((px + shift) % width + width * ply) + 0] = 255;
    data[3 * ((px + shift) % width + width * pry) + 1] = 255;

    if (counterPixel == 0) {
        shift++;
        if (shift == width)
            shift = 0;

        counterPixel = samplesPerPixel;
        for (int y = 0; y < height; y++) {
            data[3 * ((width - 1 + shift) % width + width * y) + 0] = 0;
            data[3 * ((width - 1 + shift) % width + width * y) + 1] = 0;
            data[3 * ((width - 1 + shift) % width + width * y) + 2] = 0;
        }

    }
    counterPixel--;

    if (counter == 0) {
        counter = counterMax;
        imgPub.publish(plotData());
    }
    counter--;
}

sensor_msgs::Image AudioPlot::plotData()
{
    sensor_msgs::Image plot;
    plot.header.stamp = ros::Time::now();
    plot.width = width;
    plot.height = height;
    plot.step = width * 3;
    plot.encoding = "rgb8";
    plot.is_bigendian = false;
    plot.data.resize(width * height * 3);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            plot.data[3 * (x + width * y) + 0] = data[3 * ((x + shift) % width + width * y) + 0];
            plot.data[3 * (x + width * y) + 1] = data[3 * ((x + shift) % width + width * y) + 1];
            plot.data[3 * (x + width * y) + 2] = data[3 * ((x + shift) % width + width * y) + 2];

        }
    }
    return plot;
}
