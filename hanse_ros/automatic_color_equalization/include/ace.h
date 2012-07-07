#ifndef ACE_H
#define ACE_H

#include <opencv2/opencv.hpp>

#define ACE_SAMPLE_GRID         0x01
#define ACE_SAMPLE_GAUSS        0x02
#define ACE_SAMPLE_UNIFORM      0x04

#define ACE_COLORSPACE_RGB      0x01
#define ACE_COLORSPACE_HSV      0x02
#define ACE_COLORSPACE_YCrCb    0x04
#define ACE_COLORSPACE_GRAY     0x08

#define ACE_CHANNEL_1           0x01
#define ACE_CHANNEL_2           0x02
#define ACE_CHANNEL_3           0x04

#define ACE_SATURATE            0x01
#define ACE_REEVALUATE          0x02
#define ACE_NO_SATURATION       0x04

class ACE
{
public:
    ACE();
    ~ACE();
    void init(int width, int height, int nSamples = 100, int sampleMethod = ACE_SAMPLE_GAUSS, double stdev = 0.3, int step = 16,
              int colorSpace = ACE_COLORSPACE_RGB, int channels = ACE_CHANNEL_1 | ACE_CHANNEL_2 | ACE_CHANNEL_3, int saturation = ACE_REEVALUATE);
    void automaticColorEqualization(IplImage *in, IplImage *out);
private:
    double response(IplImage *I, CvMat* values, CvMat* distances, int x, int y, int channel);
    double resp(double x);
private:
    int nSamples;
    int sampleMethod;
    double stdev;
    int step;
    int colorSpace;
    int channels;
    int saturation;

    CvMat *values;
    CvMat *distances;
};

#endif // ACE_H
