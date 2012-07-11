#ifndef GOERTZEL_H
#define GOERTZEL_H

#include "globals.h"

#include <vector>

class Goertzel {
public:
    Goertzel(int window, float frequency, int averageWindow);

    void setParameters(int window, float frequency, int averageWindow);

    float filter(float sample);

private:
    int window;
    float frequency;
    std::vector<float> windowSin, windowCos;
    float integralSin, integralCos;
    int listCounter;
    int sampleCounter;

    std::vector<float> averageVector;
    int averageCount;
    int averageWindow;
    float averageTemp;
};

#endif
