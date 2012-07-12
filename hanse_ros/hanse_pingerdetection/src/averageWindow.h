#ifndef AVERAGEWINDOW_H
#define AVERAGEWINDOW_H

#include <vector>

class AverageWindow {
public:
    AverageWindow(int window);

    void setWindow(int window);

    float averageCalulation(float goertzel);

private:
    std::vector<float> averageVector;
    int averageCount;
    int averageWindow;
    float averageTemp;
};

#endif
