#ifndef AVERAGE_H
#define AVERAGE_H

#include <vector>

class Average {
public:
    Average(int window);

    void setWindow(int window);

    float filter(float goertzel);

private:
    std::vector<float> averageVector;
    int averageCount;
    int averageWindow;
    float averageTemp;
};

#endif
