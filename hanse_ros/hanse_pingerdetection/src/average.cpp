#include <algorithm>
#include "average.h"

Average::Average(int window)
{
    setWindow(window);
}

void Average::setWindow(int averageWindow)
{
    averageVector.clear();
    averageVector.resize(averageWindow, 0.0f);
    averageCount = 0.0f;
    averageTemp = 0.0f;
}


float Average::filter(float goertzel)
{
    averageTemp -= averageVector[averageCount];
    averageVector[averageCount] = goertzel;
    averageTemp += averageVector[averageCount];

    float returnAverage = averageTemp/averageWindow;

    averageCount++;
    if(averageCount == averageWindow){
        averageCount = 0;
    }

    return returnAverage;
}
