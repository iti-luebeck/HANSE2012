#include <algorithm>
#include "averageWindow.h"
#include <ros/ros.h>

AverageWindow::AverageWindow(int window)
{
    setWindow(window);
}

void AverageWindow::setWindow(int averageWindow){
    if (averageWindow != this->averageWindow) {
        averageVector.clear();
        averageVector.resize(averageWindow, 0.0f);
        averageCount = 0;
        averageTemp = 0.0f;
        this->averageWindow = averageWindow;
    }
}


float AverageWindow::averageCalulation(float goertzel){

    averageTemp -= averageVector[averageCount % averageWindow];
    averageVector[averageCount % averageWindow] = goertzel;
    averageTemp += averageVector[averageCount % averageWindow];

    averageCount++;
    float returnAverage = averageTemp / std::min(averageWindow, averageCount);

    return returnAverage;
}
