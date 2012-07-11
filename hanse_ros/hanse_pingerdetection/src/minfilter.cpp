#include <algorithm>
#include "minfilter.h"

MinFilter::MinFilter(int window) :
    windowCounter(0)
{
    setWindow(window);
}

void MinFilter::setWindow(int window)
{
    this->window = window;
    windowData.clear();
    windowData.resize(window, 0.0f);
    windowCounter = 0;
}


float MinFilter::filter(float sample)
{
    windowData[windowCounter] = sample;
    windowCounter++;
    if (windowCounter == window)
        windowCounter = 0;
    return *std::min_element(windowData.begin(), windowData.end());
}
