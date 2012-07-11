#include <cmath>
#include "goertzel.h"

Goertzel::Goertzel(int window, float frequency, int averageWindow) :
    integralSin(0.0f), integralCos(0.0f),
    listCounter(0),
    sampleCounter(0)
{
    setParameters(window, frequency, averageWindow);
}

void Goertzel::setParameters(int window, float frequency, int averageWindow)
{
    if (this->window == window && this->frequency == frequency && this->averageWindow == averageWindow)
        return;
    this->window = window;
    this->frequency = frequency;
    this->averageWindow = averageWindow;
    windowSin.clear();
    windowSin.resize(window, 0.0f);
    windowCos.clear();
    windowCos.resize(window, 0.0f);
    integralSin = 0.0f;
    integralCos = 0.0f;
    listCounter = 0;
    sampleCounter = 0;
    averageVector.clear();
    averageVector.resize(averageWindow, 0.0f);
    averageCount = 0.0f;
    averageTemp = 0.0f;
}

float Goertzel::filter(float sample)
{
    integralSin -= windowSin[listCounter];
    integralCos -= windowCos[listCounter];


    float sinVal, cosVal;
    sincosf(2 * M_PI * (float)sampleCounter / sampleRate * frequency, &sinVal, &cosVal);

    sinVal *= sample;
    cosVal *= sample;

    sampleCounter++;

    integralSin += sinVal;
    integralCos += cosVal;

    windowSin[listCounter] = sinVal;
    windowCos[listCounter] = cosVal;

    listCounter++;
    if (listCounter == window){
        listCounter = 0;
    }

    integralSin *= 0.9999;
    integralCos *= 0.9999;

    float goertzel = integralSin * integralSin + integralCos * integralCos;

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
