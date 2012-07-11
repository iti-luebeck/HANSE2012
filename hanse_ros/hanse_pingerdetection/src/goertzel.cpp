#include <cmath>
#include "goertzel.h"

Goertzel::Goertzel(int window, float frequency) :
    integralSin(0.0f), integralCos(0.0f),
    listCounter(0),
    sampleCounter(0)
{
    setParameters(window, frequency);
}

void Goertzel::setParameters(int window, float frequency)
{
    if (this->window == window && this->frequency == frequency)
        return;
    this->window = window;
    this->frequency = frequency;
    windowSin.clear();
    windowSin.resize(window, 0.0f);
    windowCos.clear();
    windowCos.resize(window, 0.0f);
    integralSin = 0.0f;
    integralCos = 0.0f;
    listCounter = 0;
    sampleCounter = 0;
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
    if (listCounter == window)
        listCounter = 0;

    integralSin *= 0.9999;
    integralCos *= 0.9999;

    return (integralSin * integralSin + integralCos * integralCos);
}
