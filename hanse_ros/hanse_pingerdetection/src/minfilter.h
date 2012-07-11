#ifndef MINFILTER_H
#define MINFILTER_H

#include <vector>

class MinFilter {
public:
    MinFilter(int window);

    void setWindow(int window);

    float filter(float sample);

private:
    int window;
    std::vector<float> windowData;
    int windowCounter;
};

#endif
