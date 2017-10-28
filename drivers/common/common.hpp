#pragma once

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

namespace DriverFramework
{

class GPIO
{
public:
    enum DIR {
        INPUT,
        OUTPUT
    };

    enum VALUE {
        HIGH,
        LOW
    };

    GPIO(int gpio) : _gpio(gpio){}
    ~GPIO(){}

    int enable();

    int disable();

    int setDirection(DIR dir);

    int setValue(VALUE val);

    int readValue(VALUE& val);

private:
    int  _gpio;
    char buf[128];
};

} //end namespace
