#include <string.h>
#include <stdio.h>
#include "common.hpp"
#include "DriverFramework.hpp"

namespace DriverFramework
{

int GPIO::enable()
{
    int ret = 0;
    int fd = ::open("/sys/class/gpio/export", O_WRONLY);
    sprintf(buf, "%d", _gpio);
    if (fd > 0) {
        ssize_t nbytes = ::write(fd, buf, strlen(buf));

        if (nbytes != (ssize_t)strlen(buf)) {
            ret = -1;
            DF_LOG_ERR("error: could not enable gpio (_gpio=%d, writed %d)", _gpio, (int)nbytes);
        }

        ::close(fd);
    } else {
        ret = -1;
        DF_LOG_ERR("error: could open gpio for enable (_gpio=%d, fd = %d)", _gpio, fd);
    }

    return ret;
}

int GPIO::disable()
{
    int ret = 0;
    int fd = ::open("/sys/class/gpio/unexport", O_WRONLY);
    sprintf(buf, "%d", _gpio);
    if (fd > 0) {
        ssize_t nbytes = ::write(fd, buf, strlen(buf));

        if (nbytes != (ssize_t)strlen(buf)) {
            ret = -1;
            DF_LOG_ERR("error: could not disable gpio (_gpio=%d, writed %d)",  _gpio, (int)nbytes);
        }
        ::close(fd);
    } else {
        ret = -1;
        DF_LOG_ERR("error: could open gpio for disable (_gpio=%d, fd = %d)",  _gpio, fd);
    }

    return ret;
}

int GPIO::setDirection(DIR dir)
{
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", _gpio);
    int fd = ::open(buf, O_WRONLY);
    int ret = 0;
    if (fd > 0) {
        ssize_t nbytes;
        if (dir == INPUT) {
            nbytes = ::write(fd, "in", 2);
            if (nbytes != 2) {
                DF_LOG_ERR("error: could not set direction of gpio to IN (_gpio=%d, writed %d)",  _gpio, (int)nbytes);
                ret = -1;
            }
        } else if (dir == OUTPUT) {
            nbytes = ::write(fd, "out", 3);
            if (nbytes != 3) {
                DF_LOG_ERR("error: could not set direction of gpio to OUT (_gpio=%d, writed %d)",  _gpio, (int)nbytes);
                ret = -1;
            }
        }
        ::close(fd);
    } else {
        ret = -1;
        DF_LOG_ERR("error: could open gpio for setDirection (_gpio=%d, fd = %d)",  _gpio, fd);
    }

    return ret;
}

int GPIO::setValue(VALUE val)
{
    sprintf(buf, "/sys/class/gpio/gpio%d/value", _gpio);

    int fd = ::open(buf, O_WRONLY);
    int ret = 0;
    if (fd > 0) {
        ssize_t nbytes = -1;
        if (val == HIGH) {
            nbytes = ::write(fd, "1", 1);
        } else if (val == LOW) {
            nbytes = ::write(fd, "0", 1);
        }
        if (nbytes == 0) {
            ret = -1;
            DF_LOG_ERR("error: could not set gpio value (_gpio=%d, writed %d)",  _gpio, (int)nbytes);
        }
        ::close(fd);
    } else {
        ret = -1;
        DF_LOG_ERR("error: could open gpio for setValue (_gpio=%d, fd = %d)",  _gpio, fd);
    }

    return ret;
}

int GPIO::readValue(VALUE& val)
{
    sprintf(buf, "/sys/class/gpio/gpio%d/value", _gpio);
    char value[1];
    int fd = ::open(buf, O_RDONLY);
    int ret = 0;
    if (fd > 0) {
        ssize_t nbytes = ::read(fd, value, 1);

        if (nbytes == 0) {
            ret = -1;
            DF_LOG_ERR("error: could not read gpio value (_gpio=%d, writed %d)",  _gpio, (int)nbytes);
        }
        ::close(fd);
        val = (value[0]=='0' ? LOW : HIGH);
    } else {
        ret = -1;
        DF_LOG_ERR("error: could open gpio for readValue (_gpio=%d, fd = %d)", _gpio, fd);
    }

    return ret;
}

} //end namespace
