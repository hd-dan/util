#ifndef JOYSTICK_H
#define JOYSTICK_H

//#include "util.hpp"
#include <stdlib.h>
#include <sstream> //ostringstream
#include <vector>

#include <unistd.h> //read
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <boost/thread.hpp>

struct js_state{
    std::vector<double> axis;
    std::vector<int> button;
};

class joystick{
private:
    struct js_event event_;
    struct js_state state_;
//    std::ostringstream js_name_;
    std::string js_name_;

    int js_;
    unsigned long axis_num_, button_num_;
    bool stopLoop_flag_;

    std::vector<int> axis_calibrate_;
    struct js_state processJs();

    boost::thread jsThread;

public:
    joystick(int js_num=0);
    ~joystick();

    struct js_state readJs();
    void loopReadJs();
    struct js_state getState();
    void calibrateAxis(std::vector<int> shift);
    void stopLoop();
};

#endif // JOYSTICK_H
