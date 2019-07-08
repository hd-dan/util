#include "joystick.h"

joystick::joystick(int js_num):js_(0){
    std::string js_dir("/dev/input/js");
    js_dir+= std::to_string(js_num);

    js_= open(js_dir.c_str(), O_RDONLY);
    if (js_<1){
        printf("Could not open JoyStick js%d\n", js_num);
        return;
    }
    __u8 axis, button;
    char name[256];
    ioctl(js_, JSIOCGNAME(256), &name);
    ioctl(js_, JSIOCGAXES, &axis);
    ioctl(js_, JSIOCGBUTTONS, &button);

    axis_num_= (unsigned long) axis;
    button_num_= (unsigned long) button;
    js_name_= name;

    state_.axis= std::vector<double>(axis_num_,0);
    state_.button= std::vector<int>(button_num_,0);
    axis_calibrate_= std::vector<int>(axis_num_,0);

    printf("JOYSTICK: %s\n", js_name_.c_str());
    stopLoop_flag_= true;

    jsThread_= boost::thread(&joystick::loopReadJs, this);
}

joystick::~joystick(){
    stopLoop_flag_= true;
    jsThread_.interrupt();
    jsThread_.join();
    close(js_);
}

struct js_state joystick::readJs(){
    fd_set set;
    struct timeval tv;

    FD_ZERO(&set);
    FD_SET(js_, &set);
    tv.tv_sec=0; tv.tv_usec=0;

    int ret= select(js_+1, &set, NULL, NULL, &tv);
    if (ret>0 && FD_ISSET(js_,&set) ){
        joystick::processJs();
    }
    return state_;
}

struct js_state joystick::processJs(){
    ssize_t bytes= read(js_, &event_, sizeof(event_));
    if (bytes<1) return state_;

    unsigned long enumber= (unsigned long)event_.number;
    int val= event_.value;

    if (event_.type == JS_EVENT_AXIS){
        state_.axis.at(enumber)= double(val-axis_calibrate_.at(enumber) )/32767;
    }else if (event_.type == JS_EVENT_BUTTON){
        state_.button.at(enumber)= val;
    }
    return state_;
}

void joystick::loopReadJs(){
    stopLoop_flag_= false;
    while (!stopLoop_flag_){
        joystick::processJs();
        usleep(1000);
        boost::this_thread::interruption_point();
    }
    return;
}

struct js_state joystick::getState(){
    return state_;
}

void joystick::calibrateAxis(std::vector<int> shift){
//    assert both vector same length
    axis_calibrate_= shift;
    return;
}

void joystick::stopLoop(){
    stopLoop_flag_= true;
}
