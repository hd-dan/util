#include "keyboard.h"

keyboard::keyboard():kbKey_(' '),fNewKey_(0),fStop_(0){
    if (tcgetattr(STDIN_FILENO,&termSet_)<0){
        printf("Failed to get current TermIO\n");
        exit(1);
    }
    termSet_.c_lflag &= ~ICANON;
    if (tcsetattr(STDIN_FILENO,TCSANOW,&termSet_)<0){
        printf("Failed to set TermIO to without buffer\n");
        exit(1);
    }

    tReadKb_= boost::thread(&keyboard::loopReadKb,this);
}

keyboard::~keyboard(){
    keyboard::closeKb();
}

char keyboard::getKbKey(){
    fNewKey_=0;
    return kbKey_;
}

bool keyboard::checkNewKey(){
    return fNewKey_;
}

void keyboard::loopReadKb(){
    fStop_=0;
    while(!fStop_){
        keyboard::readKb();
        usleep(1e4);
        boost::this_thread::interruption_point();
    }
    return;
}

void keyboard::readKb(){
//    std::cin>>kbKey_;
    kbKey_=char(getchar());
    fNewKey_=1;
    return;
}

void keyboard::closeKb(){
    fStop_=1;
    tReadKb_.interrupt();
    tReadKb_.join();

    tcgetattr(STDIN_FILENO,&termSet_);
    termSet_.c_lflag |= ICANON;
    if (tcsetattr(STDIN_FILENO,TCSANOW,&termSet_)<0)
        printf("Fail to set Terminal IO back to normal\n");
    return;
}
