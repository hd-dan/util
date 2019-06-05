#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <iostream>
#include <termios.h>
#include <boost/thread.hpp>

class keyboard{
private:

    struct termios termSet_;

    char kbKey_;
    bool fNewKey_;

    bool fStop_;
    boost::thread tReadKb_;

public:
    keyboard();
    ~keyboard();

    char getKbKey();
    bool checkNewKey();

    void loopReadKb();
    void readKb();

    void closeKb();
};

#endif // KEYBOARD_H
