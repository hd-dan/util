#include <iostream>

#include "joystick.h"
#include "keyboard.h"
#include "util.hpp"

#include <unistd.h>
#include <termios.h>

#include "../commun/commun.h"

void testJoy(){
    std::string ip("127.0.0.1");
    commun plotClient(ip,8888,0);

    joystick js(0);
    js_state state;

    std::vector<double> data(2,0);
    do{
        state= js.getState();
        data.at(0)= state.axis.at(0);
        data.at(1)= state.axis.at(1);

        plotClient.sendData(data);

        usleep(1e4);
    }while (!state.button.at(7));
}

void testKb(){
    keyboard kb;
    char kbKey= kb.getKbKey();
    while (kbKey!='q'){
        if (kb.checkNewKey()){
            kbKey= kb.getKbKey();
            printf("Input: %c\n",kbKey);
        }
    }
}

int findLowestToVal(std::vector<double> x, double val){
    std::vector<double>::iterator low= std::lower_bound(x.begin(),x.end(),val);
    return int(low-x.begin());
}


int main(){
    std::cout << "Hello World!" << std::endl;

//    testJoy();
//    testKb();

//    std::vector<std::vector<double> >A= {{1,2,3},{4,1,2},{7,5,2}};
//    std::vector<std::vector<double> >B= {{6,2,1},{7,73,2},{3,2,12}};
//    std::vector<std::vector<double> >C= {{53,2,1},{56,2,1},{9,34,2}};
//    std::vector<double> x= {2,31,5};

    std::vector<double> h= {0.132,-0.596,-5.51};
    std::vector<double> v= {0,0.96,-0.025};

    std::vector<double> phv= project(h,v);

    print_vector("proj_h_on_v",phv);
    printf("dir: %.3f\n",dot(phv,v));

    return 0;
}
