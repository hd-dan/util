#include <iostream>

#include "joystick.h"
#include "util.hpp"

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

int main(){
    std::cout << "Hello World!" << std::endl;

//    testJoy();
    std::vector<double> x(11,0);
    for (unsigned int i=0;i<x.size();i++){
        x.at(i)= i*0.7289;
    }
    print_vector("x",x);

    double lowest= 4;
    std::vector<double>::iterator low= std::lower_bound(x.begin(),x.end(),lowest);

    if (low!=x.end()){
        long window= long(x.end()-low);
        x= std::vector<double>(x.end()-window,x.end());
//        x= std::vector<double>(low,x.end());
        printf("Position: %d\n",window);
    }

    print_vector("new x",x);

    return 0;
}
