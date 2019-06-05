#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <iostream>
#include <vector>
#include <math.h>

template <class num>
num deg2rad(num x){
    return M_PI/180.0*x;
}

template <class num>
num rad2deg(num x){
    return 180.0/M_PI*x;
}

template <class num>
std::vector<num> deg2rad(std::vector<num> x){
    for (unsigned int i=0;i<x.size();i++){
        x.at(i)*= M_PI/180.0;
    }
    return x;
}

template <class num>
std::vector<num> rad2deg(std::vector<num> x){
    for (unsigned int i=0;i<x.size();i++){
        x.at(i)*= 180.0/M_PI;
    }
    return x;
}

template <class num>
int sign(num x, double thres=1e-10){
    return(x<thres)? -1: 1;
}




#endif // MATH_UTIL_H
