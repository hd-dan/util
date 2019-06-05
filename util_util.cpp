#include "util_util.h"

std::string checkPath(std::string path){
    if (path.at(0)=='~'){
        path.erase(0,1);
        path= std::string("/home/") + getenv("LOGNAME") + path;
    }

    std::string dir= path.substr(0,path.find_last_of('/'));
    struct stat st;
    if (stat(dir.c_str(),&st)!=0){
        mkdir(dir.c_str(),S_IRWXU);
    }
    return path;
}
