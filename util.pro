TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    joystick.cpp \
    ../commun/commun.cpp

HEADERS += \
    joystick.h \
    util.hpp \
    ../commun/commun.h

LIBS += -lboost_system -lboost_thread
