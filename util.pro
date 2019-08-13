TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11


SOURCES += \
        main.cpp \
    joystick.cpp \
    ../commun/commun.cpp \
    keyboard.cpp \
    util_util.cpp

HEADERS += \
    joystick.h \
    util.hpp \
    ../commun/commun.h \
    math_util.hpp \
    linear_algebra.hpp \
    matrix_util.hpp \
    keyboard.h \
    util_util.h

macx {
    INCLUDEPATH += /usr/local/Cellar/boost/1.70.0/include/
    LIBS += -L/usr/local/Cellar/boost/1.70.0/lib -lboost_thread-mt
}
unix:!macx {
    LIBS += -lboost_thread
}
LIBS += -lboost_system -pthread
