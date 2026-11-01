QT = core
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= debug_and_release

INCLUDEPATH += ..

SOURCES += \
    gnss_utils_smoke.cpp \
    ../gnss_utils.cpp

HEADERS += \
    ../gnss_utils.h \
    ../rtcmdecoder.h
