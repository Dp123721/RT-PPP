QT = core
CONFIG += console c++17
CONFIG -= app_bundle

INCLUDEPATH += ..

SOURCES += \
    pppjsonwriter_smoke.cpp \
    ../pppjsonwriter.cpp

HEADERS += \
    ../pppjsonwriter.h \
    ../ppp_engine.h
