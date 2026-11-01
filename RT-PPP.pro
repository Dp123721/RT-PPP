QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# Keep qmake generated files out of the source tree. Run qmake from a
# toolchain-specific directory under build (for example build/linux).
OBJECTS_DIR = $$OUT_PWD/obj
MOC_DIR = $$OUT_PWD/generated
UI_DIR = $$OUT_PWD/generated
RCC_DIR = $$OUT_PWD/generated
DESTDIR = $$OUT_PWD/bin

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ambbie.cpp \
    ambfix.cpp \
    gfs_zwd.cpp \
    gpt3_vmf3.cpp \
    gnss_ephemeris.cpp \
    gnss_signals.cpp \
    gnss_utils.cpp \
    main.cpp \
    mainwindow.cpp \
    outputworker.cpp \
    postppp.cpp \
    ppp_engine.cpp \
    pppruntimeservice.cpp \
    pppjsonwriter.cpp \
    telemetrywriter.cpp \
    ppp_worker.cpp \
    rinexobswriter.cpp \
    rtcmprocessingservice.cpp \
    sp3writer.cpp \
    rtcmdecoder.cpp \
    settingsstore.cpp \
    streamclients.cpp

HEADERS += \
    ambbie.h \
    ambfix.h \
    gfs_zwd.h \
    gpt3_vmf3.h \
    gnss_ephemeris.h \
    gnss_signals.h \
    gnss_utils.h \
    mainwindow.h \
    outputworker.h \
    postppp.h \
    ppp_engine.h \
    pppruntimeservice.h \
    pppjsonwriter.h \
    telemetrywriter.h \
    ppp_worker.h \
    rinexobswriter.h \
    rtcmprocessingservice.h \
    sp3writer.h \
    rtcmdecoder.h \
    settingsstore.h \
    streamclients.h

FORMS += \
    ui/mainwindow.ui \
    ui/addntripcasterdialog.ui \
    ui/addtcpdialog.ui \
    ui/addmqttdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
