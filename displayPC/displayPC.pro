# 1. It's necessary that add ros into env path when launch qtcreator.
# Launch qtcreator in a terminal where ros is in env path or add ros into qtcreator shortcut(Exe=bash -i -c $path/qtcreator)

# 2. Limit num of thread during building. I failed to build with 16 threads and succeeded with 8 threads.

# 3. I built with qt 5.12.8 and cooresponding qtcreator.

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    parsedataform.cpp

HEADERS += \
mainwindow.h \
        myutility/cutility.h \
        myutility/qutility.h \
        myutility/rutility.h \
        myprocess/perception/boundingbox.h \
        myutility/mymath.h \
        parsedataform.h \
        global.h

FORMS += \
    mainwindow.ui \
    pic.ui \
    parsedataform.ui

include(./widget/widget.pri)
include(./myprocess/myprocess.pri)
include(./viewer/viewer.pri)
include(./remote/remote.pri)
include(./third_party/yaml-cpp/yaml-cpp.pri)

INCLUDEPATH += /usr/include/pcl-1.10
LIBS += /usr/lib/x86_64-linux-gnu/libpcl_*.so

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /opt/ros/noetic/include
DEPENDPATH += /opt/ros/noetic/include
LIBS += -L/opt/ros/noetic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime

INCLUDEPATH += /usr/include/boost
#LIBS += -L/usr/lib/x86_64-linux-gnu -lboost
LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

INCLUDEPATH += /usr/include/vtk-7.1
LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so

#INCLUDEPATH += /usr/include/opencv2/imgproc.hpp
#INCLUDEPATH += /usr/include/opencv2/core.hpp

INCLUDEPATH += /usr/include/opencv4
#LIBS += /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_core.so
LIBS += /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so

DISTFILES += \
    config/config.ini \
    source/add.jpeg \
    source/last.png \
    source/minus.png \
    source/next.png \
    source/pause.png \
    source/play.png \
    source/ico.ico \
    source/logo.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
