include(./patchwork/PatchWork.pri)
#include(./linefit/LineFit.pri)
include(./covariance/Covariance.pri)

HEADERS += \
    $$PWD/perceptioninterface.h \
    $$PWD/euclideancluster.h \
    $$PWD/boundingbox.h \
    $$PWD/objecttracker.h

SOURCES += \
    $$PWD/euclideancluster.cpp \
    $$PWD/perceptioninterface.cpp \
    $$PWD/objecttracker.cpp

DISTFILES += \
    $$PWD/readme.txt
