QT += core
QT -= gui

CONFIG += c++11

TARGET = RGBDFrames2PC
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(pcl_32.pri)
#include(pcl_64.pri)

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410d
else:unix: LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410

INCLUDEPATH += $$PWD/../../CV/opencv/build/include
DEPENDPATH += $$PWD/../../CV/opencv/build/include



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410d
else:unix: LIBS += -L$$PWD/../../CV/opencv/build/x64/vc15/lib/ -lopencv_world410

INCLUDEPATH += $$PWD/../../CV/opencv/build/include
DEPENDPATH += $$PWD/../../CV/opencv/build/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../CV/opencv/build/x64/vc15/lib/libopencv_world410.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../CV/opencv/build/x64/vc15/lib/libopencv_world410d.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../CV/opencv/build/x64/vc15/lib/opencv_world410.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../CV/opencv/build/x64/vc15/lib/opencv_world410d.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../CV/opencv/build/x64/vc15/lib/libopencv_world410.a




unix|win32: LIBS += -L$$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/ -lk4a

INCLUDEPATH += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/build/native/include
DEPENDPATH += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/build/native/include

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/k4a.lib
else:unix|win32-g++: PRE_TARGETDEPS += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/libk4a.a

unix|win32: LIBS += -L$$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/ -lk4arecord

INCLUDEPATH += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/build/native/include
DEPENDPATH += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/build/native/include

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/k4arecord.lib
else:unix|win32-g++: PRE_TARGETDEPS += $$PWD/Microsoft.Azure.Kinect.Sensor.1.4.1/lib/native/amd64/release/libk4arecord.a

HEADERS +=

DISTFILES +=
