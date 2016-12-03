#-------------------------------------------------
#
# Project created by QtCreator 2014-04-20T03:11:59
#
#-------------------------------------------------

QT       += core gui  widgets

TARGET = QtOficinas3
TEMPLATE = app

LIBS += `pkg-config --cflags --libs opencv`

include(./qextserialport-master/src/qextserialport.pri)

SOURCES += main.cpp\
        MainWindow.cpp \
    CVImageWidget.cpp \
    Environment.cpp \
    mapa.cpp \
    Robot.cpp \
    ImageProcessing.cpp \
    ControlInterface.cpp \
    CaptionWidget.cpp \
    Util.cpp

HEADERS  += MainWindow.h \
    CVImageWidget.h \
    Util.h \
    Environment.h \
    mapa.h \
    Robot.h \
    ImageProcessing.h \
    ControlInterface.h \
    CaptionWidget.h

FORMS    += MainWindow.ui
