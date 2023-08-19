TEMPLATE = app
CONFIG += console c++ll
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += link_pkgconfig
PKGCONFIG += opencv4

LIBS += -lzbar

SOURCES += \
        qr_yt.cpp
