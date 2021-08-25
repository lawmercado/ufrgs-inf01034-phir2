TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++0x

unix {
    INCLUDEPATH += /usr/lib/gcc/x86_64-linux-gnu/9/include
}

SOURCES += \
    src/GlutClass.cpp \
    src/PioneerBase.cpp \
    src/Grid.cpp \
    src/main.cpp \
    src/Robot.cpp \
    src/Utils.cpp \

OTHER_FILES += \
    CONTROLE.txt

HEADERS += \
    src/GlutClass.h \
    src/Grid.h \
    src/PioneerBase.h \
    src/Robot.h \
    src/Utils.h \


INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/Aria/lib -lAria

LIBS+=-lpthread -lglut -ldl -lrt -lGL -lfreeimage
