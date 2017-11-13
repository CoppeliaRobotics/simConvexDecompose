QT -= core
QT -= gui

TARGET = v_repExtConvexDecompose
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
CONFIG += shared
INCLUDEPATH += "../include"
INCLUDEPATH += "../v_repMath"
INCLUDEPATH += "hacd"
INCLUDEPATH += "vhacd/inc"
INCLUDEPATH += "vhacd/public"
INCLUDEPATH += "vhacd/src"

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CXXFLAGS += -fpermissive

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs

    QMAKE_CFLAGS += -fpermissive
}

win32 {
    DEFINES += WIN_VREP
}

macx {
    DEFINES += MAC_VREP
}

unix:!macx {
    DEFINES += LIN_VREP
}

SOURCES += \
    ../common/v_repLib.cpp \
    v_repExtConvexDecompose.cpp \
    hacd/hacdGraph.cpp \
    hacd/hacdHACD.cpp \
    hacd/hacdICHull.cpp \
    hacd/hacdManifoldMesh.cpp \
    hacd/hacdMeshDecimator.cpp \
    hacd/hacdMicroAllocator.cpp \
    hacd/hacdRaycastMesh.cpp \
    vhacd/src/btAlignedAllocator.cpp \
    vhacd/src/btConvexHullComputer.cpp \
    vhacd/src/VHACD.cpp \
    vhacd/src/vhacdICHull.cpp \
    vhacd/src/vhacdManifoldMesh.cpp \
    vhacd/src/vhacdMesh.cpp \
    vhacd/src/vhacdVolume.cpp \

HEADERS +=\
    ../include/v_repLib.h \
    v_repExtConvexDecompose.h \
    hacd/hacdCircularList.h \
    hacd/hacdGraph.h \
    hacd/hacdHACD.h \
    hacd/hacdICHull.h \
    hacd/hacdManifoldMesh.h \
    hacd/hacdMeshDecimator.h \
    hacd/hacdMicroAllocator.h \
    hacd/hacdRaycastMesh.h \
    hacd/hacdSArray.h \
    hacd/hacdVector.h \
    hacd/hacdVersion.h \
    vhacd/inc/btAlignedAllocator.h \
    vhacd/inc/btAlignedObjectArray.h \
    vhacd/inc/btConvexHullComputer.h \
    vhacd/inc/btMinMax.h \
    vhacd/inc/btScalar.h \
    vhacd/inc/btVector3.h \
    vhacd/public/VHACD.h \
    vhacd/inc/vhacdCircular.h \
    vhacd/inc/vhacdlCHull.h \
    vhacd/inc/vhacdManifoldMesh.h \
    vhacd/inc/vhacdMesh.h \
    vhacd/inc/vhacdMesh.h \
    vhacd/inc/vhacdMutex.h \
    vhacd/inc/vhacdSArray.h \
    vhacd/inc/vhacdTimer.h \
    vhacd/inc/vhacdVector.h \
    vhacd/inc/vhacdVHACD.h \
    vhacd/inc/vhacdVolume.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
