QT -= core
QT -= gui

TARGET = simConvexDecompose
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
DEFINES += SIM_MATH_DOUBLE
CONFIG += shared plugin
INCLUDEPATH += "../include"
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
    QMAKE_CXXFLAGS += -fvisibility=hidden
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
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
}

SOURCES += \
    ../include/simLib/simLib.cpp \
    sourceCode/simConvexDecompose.cpp \
    external/hacd/hacdGraph.cpp \
    external/hacd/hacdHACD.cpp \
    external/hacd/hacdICHull.cpp \
    external/hacd/hacdManifoldMesh.cpp \
    external/hacd/hacdMeshDecimator.cpp \
    external/hacd/hacdMicroAllocator.cpp \
    external/hacd/hacdRaycastMesh.cpp \
    external/vhacd/src/btAlignedAllocator.cpp \
    external/vhacd/src/btConvexHullComputer.cpp \
    external/vhacd/src/FloatMath.cpp \
    external/vhacd/src/VHACD.cpp \
    external/vhacd/src/VHACD-ASYNC.cpp \
    external/vhacd/src/vhacdICHull.cpp \
    external/vhacd/src/vhacdManifoldMesh.cpp \
    external/vhacd/src/vhacdMesh.cpp \
    external/vhacd/src/vhacdRaycastMesh.cpp \
    external/vhacd/src/vhacdVolume.cpp \

HEADERS +=\
    ../include/simLib/simLib.h \
    sourceCode/simConvexDecompose.h \
    external/hacd/hacdCircularList.h \
    external/hacd/hacdGraph.h \
    external/hacd/hacdHACD.h \
    external/hacd/hacdICHull.h \
    external/hacd/hacdManifoldMesh.h \
    external/hacd/hacdMeshDecimator.h \
    external/hacd/hacdMicroAllocator.h \
    external/hacd/hacdRaycastMesh.h \
    external/hacd/hacdSArray.h \
    external/hacd/hacdVector.h \
    external/hacd/hacdVersion.h \
    external/vhacd/public/VHACD.h \
    external/vhacd/inc/btAlignedAllocator.h \
    external/vhacd/inc/btAlignedObjectArray.h \
    external/vhacd/inc/btConvexHullComputer.h \
    external/vhacd/inc/btMinMax.h \
    external/vhacd/inc/btScalar.h \
    external/vhacd/inc/btVector3.h \
    external/vhacd/inc/FloatMath.h \
    external/vhacd/inc/vhacdCircularList.h \
    external/vhacd/inc/vhacdlCHull.h \
    external/vhacd/inc/vhacdManifoldMesh.h \
    external/vhacd/inc/vhacdMesh.h \
    external/vhacd/inc/vhacdMutex.h \
    external/vhacd/inc/vhacdRaycastMesh.h \
    external/vhacd/inc/vhacdSArray.h \
    external/vhacd/inc/vhacdTimer.h \
    external/vhacd/inc/vhacdVector.h \
    external/vhacd/inc/vhacdVHACD.h \
    external/vhacd/inc/vhacdVolume.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}