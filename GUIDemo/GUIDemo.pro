QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += warn_on
CONFIG += console
#------------------------------------------------------------
# Source Files
#------------------------------------------------------------

FORMS += \
    GUIDemo.ui \
    Registration.ui \

HEADERS += \
    CustomFrames.h \
    GUIDemo.h \
    MultiRadar.h \
    OverlayManager.h \
    QControlUtils.h \
    ScannerInfo.h \
    TabBase.h \
    TabAdvanced.h \
    TabBScan.h \
    TabGuardZone.h \
    TabImage.h \
    TabInstallation.h \
    TabFeatures.h \
    TabNewFunction.h \
    TabSectorBlanking.h \
    TabPPI.h \
    TabTargets.h \ \
    TargetTracking/RadarFiltering.h \
    TargetTracking/TargetTracking.h \
    TargetTracking/TargetTrackingData.h \
    TargetTracking/math_utils.h \
    TargetTracking/timecounter.h \
    datarecorder.h \
    datareplay.h \
    datatransimission.h

SOURCES += \
    CustomFrames.cpp \
    GUIDemo.cpp \
    MultiRadar.cpp \
    OverlayManager.cpp \
    QControlUtils.cpp \
    ScannerInfo.cpp \
    TabBase.cpp \
    TabAdvanced.cpp \
    TabBScan.cpp \
    TabGuardZone.cpp \
    TabImage.cpp \
    TabInstallation.cpp \
    TabFeatures.cpp \
    TabNewFunction.cpp \
    TabSectorBlanking.cpp \
    TabPPI.cpp \
    TabTargets.cpp \
    TargetTracking/RadarFiltering.cpp \
    TargetTracking/math_utils.cpp \
    TargetTracking/timecounter.cpp \
    datarecorder.cpp \
    datareplay.cpp \
    datatransimission.cpp \
    main.cpp \

#------------------------------------------------------------
# Include directories
#------------------------------------------------------------

INCLUDES = \
    ../third_party/SIMRAD_SDK/SDK_3.0.02/include \
    ../third_party/pyclustering/ccore/include

#------------------------------------------------------------
# UI Generation
#------------------------------------------------------------

CONFIG(debug,   debug|release): TARGET_DIR = debug
CONFIG(release, debug|release): TARGET_DIR = release

UI_DIR = GeneratedFiles
MOC_DIR = $${UI_DIR}/$${TARGET_DIR}
INCLUDEPATH += $${UI_DIR}

#------------------------------------------------------------
# Libraries
#------------------------------------------------------------

LIBS += \
#    -L../../lib/$${TARGET_DIR} \
  	-lNRPClient \
  	-lNRPPPI \
        -lsqlite3 \
        -L$$PWD/../third_party/pyclustering/ccore/libs/ -lpyclustering
   
#------------------------------------------------------------
# Final config
#------------------------------------------------------------
  
#set the qmake variables
DEPENDPATH += $$INCLUDES
INCLUDEPATH += $$INCLUDES

