QT += core gui widgets serialport multimedia

CONFIG += c++17

# Evita warnings desnecessários
DEFINES += QT_DEPRECATED_WARNINGS

# ---- Fontes ----
SOURCES += \
    src/main.cpp \
    src/MainWindow.cpp \
    src/VideoWidget.cpp \
    src/PTZPanel.cpp \
    src/LogPanel.cpp \
    src/CaptureEngine.cpp \
    src/PTZController.cpp \
    src/YOLODetector.cpp

HEADERS += \
    src/MainWindow.h \
    src/VideoWidget.h \
    src/PTZPanel.h \
    src/LogPanel.h \
    src/CaptureEngine.h \
    src/PTZController.h \
    src/YOLODetector.h

# ---- OpenCV 4.12.0 ----
OPENCV_DIR = C:/opencv/build

INCLUDEPATH += $$OPENCV_DIR/include

# OpenCV libs (MSVC)
LIBS += -L$$OPENCV_DIR/x64/vc16/lib

CONFIG(debug, debug|release) {
    LIBS += -lopencv_world4120d
} else {
    LIBS += -lopencv_world4120
}

# ---- Copiar modelo YOLO ----
yolo_model.files = yolov8n.onnx
yolo_model.path = $$OUT_PWD
INSTALLS += yolo_model

# ---- Windows específico ----
win32 {
    # Evita console em release
    CONFIG(release, debug|release): CONFIG += windows
}

# Default rules for deployment
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
