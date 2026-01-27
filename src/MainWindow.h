// src/MainWindow.h - VERSÃO ULTRA-PRECISA v2.1
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <memory>
#include <opencv2/opencv.hpp>

class VideoWidget;
class PTZPanel;
class LogPanel;
class CaptureEngine;
class PTZController;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onStartClicked();
    void onStopClicked();
    void onFrameReady(const QImage &frame);
    void onFPSUpdate(double fps);
    void onDetectionCount(int count);
    void onMotionStateUpdate(const QString &state);
    void refreshWebcams();
    void onCalibrateClicked();

private:
    void setupUI();
    void createMenuBar();
    void createStatusBar();
    void updateUIState(bool running);
    
    VideoWidget *videoWidget;
    PTZPanel *ptzPanel;
    LogPanel *logPanel;
    
    QComboBox *webcamCombo;
    QComboBox *comPortCombo;
    QSpinBox *fpsSpinBox;
    QDoubleSpinBox *thresholdSpinBox;
    QDoubleSpinBox *fovHSpinBox;
    QDoubleSpinBox *fovVSpinBox;
    QDoubleSpinBox *latencySpinBox;
    QCheckBox *autoTrackCheckbox;
    
    QPushButton *startButton;
    QPushButton *stopButton;
    QLabel *fpsLabel;
    QLabel *statusLabel;
    QLabel *detectionLabel;
    QLabel *motionLabel;
    
    std::unique_ptr<CaptureEngine> captureEngine;
    std::unique_ptr<PTZController> ptzController;
    
    bool isRunning;
};

#endif