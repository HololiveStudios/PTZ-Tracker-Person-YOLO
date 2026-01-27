// src/MainWindow.cpp - VERSÃO ULTRA-PRECISA v2.1
#include "MainWindow.h"
#include "VideoWidget.h"
#include "PTZPanel.h"
#include "LogPanel.h"
#include "CaptureEngine.h"
#include "PTZController.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusBar>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <QTimer>
#include <QInputDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), isRunning(false)
{
    setWindowTitle("PTZ Person Tracker Pro - v2.1 ULTRA (YOLO + Velocity Matching)");
    resize(1500, 950);
    
    setupUI();
    createMenuBar();
    createStatusBar();
    updateUIState(false);
    
    QTimer::singleShot(100, this, &MainWindow::refreshWebcams);
}

MainWindow::~MainWindow() {
    if (isRunning) onStopClicked();
}

void MainWindow::setupUI() {
    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    
    // ========== CONTROLES PRINCIPAIS ==========
    QGroupBox *controlGroup = new QGroupBox("Configuracoes");
    QHBoxLayout *controlLayout = new QHBoxLayout(controlGroup);
    
    controlLayout->addWidget(new QLabel("Webcam:"));
    webcamCombo = new QComboBox();
    webcamCombo->addItem("Carregando...", -1);
    controlLayout->addWidget(webcamCombo, 1);
    
    QPushButton *refreshBtn = new QPushButton("Refresh");
    refreshBtn->setMaximumWidth(60);
    refreshBtn->setToolTip("Atualizar lista de webcams");
    connect(refreshBtn, &QPushButton::clicked, this, &MainWindow::refreshWebcams);
    controlLayout->addWidget(refreshBtn);
    
    controlLayout->addWidget(new QLabel("PTZ:"));
    comPortCombo = new QComboBox();
    comPortCombo->addItem("Desabilitado");
    for (const auto &info : QSerialPortInfo::availablePorts()) {
        comPortCombo->addItem(info.portName());
    }
    controlLayout->addWidget(comPortCombo);
    
    controlLayout->addWidget(new QLabel("FPS:"));
    fpsSpinBox = new QSpinBox();
    fpsSpinBox->setRange(1, 60);
    fpsSpinBox->setValue(30);
    controlLayout->addWidget(fpsSpinBox);
    
    controlLayout->addWidget(new QLabel("Precisao:"));
    thresholdSpinBox = new QDoubleSpinBox();
    thresholdSpinBox->setRange(0.1, 0.9);
    thresholdSpinBox->setValue(0.5);
    thresholdSpinBox->setDecimals(2);
    thresholdSpinBox->setSuffix(" conf");
    connect(thresholdSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double val) {
        if (captureEngine) {
            captureEngine->setConfidenceThreshold(static_cast<float>(val));
            if (logPanel) logPanel->addLog(QString("Threshold: %1").arg(val), 0);
        }
    });
    controlLayout->addWidget(thresholdSpinBox);
    
    autoTrackCheckbox = new QCheckBox("Auto PTZ");
    connect(autoTrackCheckbox, &QCheckBox::toggled, [this](bool checked) {
        if (captureEngine) {
            captureEngine->setAutoTracking(checked);
            if (logPanel) {
                logPanel->addLog(checked ? "Auto PTZ ativado" : "Auto PTZ desativado", 
                               checked ? 1 : 0);
            }
        }
    });
    controlLayout->addWidget(autoTrackCheckbox);
    
    mainLayout->addWidget(controlGroup);
    
    // ========== CALIBRAÇÃO ==========
    QGroupBox *calibGroup = new QGroupBox("Calibracao Avancada");
    QHBoxLayout *calibLayout = new QHBoxLayout(calibGroup);
    
    calibLayout->addWidget(new QLabel("FOV H:"));
    fovHSpinBox = new QDoubleSpinBox();
    fovHSpinBox->setRange(10, 120);
    fovHSpinBox->setValue(60.0);
    fovHSpinBox->setSuffix(" deg");
    fovHSpinBox->setToolTip("Field of View Horizontal da camera PTZ");
    calibLayout->addWidget(fovHSpinBox);
    
    calibLayout->addWidget(new QLabel("FOV V:"));
    fovVSpinBox = new QDoubleSpinBox();
    fovVSpinBox->setRange(10, 90);
    fovVSpinBox->setValue(40.0);
    fovVSpinBox->setSuffix(" deg");
    fovVSpinBox->setToolTip("Field of View Vertical da camera PTZ");
    calibLayout->addWidget(fovVSpinBox);
    
    calibLayout->addWidget(new QLabel("Latencia:"));
    latencySpinBox = new QDoubleSpinBox();
    latencySpinBox->setRange(50, 500);
    latencySpinBox->setValue(203.0);
    latencySpinBox->setSuffix(" ms");
    latencySpinBox->setToolTip("Latencia total do sistema (capture + YOLO + VISCA + mecanica)");
    calibLayout->addWidget(latencySpinBox);
    
    QPushButton *calibBtn = new QPushButton("Calibrar");
    calibBtn->setToolTip("Assistente de calibracao automatica");
    connect(calibBtn, &QPushButton::clicked, this, &MainWindow::onCalibrateClicked);
    calibLayout->addWidget(calibBtn);
    
    calibLayout->addStretch();
    
    mainLayout->addWidget(calibGroup);
    
    // ========== ÁREA PRINCIPAL ==========
    QSplitter *splitter = new QSplitter(Qt::Horizontal);
    
    videoWidget = new VideoWidget();
    splitter->addWidget(videoWidget);
    
    QWidget *rightPanel = new QWidget();
    QVBoxLayout *rightLayout = new QVBoxLayout(rightPanel);
    
    ptzPanel = new PTZPanel();
    rightLayout->addWidget(ptzPanel);
    
    // ========== STATUS DE MOVIMENTO ==========
    QGroupBox *motionGroup = new QGroupBox("Estado do Movimento");
    QVBoxLayout *motionLayout = new QVBoxLayout(motionGroup);
    
    motionLabel = new QLabel("Aguardando deteccao...");
    motionLabel->setStyleSheet("font-family: 'Consolas'; font-size: 11px; color: #4CAF50;");
    motionLayout->addWidget(motionLabel);
    
    rightLayout->addWidget(motionGroup);
    
    logPanel = new LogPanel();
    rightLayout->addWidget(logPanel);
    
    splitter->addWidget(rightPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 1);
    
    mainLayout->addWidget(splitter, 1);
    
    // ========== BOTÕES ==========
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    
    startButton = new QPushButton("Iniciar Deteccao Ultra-Precisa");
    startButton->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; font-size: 14px;");
    connect(startButton, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    buttonLayout->addWidget(startButton);
    
    stopButton = new QPushButton("Parar");
    stopButton->setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px; font-size: 14px;");
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    buttonLayout->addWidget(stopButton);
    
    buttonLayout->addStretch();
    
    mainLayout->addLayout(buttonLayout);
    
    setCentralWidget(central);
    
    logPanel->addLog("Sistema YOLO Ultra-Preciso iniciado", 1);
    logPanel->addLog("Kalman Adaptativo ativo", 1);
    logPanel->addLog("Velocity Matching ativo", 1);
    logPanel->addLog("Predicao de aceleracao ativa", 1);
}

void MainWindow::onCalibrateClicked() {
    QMessageBox::information(this, "Calibracao Automatica",
        "<h3>Assistente de Calibracao</h3>"
        "<p><b>Passo 1: Medir FOV</b></p>"
        "<ol>"
        "<li>Posicione um objeto de tamanho conhecido (ex: regua de 1m) a 2m de distancia</li>"
        "<li>Anote quantos pixels o objeto ocupa na tela</li>"
        "<li>Calcule: FOV = 2 * arctan((tamanho_real / 2) / distancia) * (180/pi)</li>"
        "</ol>"
        "<p><b>Passo 2: Medir Latencia</b></p>"
        "<ol>"
        "<li>Mova a camera manualmente com velocidade 24</li>"
        "<li>Use um cronometro para medir o tempo de resposta</li>"
        "<li>Latencia tipica: 150-250ms</li>"
        "</ol>"
        "<p><b>Valores recomendados:</b></p>"
        "<ul>"
        "<li>FOV H: 50-70 graus (cameras PTZ tipicas)</li>"
        "<li>FOV V: 35-45 graus</li>"
        "<li>Latencia: 180-220ms</li>"
        "</ul>");
    
    bool ok;
    QString cameraModel = QInputDialog::getItem(this, "Modelo da Camera",
        "Selecione o modelo da sua camera PTZ:",
        {"Personalizado", "PTZ Generica (60/40)", "Sony EVI-D70 (48/36)", 
         "Canon VC-C50i (54/41)", "Logitech PTZ Pro 2 (90/70)"},
        0, false, &ok);
    
    if (ok) {
        if (cameraModel.contains("Generica")) {
            fovHSpinBox->setValue(60.0);
            fovVSpinBox->setValue(40.0);
        } else if (cameraModel.contains("Sony")) {
            fovHSpinBox->setValue(48.0);
            fovVSpinBox->setValue(36.0);
        } else if (cameraModel.contains("Canon")) {
            fovHSpinBox->setValue(54.0);
            fovVSpinBox->setValue(41.0);
        } else if (cameraModel.contains("Logitech")) {
            fovHSpinBox->setValue(90.0);
            fovVSpinBox->setValue(70.0);
        }
        
        logPanel->addLog("Calibracao aplicada: " + cameraModel, 1);
    }
}

void MainWindow::refreshWebcams() {
    if (!webcamCombo) return;
    
    webcamCombo->clear();
    logPanel->addLog("Procurando webcams...", 0);
    
    int foundCameras = 0;
    
    for (int i = 0; i < 20; i++) {
        cv::VideoCapture cap;
        
        try {
            cap.open(i, cv::CAP_ANY);
            
            if (cap.isOpened()) {
                cv::Mat testFrame;
                bool canRead = cap.read(testFrame);
                
                if (canRead && !testFrame.empty()) {
                    QString cameraName;
                    
                    #ifdef _WIN32
                        std::string backend = cap.getBackendName();
                        if (!backend.empty() && backend != "FFMPEG" && backend != "GStreamer") {
                            cameraName = QString::fromStdString(backend) + QString(" (ID: %1)").arg(i);
                        } else {
                            cameraName = QString("Camera %1").arg(i);
                        }
                    #elif __linux__
                        cameraName = QString("/dev/video%1").arg(i);
                    #elif __APPLE__
                        cameraName = QString("Camera %1").arg(i);
                    #else
                        cameraName = QString("Camera %1").arg(i);
                    #endif
                    
                    if (i >= 10) {
                        cameraName += " (Virtual?)";
                    }
                    
                    webcamCombo->addItem(cameraName, i);
                    foundCameras++;
                }
                
                cap.release();
            }
        } catch (...) {
        }
    }
    
    if (foundCameras == 0) {
        webcamCombo->addItem("Nenhuma webcam detectada", -1);
        logPanel->addLog("Nenhuma webcam encontrada", 3);
    } else {
        logPanel->addLog(QString("%1 webcam(s) encontrada(s)").arg(foundCameras), 1);
    }
}

void MainWindow::createMenuBar() {
    QMenuBar *menuBar = new QMenuBar(this);
    
    QMenu *fileMenu = menuBar->addMenu("&Arquivo");
    fileMenu->addAction("&Sair", this, &QWidget::close);
    
    QMenu *toolsMenu = menuBar->addMenu("&Ferramentas");
    toolsMenu->addAction("Calibrar Sistema", this, &MainWindow::onCalibrateClicked);
    
    QMenu *helpMenu = menuBar->addMenu("&Ajuda");
    helpMenu->addAction("Sobre", [this]() {
        QMessageBox::about(this, "Sobre", 
            "<h2>PTZ Tracker Pro v2.1 ULTRA</h2>"
            "<p><b>Rastreamento Ultra-Preciso com IA</b></p>"
            "<p>YOLOv8 ONNX<br>"
            "Kalman Filter Adaptativo<br>"
            "Predicao de Aceleracao<br>"
            "Velocity Matching<br>"
            "PID Dinamico<br>"
            "Deteccao de Fuga<br>"
            "Compensacao de Latencia<br>"
            "Controle PTZ VISCA</p>");
    });
    
    setMenuBar(menuBar);
}

void MainWindow::createStatusBar() {
    statusLabel = new QLabel("Pronto");
    fpsLabel = new QLabel("FPS: --");
    detectionLabel = new QLabel("Deteccoes: 0");
    
    statusBar()->addWidget(statusLabel, 1);
    statusBar()->addPermanentWidget(detectionLabel);
    statusBar()->addPermanentWidget(fpsLabel);
}

void MainWindow::onStartClicked() {
    if (isRunning) return;
    
    int cameraIndex = webcamCombo->currentData().toInt();
    
    if (cameraIndex < 0) {
        QMessageBox::warning(this, "Erro", "Nenhuma webcam selecionada ou disponivel!");
        return;
    }
    
    logPanel->addLog("Iniciando captura ULTRA-PRECISA...", 1);
    
    captureEngine = std::make_unique<CaptureEngine>(
        std::to_string(cameraIndex),
        fpsSpinBox->value(),
        static_cast<float>(thresholdSpinBox->value())
    );
    
    // Aplicar calibração
    captureEngine->setCameraFOV(
        static_cast<float>(fovHSpinBox->value()),
        static_cast<float>(fovVSpinBox->value())
    );
    captureEngine->setSystemLatency(static_cast<float>(latencySpinBox->value()));
    
    captureEngine->setAutoTracking(autoTrackCheckbox->isChecked());
    
    if (comPortCombo->currentText() != "Desabilitado") {
        ptzController = std::make_unique<PTZController>(
            comPortCombo->currentText().toStdString(), 9600
        );
        ptzPanel->setEnabled(true);
        
        connect(ptzPanel, &PTZPanel::panTiltRequested, 
                ptzController.get(), &PTZController::panTilt);
        connect(ptzPanel, &PTZPanel::zoomRequested,
                ptzController.get(), &PTZController::zoom);
        connect(ptzPanel, &PTZPanel::homeRequested,
                ptzController.get(), &PTZController::home);
        connect(ptzPanel, &PTZPanel::menuRequested,
                ptzController.get(), &PTZController::openMenu);
        
        connect(captureEngine.get(), &CaptureEngine::ptzAdjustmentNeeded,
                ptzController.get(), &PTZController::panTilt);
        
        logPanel->addLog("PTZ conectado", 1);
    }
    
    connect(captureEngine.get(), &CaptureEngine::frameReady,
            this, &MainWindow::onFrameReady);
    connect(captureEngine.get(), &CaptureEngine::fpsUpdated,
            this, &MainWindow::onFPSUpdate);
    connect(captureEngine.get(), &CaptureEngine::detectionCount,
            this, &MainWindow::onDetectionCount);
    connect(captureEngine.get(), &CaptureEngine::motionStateUpdated,
            this, &MainWindow::onMotionStateUpdate);
    
    captureEngine->start();
    
    isRunning = true;
    updateUIState(true);
    statusLabel->setText("Capturando ULTRA-PRECISO...");
    logPanel->addLog("Todos os sistemas ativos!", 1);
    logPanel->addLog(QString("FOV: %1 x %2 deg").arg(fovHSpinBox->value()).arg(fovVSpinBox->value()), 0);
    logPanel->addLog(QString("Latencia: %1 ms").arg(latencySpinBox->value()), 0);
}

void MainWindow::onStopClicked() {
    if (!isRunning) return;
    
    captureEngine->stop();
    captureEngine.reset();
    
    if (ptzController) {
        ptzController->stop();
        ptzController.reset();
    }
    
    isRunning = false;
    updateUIState(false);
    statusLabel->setText("Parado");
    fpsLabel->setText("FPS: --");
    detectionLabel->setText("Deteccoes: 0");
    motionLabel->setText("Aguardando deteccao...");
    logPanel->addLog("Captura encerrada", 0);
}

void MainWindow::onFrameReady(const QImage &frame) {
    videoWidget->setFrame(frame);
}

void MainWindow::onFPSUpdate(double fps) {
    fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
}

void MainWindow::onDetectionCount(int count) {
    detectionLabel->setText(QString("Deteccoes: %1").arg(count));
}

void MainWindow::onMotionStateUpdate(const QString &state) {
    motionLabel->setText(state);
}

void MainWindow::updateUIState(bool running) {
    startButton->setEnabled(!running);
    stopButton->setEnabled(running);
    webcamCombo->setEnabled(!running);
    comPortCombo->setEnabled(!running);
    fpsSpinBox->setEnabled(!running);
    fovHSpinBox->setEnabled(!running);
    fovVSpinBox->setEnabled(!running);
    latencySpinBox->setEnabled(!running);
}