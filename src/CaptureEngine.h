// src/CaptureEngine.h - VERSÃO ULTRA-PRECISA v2.1
#ifndef CAPTUREENGINE_H
#define CAPTUREENGINE_H

#include <QObject>
#include <QThread>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <memory>
#include <deque>
#include <chrono>
#include "YOLODetector.h"

// ========== STRUCTS DE CONTROLE ==========

struct AdaptivePIDGains {
    float Kp, Ki, Kd;
};

struct VelocityProfile {
    float target_velocity;      // Velocidade da pessoa (graus/s)
    float camera_velocity;      // Velocidade da câmera (graus/s)
    float required_velocity;    // Velocidade necessária
    float acceleration;         // Aceleração da pessoa
    float lag_compensation;     // Compensação de latência
};

struct MotionState {
    float x, y;                 // Posição atual
    float vx, vy;               // Velocidade atual
    float ax, ay;               // Aceleração atual
    float predicted_x, predicted_y;  // Posição predita
    float speed;                // Magnitude da velocidade
    bool is_accelerating;       // Flag de aceleração
    bool is_escaping;           // Flag de fuga do frame
};

// ========== CLASSE PRINCIPAL ==========

class CaptureEngine : public QObject {
    Q_OBJECT

public:
    CaptureEngine(const std::string& source, int fps, float threshold);
    ~CaptureEngine();
    
    void start();
    void stop();
    void setConfidenceThreshold(float threshold);
    void setAutoTracking(bool enabled);
    void setManualTarget(float x, float y);
    
    // Calibração
    void setCameraFOV(float horizontal_deg, float vertical_deg);
    void setSystemLatency(float latency_ms);

signals:
    void frameReady(const QImage& frame);
    void fpsUpdated(double fps);
    void detectionCount(int count);
    void ptzAdjustmentNeeded(int pan, int tilt);
    void motionStateUpdated(const QString& state);

private:
    // ========== CORE METHODS ==========
    void captureLoop();
    void processPTZControl(const cv::Mat& frame, const std::vector<Detection>& detections, float dt);
    Detection selectBestTarget(const cv::Mat& frame, const std::vector<Detection>& detections);
    QImage matToQImage(const cv::Mat& mat);
    void drawDetections(cv::Mat& frame, const std::vector<Detection>& dets);
    void resetPIDState();
    
    // ========== KALMAN ADAPTATIVO ==========
    void updateAdaptiveKalman(float nx, float ny, float dt, float confidence);
    void predictWithAcceleration(float dt, float& pred_nx, float& pred_ny);
    
    // ========== MOTION ANALYSIS ==========
    MotionState analyzeMotion(float nx, float ny, float dt);
    AdaptivePIDGains computeAdaptivePID(float target_speed, char axis);
    bool isTargetEscaping(float nx, float ny, float vx, float vy);
    
    // ========== VELOCITY MATCHING ==========
    VelocityProfile computeOptimalPTZVelocity(
        float target_nx, float target_ny,
        float target_vx, float target_vy,
        float target_ax, float target_ay,
        float frame_width, float frame_height
    );
    
    int velocityToVISCA(float angular_velocity, char axis);
    float pixelsToDegreesPerSecond(float px_per_sec, float fov_deg, float dimension);
    float smoothVelocity(float raw_velocity, std::deque<float>& history);
    
    // ========== HELPERS ==========
    float applyDeadband(float err);
    float applyNonLinearity(float raw_speed);
    bool shouldThrottleVISCA();
    
    // ========== MEMBER VARIABLES ==========
    
    // Core
    std::string videoSource;
    int targetFPS;
    float confThreshold;
    std::atomic<bool> running;
    std::atomic<bool> autoTracking;
    QThread* captureThread;
    std::unique_ptr<YOLODetector> detector;
    
    // PID State
    float integral_x, integral_y;
    float prev_err_x, prev_err_y;
    float filtered_derivative_x, filtered_derivative_y;
    float prev_ptz_speed_x, prev_ptz_speed_y;
    
    // Tracking State
    float last_nx, last_ny, last_nz;
    int lost_frames;
    bool manual_mode;
    float manual_target_x, manual_target_y;
    
    // Kalman Filter (4 states: x, y, vx, vy)
    cv::Mat kalman_state;
    cv::Mat kalman_cov;
    float kalman_q, kalman_r;
    bool kalman_initialized;
    
    // Motion History
    std::deque<float> position_history_x;
    std::deque<float> position_history_y;
    std::deque<float> velocity_history_x;
    std::deque<float> velocity_history_y;
    float prev_vx, prev_vy;  // Para calcular aceleração
    
    // Velocity Matching
    float camera_fov_horizontal;
    float camera_fov_vertical;
    float system_latency_ms;
    
    // Control Parameters
    float conf_min, nz_min, deadband;
    float Kp_x, Ki_x, Kd_x;
    float Kp_y, Ki_y, Kd_y;
    float I_max, gamma;
    float near_edge_threshold, approach_limit, v_thresh;
    float slew_rate, lpf_tau, stop_threshold;
    int lost_max_frames;
    
    // Deadzone & Throttle
    float deadzone_inner, deadzone_outer;
    std::chrono::steady_clock::time_point last_visca_time;
    int visca_throttle_ms;
    
    // Latency Components (ms)
    static constexpr float CAMERA_CAPTURE_LATENCY = 33.0f;
    static constexpr float YOLO_PROCESSING_LATENCY = 40.0f;
    static constexpr float VISCA_COMMAND_LATENCY = 50.0f;
    static constexpr float PTZ_MECHANICAL_LATENCY = 80.0f;
    
    // VISCA Speed Calibration
    static constexpr float VISCA_SPEED_1_DEG_PER_SEC = 2.0f;
    static constexpr float VISCA_SPEED_24_DEG_PER_SEC = 100.0f;
};

#endif // CAPTUREENGINE_H