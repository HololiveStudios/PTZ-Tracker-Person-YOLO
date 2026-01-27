// src/CaptureEngine.cpp - VERSÃO ULTRA-PRECISA v2.1
#include "CaptureEngine.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <numeric>

CaptureEngine::CaptureEngine(const std::string& source, int fps, float threshold)
    : videoSource(source), targetFPS(fps), confThreshold(threshold), 
      running(false), captureThread(nullptr), autoTracking(false),
      integral_x(0), integral_y(0), prev_err_x(0), prev_err_y(0),
      filtered_derivative_x(0), filtered_derivative_y(0),
      prev_ptz_speed_x(0), prev_ptz_speed_y(0),
      last_nx(0.5f), last_ny(0.5f), last_nz(0),
      lost_frames(0), manual_mode(false),
      manual_target_x(0.5f), manual_target_y(0.5f),
      kalman_initialized(false),
      prev_vx(0), prev_vy(0)
{
    detector = std::make_unique<YOLODetector>("yolov8n.onnx", threshold);
    
    // Control parameters
    conf_min = 0.5f;
    nz_min = 0.02f;
    deadband = 0.03f;
    
    // PID gains (inicial - serão ajustados dinamicamente)
    Kp_x = 1.5f; Ki_x = 0.08f; Kd_x = 0.12f;
    Kp_y = 1.3f; Ki_y = 0.08f; Kd_y = 0.10f;
    I_max = 0.5f;
    
    gamma = 0.75f;
    near_edge_threshold = 0.20f;
    approach_limit = 0.05f;
    v_thresh = 0.03f;
    slew_rate = 1.2f;
    lpf_tau = 0.05f;
    stop_threshold = 0.015f;
    lost_max_frames = 10;
    
    // Kalman initialization
    kalman_state = cv::Mat::zeros(4, 1, CV_32F);
    kalman_cov = cv::Mat::eye(4, 4, CV_32F) * 0.1f;
    kalman_q = 0.01f;
    kalman_r = 0.05f;
    
    // Deadzone
    deadzone_inner = 0.20f;
    deadzone_outer = 0.40f;
    
    // VISCA throttle
    visca_throttle_ms = 60;
    last_visca_time = std::chrono::steady_clock::now();
    
    // Camera FOV (ajustar para sua câmera!)
    camera_fov_horizontal = 60.0f;
    camera_fov_vertical = 40.0f;
    
    // System latency
    system_latency_ms = CAMERA_CAPTURE_LATENCY + 
                        YOLO_PROCESSING_LATENCY + 
                        VISCA_COMMAND_LATENCY + 
                        PTZ_MECHANICAL_LATENCY;
    
    // History buffers
    position_history_x.resize(10, 0.5f);
    position_history_y.resize(10, 0.5f);
    velocity_history_x.resize(5, 0.0f);
    velocity_history_y.resize(5, 0.0f);
}

CaptureEngine::~CaptureEngine() {
    stop();
}

void CaptureEngine::setCameraFOV(float horizontal_deg, float vertical_deg) {
    camera_fov_horizontal = horizontal_deg;
    camera_fov_vertical = vertical_deg;
}

void CaptureEngine::setSystemLatency(float latency_ms) {
    system_latency_ms = latency_ms;
}

void CaptureEngine::setConfidenceThreshold(float threshold) {
    confThreshold = threshold;
    if (detector) {
        detector->setConfidenceThreshold(threshold);
    }
}

void CaptureEngine::setAutoTracking(bool enabled) {
    autoTracking = enabled;
    if (enabled) {
        resetPIDState();
    }
}

void CaptureEngine::setManualTarget(float x, float y) {
    manual_mode = true;
    manual_target_x = x;
    manual_target_y = y;
    resetPIDState();
}

void CaptureEngine::resetPIDState() {
    integral_x = 0;
    integral_y = 0;
    prev_err_x = 0;
    prev_err_y = 0;
    filtered_derivative_x = 0;
    filtered_derivative_y = 0;
    prev_ptz_speed_x = 0;
    prev_ptz_speed_y = 0;
    lost_frames = 0;
    prev_vx = 0;
    prev_vy = 0;
    
    kalman_initialized = false;
    kalman_state = cv::Mat::zeros(4, 1, CV_32F);
    kalman_cov = cv::Mat::eye(4, 4, CV_32F) * 0.1f;
    
    std::fill(position_history_x.begin(), position_history_x.end(), 0.5f);
    std::fill(position_history_y.begin(), position_history_y.end(), 0.5f);
    std::fill(velocity_history_x.begin(), velocity_history_x.end(), 0.0f);
    std::fill(velocity_history_y.begin(), velocity_history_y.end(), 0.0f);
}

// ========== KALMAN ADAPTATIVO ==========

void CaptureEngine::updateAdaptiveKalman(float nx, float ny, float dt, float confidence) {
    if (!kalman_initialized) {
        kalman_state.at<float>(0) = nx;
        kalman_state.at<float>(1) = ny;
        kalman_state.at<float>(2) = 0;
        kalman_state.at<float>(3) = 0;
        kalman_initialized = true;
        return;
    }
    
    // 1. PREDICT
    cv::Mat F = (cv::Mat_<float>(4, 4) << 
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1);
    
    kalman_state = F * kalman_state;
    
    // Adaptive Q: aumenta com velocidade (modelo menos confiável em alta velocidade)
    float vx = kalman_state.at<float>(2);
    float vy = kalman_state.at<float>(3);
    float speed = std::sqrt(vx * vx + vy * vy);
    
    float adaptive_q = kalman_q * (1.0f + speed * 8.0f);
    cv::Mat Q = cv::Mat::eye(4, 4, CV_32F) * adaptive_q;
    
    kalman_cov = F * kalman_cov * F.t() + Q;
    
    // 2. UPDATE
    cv::Mat H = (cv::Mat_<float>(2, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0);
    
    cv::Mat z = (cv::Mat_<float>(2, 1) << nx, ny);
    cv::Mat y = z - H * kalman_state;
    
    // Adaptive R: diminui com alta confiança (medição mais confiável)
    float adaptive_r = kalman_r / (confidence + 0.1f);
    cv::Mat R = cv::Mat::eye(2, 2, CV_32F) * adaptive_r;
    
    cv::Mat S = H * kalman_cov * H.t() + R;
    cv::Mat K = kalman_cov * H.t() * S.inv();
    
    kalman_state = kalman_state + K * y;
    kalman_cov = (cv::Mat::eye(4, 4, CV_32F) - K * H) * kalman_cov;
}

void CaptureEngine::predictWithAcceleration(float dt, float& pred_nx, float& pred_ny) {
    if (!kalman_initialized) {
        pred_nx = last_nx;
        pred_ny = last_ny;
        return;
    }
    
    float current_x = kalman_state.at<float>(0);
    float current_y = kalman_state.at<float>(1);
    float vx = kalman_state.at<float>(2);
    float vy = kalman_state.at<float>(3);
    
    // Estimar aceleração
    float ax = (vx - prev_vx) / (dt + 1e-6f);
    float ay = (vy - prev_vy) / (dt + 1e-6f);
    
    // Predição com aceleração (s = ut + 0.5at²)
    float lookahead = dt * 4.0f;  // 4 frames à frente
    pred_nx = current_x + vx * lookahead + 0.5f * ax * lookahead * lookahead;
    pred_ny = current_y + vy * lookahead + 0.5f * ay * lookahead * lookahead;
    
    pred_nx = std::clamp(pred_nx, 0.0f, 1.0f);
    pred_ny = std::clamp(pred_ny, 0.0f, 1.0f);
}

// ========== MOTION ANALYSIS ==========

MotionState CaptureEngine::analyzeMotion(float nx, float ny, float dt) {
    MotionState state;
    
    state.x = nx;
    state.y = ny;
    
    if (kalman_initialized) {
        state.vx = kalman_state.at<float>(2);
        state.vy = kalman_state.at<float>(3);
        
        // Aceleração
        state.ax = (state.vx - prev_vx) / (dt + 1e-6f);
        state.ay = (state.vy - prev_vy) / (dt + 1e-6f);
        
        state.speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
        
        float accel_mag = std::sqrt(state.ax * state.ax + state.ay * state.ay);
        state.is_accelerating = (accel_mag > 0.5f);
        
        state.is_escaping = isTargetEscaping(nx, ny, state.vx, state.vy);
        
        // Predição
        predictWithAcceleration(dt, state.predicted_x, state.predicted_y);
        
        prev_vx = state.vx;
        prev_vy = state.vy;
    } else {
        state.vx = state.vy = 0;
        state.ax = state.ay = 0;
        state.speed = 0;
        state.is_accelerating = false;
        state.is_escaping = false;
        state.predicted_x = nx;
        state.predicted_y = ny;
    }
    
    return state;
}

AdaptivePIDGains CaptureEngine::computeAdaptivePID(float target_speed, char axis) {
    AdaptivePIDGains gains;
    
    if (target_speed < 0.02f) {  // Parado/lento
        gains.Kp = (axis == 'x') ? 1.0f : 0.9f;
        gains.Ki = 0.05f;
        gains.Kd = 0.08f;
    } 
    else if (target_speed < 0.10f) {  // Velocidade média
        gains.Kp = (axis == 'x') ? 1.5f : 1.3f;
        gains.Ki = 0.08f;
        gains.Kd = 0.12f;
    } 
    else {  // Velocidade alta
        gains.Kp = (axis == 'x') ? 2.5f : 2.2f;
        gains.Ki = 0.12f;
        gains.Kd = 0.20f;
    }
    
    return gains;
}

bool CaptureEngine::isTargetEscaping(float nx, float ny, float vx, float vy) {
    bool near_left = (nx < 0.15f) && (vx < -0.02f);
    bool near_right = (nx > 0.85f) && (vx > 0.02f);
    bool near_top = (ny < 0.15f) && (vy < -0.02f);
    bool near_bottom = (ny > 0.85f) && (vy > 0.02f);
    
    return near_left || near_right || near_top || near_bottom;
}

// ========== VELOCITY MATCHING ==========

VelocityProfile CaptureEngine::computeOptimalPTZVelocity(
    float target_nx, float target_ny,
    float target_vx, float target_vy,
    float target_ax, float target_ay,
    float frame_width, float frame_height)
{
    VelocityProfile profile;
    
    // 1. Converter velocidade para pixels/s
    float target_speed_px_x = target_vx * frame_width;
    float target_speed_px_y = target_vy * frame_height;
    
    // 2. Converter para graus/s
    float target_speed_deg_x = pixelsToDegreesPerSecond(
        target_speed_px_x, camera_fov_horizontal, frame_width
    );
    float target_speed_deg_y = pixelsToDegreesPerSecond(
        target_speed_px_y, camera_fov_vertical, frame_height
    );
    
    profile.target_velocity = std::sqrt(
        target_speed_deg_x * target_speed_deg_x + 
        target_speed_deg_y * target_speed_deg_y
    );
    
    // 3. Compensar latência
    float latency_sec = system_latency_ms / 1000.0f;
    
    float pred_displacement_x = target_vx * latency_sec + 
                                0.5f * target_ax * latency_sec * latency_sec;
    float pred_displacement_y = target_vy * latency_sec + 
                                0.5f * target_ay * latency_sec * latency_sec;
    
    float compensation_deg_x = pred_displacement_x * camera_fov_horizontal;
    float compensation_deg_y = pred_displacement_y * camera_fov_vertical;
    
    profile.lag_compensation = std::sqrt(
        compensation_deg_x * compensation_deg_x + 
        compensation_deg_y * compensation_deg_y
    );
    
    // 4. Velocidade necessária
    profile.required_velocity = profile.target_velocity + 
                                (profile.lag_compensation / (latency_sec + 1e-6f));
    
    // 5. Smooth velocity
    profile.camera_velocity = smoothVelocity(
        profile.required_velocity, 
        velocity_history_x
    );
    
    // 6. Boost se acelerando muito
    float accel_mag = std::sqrt(target_ax * target_ax + target_ay * target_ay);
    if (accel_mag > 0.5f) {
        profile.camera_velocity *= 1.4f;
    }
    
    profile.acceleration = accel_mag;
    
    return profile;
}

int CaptureEngine::velocityToVISCA(float angular_velocity, char axis) {
    float abs_velocity = std::abs(angular_velocity);
    
    // Interpolação linear
    float speed_range = VISCA_SPEED_24_DEG_PER_SEC - VISCA_SPEED_1_DEG_PER_SEC;
    float normalized = (abs_velocity - VISCA_SPEED_1_DEG_PER_SEC) / speed_range;
    
    int visca_speed = 1 + static_cast<int>(normalized * 23.0f);
    visca_speed = std::clamp(visca_speed, 1, 24);
    
    // Tilt geralmente mais lento
    if (axis == 'y') {
        visca_speed = static_cast<int>(visca_speed * 0.85f);
        visca_speed = std::max(1, visca_speed);
    }
    
    return visca_speed;
}

float CaptureEngine::pixelsToDegreesPerSecond(
    float px_per_sec, float fov_deg, float dimension) 
{
    return (px_per_sec * fov_deg) / dimension;
}

float CaptureEngine::smoothVelocity(float raw_velocity, std::deque<float>& history) {
    history.pop_front();
    history.push_back(raw_velocity);
    
    float smoothed = 0;
    float weight_sum = 0;
    
    for (size_t i = 0; i < history.size(); i++) {
        float weight = static_cast<float>(i + 1);
        smoothed += history[i] * weight;
        weight_sum += weight;
    }
    
    return smoothed / weight_sum;
}

// ========== MAIN CONTROL LOOP ==========

void CaptureEngine::start() {
    if (running) return;
    running = true;
    
    captureThread = QThread::create([this]() { captureLoop(); });
    captureThread->start();
}

void CaptureEngine::stop() {
    running = false;
    if (captureThread) {
        captureThread->wait();
        delete captureThread;
        captureThread = nullptr;
    }
}

void CaptureEngine::captureLoop() {
    cv::VideoCapture cap;
    
    try {
        int deviceId = std::stoi(videoSource);
        cap.open(deviceId);
    } catch (...) {
        cap.open(videoSource);
    }
    
    if (!cap.isOpened()) {
        return;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    double frameTime = 1000.0 / targetFPS;
    auto lastFpsTime = std::chrono::steady_clock::now();
    auto lastFrameTime = std::chrono::steady_clock::now();
    int frameCounter = 0;
    
    while (running) {
        auto startTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastFrameTime).count();
        lastFrameTime = now;
        
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        auto detections = detector->detect(frame);
        drawDetections(frame, detections);
        
        // PTZ Control ultra-preciso
        if (autoTracking || manual_mode) {
            processPTZControl(frame, detections, dt);
        }
        
        emit frameReady(matToQImage(frame));
        emit detectionCount(static_cast<int>(detections.size()));
        
        frameCounter++;
        auto elapsed = std::chrono::duration<double>(now - lastFpsTime).count();
        
        if (elapsed >= 1.0) {
            double fps = frameCounter / elapsed;
            emit fpsUpdated(fps);
            frameCounter = 0;
            lastFpsTime = now;
        }
        
        auto processingTime = std::chrono::steady_clock::now() - startTime;
        auto waitTime = std::chrono::milliseconds(static_cast<int>(frameTime)) - 
                       std::chrono::duration_cast<std::chrono::milliseconds>(processingTime);
        
        if (waitTime.count() > 0) {
            std::this_thread::sleep_for(waitTime);
        }
    }
    
    cap.release();
}

void CaptureEngine::processPTZControl(
    const cv::Mat& frame, 
    const std::vector<Detection>& detections, 
    float dt) 
{
    float nx, ny, nz;
    bool target_found = false;
    float detection_confidence = 0.0f;
    
    if (manual_mode) {
        nx = manual_target_x;
        ny = manual_target_y;
        nz = 0.1f;
        target_found = true;
        detection_confidence = 1.0f;
    } else if (!detections.empty()) {
        Detection bestTarget = selectBestTarget(frame, detections);
        
        if (bestTarget.confidence >= conf_min) {
            float cx = bestTarget.bbox.x + bestTarget.bbox.width / 2.0f;
            float cy = bestTarget.bbox.y + bestTarget.bbox.height / 2.0f;
            float diag = std::sqrt(static_cast<float>(frame.cols * frame.cols + frame.rows * frame.rows));
            
            nx = cx / frame.cols;
            ny = cy / frame.rows;
            nz = std::sqrt(bestTarget.bbox.width * bestTarget.bbox.height) / diag;
            
            if (nz >= nz_min) {
                detection_confidence = bestTarget.confidence;
                updateAdaptiveKalman(nx, ny, dt, detection_confidence);
                target_found = true;
                last_nx = nx;
                last_ny = ny;
                last_nz = nz;
                lost_frames = 0;
            }
        }
    }
    
    if (!target_found) {
        lost_frames++;
        
        if (lost_frames < lost_max_frames) {
            integral_x *= 0.90f;
            integral_y *= 0.90f;
            
            float pred_nx, pred_ny;
            predictWithAcceleration(dt, pred_nx, pred_ny);
            nx = pred_nx;
            ny = pred_ny;
            target_found = true;  // Usar predição
        } else {
            if (autoTracking) {
                emit ptzAdjustmentNeeded(0, 0);
            }
            return;
        }
    }
    
    // ========== MOTION ANALYSIS ==========
    MotionState motion = analyzeMotion(nx, ny, dt);
    
    // Emit estado para UI
    QString state_msg = QString("Vel: %1 | Acel: %2 | Esc: %3")
        .arg(motion.speed, 0, 'f', 3)
        .arg(motion.is_accelerating ? "SIM" : "NAO")
        .arg(motion.is_escaping ? "SIM" : "NAO");
    emit motionStateUpdated(state_msg);
    
    // Usar predição se velocidade > limiar
    if (motion.speed > 0.03f && !manual_mode) {
        nx = 0.6f * nx + 0.4f * motion.predicted_x;
        ny = 0.6f * ny + 0.4f * motion.predicted_y;
    }
    
    // ========== ADAPTIVE PID ==========
    auto pid_x = computeAdaptivePID(motion.speed, 'x');
    auto pid_y = computeAdaptivePID(motion.speed, 'y');
    
    float err_x = nx - 0.5f;
    float err_y = ny - 0.5f;
    float err_mag = std::sqrt(err_x * err_x + err_y * err_y);
    
    // Deadzone central
    if (err_mag < deadzone_inner && !manual_mode) {
        emit ptzAdjustmentNeeded(0, 0);
        return;
    }
    
    float err_x_eff = applyDeadband(err_x);
    float err_y_eff = applyDeadband(err_y);
    
    // PID para X
    float u_p_x = pid_x.Kp * err_x_eff;
    integral_x += pid_x.Ki * err_x_eff * dt;
    integral_x = std::clamp(integral_x, -I_max, I_max);
    
    float derivative_x = (err_x_eff - prev_err_x) / (dt + 1e-6f);
    float alpha_d = std::exp(-dt / lpf_tau);
    filtered_derivative_x = alpha_d * filtered_derivative_x + (1 - alpha_d) * derivative_x;
    float u_d_x = pid_x.Kd * filtered_derivative_x;
    
    // PID para Y
    float u_p_y = pid_y.Kp * err_y_eff;
    integral_y += pid_y.Ki * err_y_eff * dt;
    integral_y = std::clamp(integral_y, -I_max, I_max);
    
    float derivative_y = (err_y_eff - prev_err_y) / (dt + 1e-6f);
    filtered_derivative_y = alpha_d * filtered_derivative_y + (1 - alpha_d) * derivative_y;
    float u_d_y = pid_y.Kd * filtered_derivative_y;
    
    // ========== VELOCITY MATCHING ==========
    VelocityProfile profile = computeOptimalPTZVelocity(
        nx, ny,
        motion.vx, motion.vy,
        motion.ax, motion.ay,
        static_cast<float>(frame.cols), static_cast<float>(frame.rows)
    );
    
    // Converter para VISCA
    int pan_cmd = velocityToVISCA(profile.camera_velocity, 'x');
    int tilt_cmd = velocityToVISCA(profile.camera_velocity, 'y');
    
    // Boost de emergência se escapando
    if (motion.is_escaping) {
        pan_cmd = 24;
        tilt_cmd = 20;
    }
    
    // Aplicar direção
    pan_cmd *= (err_x_eff > 0) ? 1 : -1;
    tilt_cmd *= (err_y_eff > 0) ? -1 : 1;
    
    pan_cmd = std::clamp(pan_cmd, -24, 24);
    tilt_cmd = std::clamp(tilt_cmd, -20, 20);
    
    // VISCA throttle
    if ((std::abs(err_x_eff) > 0.01f || std::abs(err_y_eff) > 0.01f) && 
        !shouldThrottleVISCA()) 
    {
        emit ptzAdjustmentNeeded(pan_cmd, tilt_cmd);
        last_visca_time = std::chrono::steady_clock::now();
    }
    
    // Update state
    prev_err_x = err_x_eff;
    prev_err_y = err_y_eff;
}

bool CaptureEngine::shouldThrottleVISCA() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_visca_time
    ).count();
    return elapsed < visca_throttle_ms;
}

// ========== HELPER FUNCTIONS ==========

Detection CaptureEngine::selectBestTarget(
    const cv::Mat& frame, 
    const std::vector<Detection>& detections) 
{
    Detection best = detections[0];
    float bestScore = 0;
    
    cv::Point2f frameCenter(frame.cols / 2.0f, frame.rows / 2.0f);
    
    for (const auto& det : detections) {
        cv::Point2f center(det.bbox.x + det.bbox.width / 2.0f,
                          det.bbox.y + det.bbox.height / 2.0f);
        
        float area = det.bbox.width * det.bbox.height;
        float distToCenter = std::sqrt(
            std::pow(center.x - frameCenter.x, 2) + 
            std::pow(center.y - frameCenter.y, 2)
        );
        
        float score = (area / (distToCenter + 100.0f)) * det.confidence;
        
        if (score > bestScore) {
            bestScore = score;
            best = det;
        }
    }
    
    return best;
}

float CaptureEngine::applyDeadband(float err) {
    if (std::abs(err) < deadband) {
        return 0.0f;
    }
    return std::copysign(std::abs(err) - deadband, err);
}

float CaptureEngine::applyNonLinearity(float raw_speed) {
    float abs_speed = std::abs(raw_speed);
    float a = 1.5f;
    float speed_factor = a * std::pow(abs_speed, gamma);
    return std::clamp(speed_factor, 0.0f, 1.0f);
}

void CaptureEngine::drawDetections(cv::Mat& frame, const std::vector<Detection>& dets) {
    for (const auto& det : dets) {
        cv::rectangle(frame, det.bbox, cv::Scalar(0, 255, 0), 2);
        
        std::string label = det.label + " " + 
                           std::to_string(static_cast<int>(det.confidence * 100)) + "%";
        
        int baseline;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                            0.6, 2, &baseline);
        
        cv::rectangle(frame, 
            cv::Point(det.bbox.x, det.bbox.y - textSize.height - 5),
            cv::Point(det.bbox.x + textSize.width, det.bbox.y),
            cv::Scalar(0, 255, 0), -1);
        
        cv::putText(frame, label, 
            cv::Point(det.bbox.x, det.bbox.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    }
    
    // Desenhar crosshair central
    int cx = frame.cols / 2;
    int cy = frame.rows / 2;
    cv::line(frame, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy), cv::Scalar(0, 0, 255), 2);
    cv::line(frame, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20), cv::Scalar(0, 0, 255), 2);
}

QImage CaptureEngine::matToQImage(const cv::Mat& mat) {
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), 
                  QImage::Format_RGB888).copy();
}