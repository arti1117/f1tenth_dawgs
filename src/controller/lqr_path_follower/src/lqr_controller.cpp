#include "lqr_path_follower/lqr_controller.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

LQRController::LQRController(const Config& config)
    : config_(config),
      prev_control_{0.0, 0.0} {
}

LQRController::Control LQRController::computeControl(
    const State& current_state,
    const std::vector<State>& reference_path,
    const std::vector<double>& reference_velocities) {

    if (reference_path.empty()) {
        return {0.0, 0.0};
    }

    // Find closest point on reference path
    size_t closest_idx = findClosestPoint(current_state, reference_path);

    // Find lookahead reference point
    State ref_state = findLookaheadPoint(current_state, reference_path,
                                        reference_velocities, closest_idx);

    // Compute error state [lateral_error, heading_error, velocity_error]
    Eigen::Vector3d error = computeErrorState(current_state, ref_state);

    // Linearize bicycle model around reference
    double ref_steering = 0.0;  // Assume reference follows path with zero steering
    auto [A, B] = linearizeBicycleModel(ref_state, ref_steering);

    // Setup LQR cost matrices
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = config_.Q_lateral;   // Lateral error
    Q(1, 1) = config_.Q_heading;   // Heading error
    Q(2, 2) = config_.Q_velocity;  // Velocity error

    double R = config_.R_steering;  // Steering effort

    // Solve DARE to get optimal gain K
    Eigen::RowVector3d K = solveDARE(A, B, Q, R);

    // Compute optimal steering: u = -K * error
    double steering = -K.dot(error);

    // Add feedforward term for path curvature (if available)
    // For simplicity, we use pure feedback here

    // Add steering rate penalty (smooth control)
    double steering_rate = (steering - prev_control_.steering) / config_.dt;
    if (std::abs(steering_rate) > 5.0) {  // Limit to 5 rad/s
        steering = prev_control_.steering + 5.0 * config_.dt * (steering_rate > 0 ? 1 : -1);
    }

    // Compute acceleration for speed tracking (simple P controller)
    double acceleration = computeSpeedControl(current_state.v, ref_state.v);

    Control control{steering, acceleration};
    control = constrainControl(control);

    // Store for next iteration
    prev_control_ = control;

    // Simulate prediction for visualization
    simulatePrediction(current_state, control);

    return control;
}

size_t LQRController::findClosestPoint(
    const State& current_state,
    const std::vector<State>& reference_path) {

    double min_dist = std::numeric_limits<double>::infinity();
    size_t closest_idx = 0;

    for (size_t i = 0; i < reference_path.size(); ++i) {
        double dx = reference_path[i].x - current_state.x;
        double dy = reference_path[i].y - current_state.y;
        double dist = std::hypot(dx, dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}

LQRController::State LQRController::findLookaheadPoint(
    const State& current_state,
    const std::vector<State>& reference_path,
    const std::vector<double>& reference_velocities,
    size_t start_idx) {

    // Adaptive lookahead based on speed
    double lookahead_dist = config_.lookahead_time * current_state.v;
    lookahead_dist = std::max(config_.min_lookahead_dist,
                             std::min(config_.max_lookahead_dist, lookahead_dist));

    // Walk along path to find lookahead point
    double accumulated_dist = 0.0;
    size_t idx = start_idx;

    while (idx < reference_path.size() - 1 && accumulated_dist < lookahead_dist) {
        double dx = reference_path[idx + 1].x - reference_path[idx].x;
        double dy = reference_path[idx + 1].y - reference_path[idx].y;
        accumulated_dist += std::hypot(dx, dy);
        idx++;
    }

    State ref_state = reference_path[std::min(idx, reference_path.size() - 1)];

    // Set reference velocity
    if (idx < reference_velocities.size()) {
        ref_state.v = reference_velocities[idx];
    }

    return ref_state;
}

Eigen::Vector3d LQRController::computeErrorState(
    const State& current, const State& reference) {

    // Rotate error to vehicle frame
    double dx = reference.x - current.x;
    double dy = reference.y - current.y;

    // Lateral error (perpendicular to vehicle heading)
    double lateral_error = -dx * std::sin(current.yaw) + dy * std::cos(current.yaw);

    // Heading error
    double heading_error = normalizeAngle(reference.yaw - current.yaw);

    // Velocity error
    double velocity_error = reference.v - current.v;

    return Eigen::Vector3d(lateral_error, heading_error, velocity_error);
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d>
LQRController::linearizeBicycleModel(const State& ref_state, double ref_steering) {

    // Linearized kinematic bicycle model around reference
    // State: [lateral_error, heading_error, velocity_error]
    // Control: steering_angle

    double v = ref_state.v;
    double L = config_.wheelbase;
    double dt = config_.dt;

    // Discrete-time state matrix A
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 1) = v * dt;  // lateral_error += heading_error * v * dt
    A(1, 2) = std::tan(ref_steering) / L * dt;  // heading_error changes with velocity

    // Discrete-time input matrix B
    Eigen::Vector3d B;
    B(0) = 0.0;
    B(1) = v / L * dt;  // heading changes with steering
    B(2) = 0.0;

    return {A, B};
}

Eigen::RowVector3d LQRController::solveDARE(
    const Eigen::Matrix3d& A,
    const Eigen::Vector3d& B,
    const Eigen::Matrix3d& Q,
    double R) {

    // Solve discrete algebraic Riccati equation iteratively
    // P = A'PA - A'PB(R + B'PB)^-1 B'PA + Q

    Eigen::Matrix3d P = Q;  // Initialize with Q
    const int max_iterations = 50;
    const double tolerance = 1e-4;

    for (int i = 0; i < max_iterations; ++i) {
        // Compute gain
        double BPB = B.transpose() * P * B;
        double denominator = R + BPB;

        if (std::abs(denominator) < 1e-10) {
            break;  // Avoid division by zero
        }

        Eigen::Vector3d K_temp = (1.0 / denominator) * P * B;

        // Update P
        Eigen::Matrix3d P_new = A.transpose() * P * A -
                                A.transpose() * P * B * K_temp.transpose() * A + Q;

        // Check convergence
        double change = (P_new - P).norm();
        P = P_new;

        if (change < tolerance) {
            break;
        }
    }

    // Compute optimal gain K
    double BPB = B.transpose() * P * B;
    double denominator = R + BPB;

    Eigen::RowVector3d K;
    if (std::abs(denominator) > 1e-10) {
        K = (1.0 / denominator) * B.transpose() * P * A;
    } else {
        K.setZero();
    }

    return K;
}

double LQRController::computeSpeedControl(double current_speed, double target_speed) {
    // Simple P controller for speed
    const double Kp = 2.0;
    double speed_error = target_speed - current_speed;
    double acceleration = Kp * speed_error;

    return acceleration;
}

LQRController::Control LQRController::constrainControl(const Control& control) {
    Control constrained;

    // Steering angle constraints
    constrained.steering = std::max(-config_.max_steering_angle,
                                   std::min(config_.max_steering_angle,
                                          control.steering));

    // Acceleration constraints (simple limits)
    constrained.acceleration = std::max(-6.0, std::min(4.0, control.acceleration));

    return constrained;
}

double LQRController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void LQRController::simulatePrediction(const State& start_state, const Control& control) {
    // Simple prediction for visualization (5 steps ahead)
    predicted_trajectory_.clear();
    predicted_trajectory_.push_back(start_state);

    State state = start_state;
    for (int i = 0; i < 5; ++i) {
        double v = state.v;
        double yaw = state.yaw;
        double delta = control.steering;
        double a = control.acceleration;
        double L = config_.wheelbase;
        double dt = config_.dt;

        // Simulate forward
        state.x = state.x + v * std::cos(yaw) * dt;
        state.y = state.y + v * std::sin(yaw) * dt;
        state.yaw = normalizeAngle(state.yaw + v * std::tan(delta) / L * dt);
        state.v = std::max(config_.min_speed, std::min(config_.max_speed, state.v + a * dt));

        predicted_trajectory_.push_back(state);
    }
}
