#include "mpc_path_follower/mpc_controller.hpp"
#include <iostream>
#include <limits>
#include <algorithm>

MPCController::MPCController(const Config& config)
    : config_(config),
      prev_control_{0.0, 0.0} {
    control_command_ = {0.0, 0.0};
}

MPCController::Solution MPCController::solve(
    const State& current_state,
    const std::vector<State>& reference_path,
    const std::vector<double>& reference_velocities) {

    Solution solution;
    solution.success = false;
    solution.cost = 0.0;

    if (reference_path.empty()) {
        return solution;
    }

    // Find closest point on reference path
    size_t closest_idx = findClosestPoint(current_state, reference_path);

    // Extract reference trajectory for prediction horizon
    std::vector<State> ref_traj = extractReferenceTrajectory(
        reference_path, reference_velocities, closest_idx);

    if (ref_traj.size() < static_cast<size_t>(config_.prediction_horizon)) {
        return solution;
    }

    // Initialize control sequence with zero (warm start can be added later)
    int N = config_.prediction_horizon;
    Eigen::VectorXd u(2 * N);  // [steering_0, accel_0, steering_1, accel_1, ...]
    u.setZero();

    // Iterative solution using Sequential Quadratic Programming (SQP)
    const int max_iterations = 10;
    const double tolerance = 1e-3;

    State state = current_state;
    std::vector<State> predicted_states;
    predicted_states.push_back(state);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Reset for this iteration
        predicted_states.clear();
        predicted_states.push_back(current_state);
        state = current_state;

        // Build QP matrices incrementally
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2 * N, 2 * N);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(2 * N);

        double cost = 0.0;

        // Simulate forward and accumulate cost
        for (int k = 0; k < N; ++k) {
            Control ctrl;
            ctrl.steering = u(2 * k);
            ctrl.acceleration = u(2 * k + 1);

            // Apply constraints
            ctrl = constrainControl(ctrl);

            // Simulate one step
            State next_state = simulateStep(state, ctrl, config_.dt);
            predicted_states.push_back(next_state);

            // Compute errors relative to reference
            double lateral_error = next_state.y - ref_traj[k].y;
            double heading_error = normalizeAngle(next_state.yaw - ref_traj[k].yaw);
            double velocity_error = next_state.v - ref_traj[k].v;

            // Stage cost
            double stage_cost =
                config_.weight_lateral_error * lateral_error * lateral_error +
                config_.weight_heading_error * heading_error * heading_error +
                config_.weight_velocity_error * velocity_error * velocity_error +
                config_.weight_steering * ctrl.steering * ctrl.steering +
                config_.weight_acceleration * ctrl.acceleration * ctrl.acceleration;

            // Control rate cost (penalize rapid changes)
            if (k == 0) {
                double d_steering = ctrl.steering - prev_control_.steering;
                double d_accel = ctrl.acceleration - prev_control_.acceleration;
                stage_cost += config_.weight_steering_rate * d_steering * d_steering +
                             config_.weight_acceleration_rate * d_accel * d_accel;
            } else {
                double d_steering = ctrl.steering - u(2 * (k - 1));
                double d_accel = ctrl.acceleration - u(2 * (k - 1) + 1);
                stage_cost += config_.weight_steering_rate * d_steering * d_steering +
                             config_.weight_acceleration_rate * d_accel * d_accel;
            }

            cost += stage_cost;

            // Linearize around current trajectory point
            auto [A, B] = linearizeBicycleModel(state, ctrl);

            // Build Hessian and gradient for this stage
            // Simplified: H_k = B^T * Q * B + R
            // g_k = B^T * Q * (x_k - x_ref)

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0, 0) = config_.weight_lateral_error * 0.1;  // x position (less weight)
            Q(1, 1) = config_.weight_lateral_error;  // y position (lateral)
            Q(2, 2) = config_.weight_heading_error;  // yaw
            Q(3, 3) = config_.weight_velocity_error; // velocity

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0, 0) = config_.weight_steering;
            R(1, 1) = config_.weight_acceleration;

            // Approximate Hessian block
            H.block<2, 2>(2 * k, 2 * k) += B.transpose() * Q * B + R;

            // Gradient block
            Eigen::Vector4d state_error;
            state_error << next_state.x - ref_traj[k].x,
                          next_state.y - ref_traj[k].y,
                          normalizeAngle(next_state.yaw - ref_traj[k].yaw),
                          next_state.v - ref_traj[k].v;

            g.segment<2>(2 * k) += B.transpose() * Q * state_error;

            state = next_state;
        }

        // Gradient descent update
        double step_size = 0.1;
        Eigen::VectorXd u_new = u - step_size * (H * u + g);

        // Apply constraints to new control sequence
        for (int k = 0; k < N; ++k) {
            Control ctrl;
            ctrl.steering = u_new(2 * k);
            ctrl.acceleration = u_new(2 * k + 1);
            ctrl = constrainControl(ctrl);
            u_new(2 * k) = ctrl.steering;
            u_new(2 * k + 1) = ctrl.acceleration;
        }

        // Check convergence
        double delta = (u_new - u).norm();
        u = u_new;

        if (delta < tolerance) {
            break;
        }
    }

    // Extract optimal control sequence
    solution.control_sequence.clear();
    for (int k = 0; k < N; ++k) {
        Control ctrl;
        ctrl.steering = u(2 * k);
        ctrl.acceleration = u(2 * k + 1);
        solution.control_sequence.push_back(ctrl);
    }

    // Store first control command
    if (!solution.control_sequence.empty()) {
        control_command_ = solution.control_sequence[0];
        prev_control_ = control_command_;
    }

    solution.predicted_states = predicted_states;
    solution.success = true;

    return solution;
}

std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 4, 2>>
MPCController::linearizeBicycleModel(const State& ref_state, const Control& ref_control) {
    // Kinematic bicycle model:
    // x_dot = v * cos(yaw)
    // y_dot = v * sin(yaw)
    // yaw_dot = v * tan(steering) / L
    // v_dot = acceleration

    double v = ref_state.v;
    double yaw = ref_state.yaw;
    double delta = ref_control.steering;
    double L = config_.wheelbase;

    // State matrix A (Jacobian w.r.t. state)
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

    // Discretize using forward Euler
    double dt = config_.dt;

    // dx/dyaw
    A(0, 2) = -v * std::sin(yaw) * dt;
    // dy/dyaw
    A(1, 2) = v * std::cos(yaw) * dt;
    // dx/dv
    A(0, 3) = std::cos(yaw) * dt;
    // dy/dv
    A(1, 3) = std::sin(yaw) * dt;
    // dyaw/dv
    A(2, 3) = std::tan(delta) / L * dt;

    // Input matrix B (Jacobian w.r.t. control)
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();

    // dyaw/dsteering
    B(2, 0) = v / (L * std::cos(delta) * std::cos(delta)) * dt;
    // dv/dacceleration
    B(3, 1) = dt;

    return {A, B};
}

MPCController::State MPCController::simulateStep(
    const State& state, const Control& control, double dt) {

    State next_state;

    double v = state.v;
    double yaw = state.yaw;
    double delta = control.steering;
    double a = control.acceleration;
    double L = config_.wheelbase;

    // Kinematic bicycle model
    next_state.x = state.x + v * std::cos(yaw) * dt;
    next_state.y = state.y + v * std::sin(yaw) * dt;
    next_state.yaw = state.yaw + v * std::tan(delta) / L * dt;
    next_state.v = state.v + a * dt;

    // Normalize yaw
    next_state.yaw = normalizeAngle(next_state.yaw);

    // Constrain velocity
    next_state.v = std::max(config_.min_speed,
                           std::min(config_.max_speed, next_state.v));

    return next_state;
}

size_t MPCController::findClosestPoint(
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

std::vector<MPCController::State> MPCController::extractReferenceTrajectory(
    const std::vector<State>& reference_path,
    const std::vector<double>& reference_velocities,
    size_t start_idx) {

    std::vector<State> ref_traj;

    int N = config_.prediction_horizon;

    for (int k = 0; k < N; ++k) {
        size_t idx = std::min(start_idx + k, reference_path.size() - 1);
        State ref_state = reference_path[idx];

        // Update velocity from reference
        if (idx < reference_velocities.size()) {
            ref_state.v = reference_velocities[idx];
        }

        ref_traj.push_back(ref_state);
    }

    return ref_traj;
}

MPCController::Control MPCController::constrainControl(const Control& control) {
    Control constrained;

    // Steering angle constraints
    constrained.steering = std::max(-config_.max_steering_angle,
                                   std::min(config_.max_steering_angle,
                                          control.steering));

    // Acceleration constraints
    constrained.acceleration = std::max(-config_.max_deceleration,
                                       std::min(config_.max_acceleration,
                                              control.acceleration));

    return constrained;
}

double MPCController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool MPCController::solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                            const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq,
                            Eigen::VectorXd& solution) {
    // Simple gradient descent QP solver for real-time performance
    // For production, consider using OSQP or qpOASES

    const int max_iter = 100;
    const double tolerance = 1e-4;
    const double step_size = 0.01;

    solution.setZero();

    for (int iter = 0; iter < max_iter; ++iter) {
        // Gradient
        Eigen::VectorXd grad = H * solution + g;

        // Gradient descent step
        Eigen::VectorXd solution_new = solution - step_size * grad;

        // Check convergence
        if ((solution_new - solution).norm() < tolerance) {
            solution = solution_new;
            return true;
        }

        solution = solution_new;
    }

    return true;  // Return best solution found
}
