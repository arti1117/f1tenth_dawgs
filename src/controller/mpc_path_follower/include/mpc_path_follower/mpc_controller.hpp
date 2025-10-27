#pragma once

#ifndef MPC_CONTROLLER_HPP
#define MPC_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>

/**
 * @brief Model Predictive Controller for path following
 *
 * Uses kinematic bicycle model with linearization around reference trajectory.
 * Solves a quadratic programming problem to find optimal control sequence.
 *
 * State: [x, y, yaw, v]
 * Control: [steering_angle, acceleration]
 */
class MPCController {
public:
    struct Config {
        // MPC parameters
        int prediction_horizon;      // N - number of prediction steps
        double dt;                    // time step [s]

        // Vehicle parameters
        double wheelbase;             // L [m]
        double max_steering_angle;    // [rad]
        double max_acceleration;      // [m/s^2]
        double max_deceleration;      // [m/s^2]
        double max_speed;             // [m/s]
        double min_speed;             // [m/s]

        // Cost function weights
        double weight_lateral_error;      // Q[1,1] - lateral position error
        double weight_heading_error;      // Q[2,2] - heading error
        double weight_velocity_error;     // Q[3,3] - velocity tracking error
        double weight_steering;           // R[0,0] - steering effort
        double weight_acceleration;       // R[1,1] - acceleration effort
        double weight_steering_rate;      // dR[0,0] - steering rate of change
        double weight_acceleration_rate;  // dR[1,1] - acceleration rate of change
    };

    struct State {
        double x;
        double y;
        double yaw;
        double v;
    };

    struct Control {
        double steering;
        double acceleration;
    };

    struct Solution {
        std::vector<State> predicted_states;
        std::vector<Control> control_sequence;
        double cost;
        bool success;
    };

    MPCController(const Config& config);

    /**
     * @brief Solve MPC optimization problem
     * @param current_state Current vehicle state
     * @param reference_path Reference trajectory points
     * @param reference_velocities Target velocities along path
     * @return MPC solution with optimal control sequence
     */
    Solution solve(const State& current_state,
                   const std::vector<State>& reference_path,
                   const std::vector<double>& reference_velocities);

    /**
     * @brief Get first control command from MPC solution
     */
    Control getControlCommand() const { return control_command_; }

private:
    Config config_;
    Control control_command_;
    Control prev_control_;  // For rate-of-change penalty

    /**
     * @brief Linearize bicycle model around reference trajectory
     * @param ref_state Reference state for linearization
     * @param ref_control Reference control for linearization
     * @return Linearized state-space matrices (A, B)
     */
    std::pair<Eigen::Matrix4d, Eigen::Matrix<double, 4, 2>>
    linearizeBicycleModel(const State& ref_state, const Control& ref_control);

    /**
     * @brief Simulate bicycle model forward
     */
    State simulateStep(const State& state, const Control& control, double dt);

    /**
     * @brief Find closest point on reference path
     */
    size_t findClosestPoint(const State& current_state,
                           const std::vector<State>& reference_path);

    /**
     * @brief Extract reference trajectory for MPC horizon
     */
    std::vector<State> extractReferenceTrajectory(
        const std::vector<State>& reference_path,
        const std::vector<double>& reference_velocities,
        size_t start_idx);

    /**
     * @brief Solve QP problem using iterative method (gradient descent)
     * For real-time performance without external QP solver dependencies
     */
    bool solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                 const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq,
                 Eigen::VectorXd& solution);

    /**
     * @brief Apply control and state constraints
     */
    Control constrainControl(const Control& control);

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle);
};

#endif // MPC_CONTROLLER_HPP
