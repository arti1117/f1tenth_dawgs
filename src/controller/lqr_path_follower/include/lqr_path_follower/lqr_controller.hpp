#pragma once

#ifndef LQR_CONTROLLER_HPP
#define LQR_CONTROLLER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>

/**
 * @brief Linear Quadratic Regulator (LQR) for path following
 *
 * Uses error dynamics and discrete-time algebraic Riccati equation (DARE)
 * to compute optimal feedback gains for path tracking.
 *
 * Much faster than MPC - only requires matrix multiplication at runtime!
 */
class LQRController {
public:
    struct Config {
        // Vehicle parameters
        double wheelbase;             // L [m]
        double dt;                    // time step [s]
        double max_steering_angle;    // [rad]
        double max_speed;             // [m/s]
        double min_speed;             // [m/s]

        // LQR cost weights
        double Q_lateral;             // lateral error weight
        double Q_heading;             // heading error weight
        double Q_velocity;            // velocity error weight
        double R_steering;            // steering control weight
        double R_steering_rate;       // steering rate weight

        // Lookahead for reference point
        double lookahead_time;        // [s] time to look ahead on path
        double min_lookahead_dist;    // [m] minimum lookahead distance
        double max_lookahead_dist;    // [m] maximum lookahead distance
    };

    struct State {
        double x;
        double y;
        double yaw;
        double v;
    };

    struct Control {
        double steering;
        double acceleration;  // For speed tracking
    };

    LQRController(const Config& config);

    /**
     * @brief Compute optimal control using LQR
     * @param current_state Current vehicle state
     * @param reference_path Reference trajectory
     * @param reference_velocities Target velocities
     * @return Optimal control (steering angle)
     */
    Control computeControl(const State& current_state,
                          const std::vector<State>& reference_path,
                          const std::vector<double>& reference_velocities);

    /**
     * @brief Get predicted trajectory (for visualization)
     */
    std::vector<State> getPredictedTrajectory() const { return predicted_trajectory_; }

private:
    Config config_;
    Control prev_control_;
    std::vector<State> predicted_trajectory_;

    /**
     * @brief Find closest point on path
     */
    size_t findClosestPoint(const State& current_state,
                           const std::vector<State>& reference_path);

    /**
     * @brief Find lookahead reference point
     */
    State findLookaheadPoint(const State& current_state,
                            const std::vector<State>& reference_path,
                            const std::vector<double>& reference_velocities,
                            size_t start_idx);

    /**
     * @brief Compute error state relative to reference
     */
    Eigen::Vector3d computeErrorState(const State& current, const State& reference);

    /**
     * @brief Linearize bicycle model around reference point
     * Returns (A, B) matrices for dx = A*x + B*u
     */
    std::pair<Eigen::Matrix3d, Eigen::Vector3d>
    linearizeBicycleModel(const State& ref_state, double ref_steering);

    /**
     * @brief Solve discrete-time algebraic Riccati equation (DARE)
     * Returns LQR gain K such that u = -K*x minimizes cost
     */
    Eigen::RowVector3d solveDARE(const Eigen::Matrix3d& A,
                                  const Eigen::Vector3d& B,
                                  const Eigen::Matrix3d& Q,
                                  double R);

    /**
     * @brief Simple speed controller (P controller)
     */
    double computeSpeedControl(double current_speed, double target_speed);

    /**
     * @brief Constrain control inputs
     */
    Control constrainControl(const Control& control);

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle);

    /**
     * @brief Simulate forward for visualization
     */
    void simulatePrediction(const State& start_state, const Control& control);
};

#endif // LQR_CONTROLLER_HPP
