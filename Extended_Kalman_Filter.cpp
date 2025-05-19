#include <iostream>
#include <Eigen/Dense>  // Required for matrix operations

class ExtendedKalmanFilter {
public:
    // Constructor: Initialize state and covariance
    ExtendedKalmanFilter() {
        // State: [x, y, theta] (initial pose)
        state = Eigen::Vector3d::Zero();  // Start at (0, 0, 0)

        // Covariance matrix (initial uncertainty)
        covariance = Eigen::Matrix3d::Identity() * 0.1;  // Tune this!
    }

    // Prediction step (uses IMU data: velocity 'v' and angular rate 'omega')
    void predict(double v, double omega, double dt) {
        // Get current heading
        double theta = state(2);

        // State transition matrix (A_t)
        Eigen::Matrix3d A_t;
        A_t << 1, 0, -v * sin(theta) * dt,
               0, 1,  v * cos(theta) * dt,
               0, 0,  1;

        // Control matrix (B_t)
        Eigen::Matrix<double, 3, 2> B_t;
        B_t << cos(theta) * dt, 0,
               sin(theta) * dt, 0,
               0,               dt;

        // Control input (u_t = [v, omega])
        Eigen::Vector2d u_t(v, omega);

        // Predict state
        state = A_t * state + B_t * u_t;

        // Process noise (Q_t) - Tune based on IMU noise!
        Eigen::Matrix3d Q_t = Eigen::Matrix3d::Identity() * 0.01;

        // Predict covariance
        covariance = A_t * covariance * A_t.transpose() + Q_t;
    }

    // Update step (uses LiDAR data: [x, y, theta])
    void update(const Eigen::Vector3d& z_t) {
        // Measurement matrix (C_t) - LiDAR measures state directly
        Eigen::Matrix3d C_t = Eigen::Matrix3d::Identity();

        // Measurement noise (R_t) - Tune based on LiDAR noise!
        Eigen::Matrix3d R_t = Eigen::Matrix3d::Identity() * 0.1;

        // Kalman Gain (K_t)
        Eigen::Matrix3d K_t = covariance * C_t.transpose() * 
                              (C_t * covariance * C_t.transpose() + R_t).inverse();

        // Update state
        state = state + K_t * (z_t - C_t * state);

        // Update covariance
        covariance = (Eigen::Matrix3d::Identity() - K_t * C_t) * covariance;
    }

    // Get current state estimate
    Eigen::Vector3d getState() const { return state; }

private:
    Eigen::Vector3d state;      // [x, y, theta]
    Eigen::Matrix3d covariance; // 3x3 covariance matrix
};

// Example usage
int main() {
    ExtendedKalmanFilter ekf;

    // Simulate IMU and LiDAR data (replace with real sensor inputs!)
    double dt = 0.1;  // Time step (100ms)

    // Loop for 10 steps
    for (int i = 0; i < 10; ++i) {
        // --- PREDICT STEP (Use IMU data here) ---
        double v = 1.0;          // Linear velocity (m/s) - Replace with IMU/encoder data!
        double omega = 0.1;      // Angular velocity (rad/s) - Replace with IMU gyro_z!
        ekf.predict(v, omega, dt);

        // --- UPDATE STEP (Use LiDAR data here) ---
        if (i % 2 == 0) {  // Assume LiDAR updates every 2 steps
            double lidar_x = 0.5 * i;      // Replace with LiDAR scan-matching result!
            double lidar_y = 0.3 * i;      // (e.g., from ICP or landmarks)
            double lidar_theta = 0.05 * i;
            Eigen::Vector3d z_t(lidar_x, lidar_y, lidar_theta);
            ekf.update(z_t);
        }

        // Print current state
        Eigen::Vector3d state = ekf.getState();
        std::cout << "Step " << i << ": x=" << state(0) 
                  << ", y=" << state(1) 
                  << ", theta=" << state(2) << std::endl;
    }

    return 0;
}