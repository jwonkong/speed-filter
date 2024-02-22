#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>

const double max_steer = M_PI / 6.0;  // max steering angle (30 degrees in radians)
const double L = 2.9;  // Wheel base of vehicle
const double dt = 0.1;  // Time step
const double Lf = L / 2.0;
const double Lr = L / 2.0;
const double Cf = 1600.0 * 2.0;  // N/rad
const double Cr = 1700.0 * 2.0;  // N/rad
const double Iz = 2250.0;  // kg/m2
const double m = 1500.0;  // kg

// Normalize an angle to [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

class VehicleModel {
public:
    double x, y, yaw, vx, vy, omega;

    VehicleModel(double x = 0.0, double y = 0.0, double yaw = 0.0, double vx = 0.01, double vy = 0.0, double omega = 0.0)
        : x(x), y(y), yaw(yaw), vx(vx), vy(vy), omega(omega) {
        c_a = 1.36;
        c_r1 = 0.01;
    }

    void update(double throttle, double delta) {
        delta = std::max(std::min(delta, max_steer), -max_steer);
        x = x + vx * cos(yaw) * dt - vy * sin(yaw) * dt;
        y = y + vx * sin(yaw) * dt + vy * cos(yaw) * dt;
        yaw = yaw + omega * dt;
        yaw = normalize_angle(yaw);
        double Ffy = -Cf * atan2(((vy + Lf * omega) / vx - delta), 1.0);
        double Fry = -Cr * atan2((vy - Lr * omega) / vx, 1.0);
        double R_x = c_r1 * vx;
        double F_aero = c_a * vx * vx;
        double F_load = F_aero + R_x;
        vx = vx + (throttle - Ffy * sin(delta) / m - F_load / m + vy * omega) * dt;
        vy = vy + (Fry / m + Ffy * cos(delta) / m - vx * omega) * dt;
        omega = omega + (Ffy * Lf * cos(delta) - Fry * Lr) / Iz * dt;
    }

private:
    double c_a;  // Aerodynamic coefficient
    double c_r1; // Rolling resistance coefficient
};

int main() {
    VehicleModel car;
    std::ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";

    // Example values for throttle and steering
    double throttle = 1.0; // You can vary throttle and delta as needed
    double delta = 0.0;    // Steering angle in radians

    for (int i = 0; i < 1000; ++i) {  // Run for 1000 time steps
        car.update(throttle, delta);
        outFile << car.x << "," << car.y << "\n";
    }

    outFile.close();
    std::cout << "Trajectory saved to vehicle_trajectory.csv\n";

    return 0;
}
