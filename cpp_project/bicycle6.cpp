#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

const float L = 2.54;  // Wheel base of vehicle [m]
const float dt = 0.1;  // Time step [s]
const float Cf = 44000 * 2.0;  // Cornering stiffness front [N/rad]
const float Cr = 47000 * 2.0;  // Cornering stiffness rear [N/rad]
const float Iz = 2420;  // Yaw inertia of the vehicle [kg*m^2]
const float m = 1500.0;  // Mass of the vehicle [kg]
const float PI = 3.14159265358979323846;
using namespace std;

float normalize_angle(float angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

class Vehicle {
public:
    float x;  // X position [m]
    float y;  // Y position [m]
    float yaw;  // Yaw angle [rad]
    float vx;  // Longitudinal velocity [m/s]
    float vy;  // Lateral velocity [m/s]
    float omega;  // Yaw rate [rad/s]

    Vehicle() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), omega(0.0) {}

    void update_nonlinear(float vx, float delta) {
        float Lf = 1.14;  // Distance from CG to front axle [m]
        float Lr = L - Lf;  // Distance from CG to rear axle [m]

        float beta = atan2(Lr / (Lf + Lr) * tan(delta), 1.0);
        float cos_yaw = cos(yaw);
        float sin_yaw = sin(yaw);

        // Update states
        x += vx * cos_yaw * dt - vy * sin_yaw * dt;
        y += vx * sin_yaw * dt + vy * cos_yaw * dt;
        yaw += omega * dt;
        yaw = normalize_angle(yaw);

        float Ffy = -Cf * atan2((vy + Lf * omega) / vx - delta, 1.0);
        float Fry = -Cr * atan2((vy - Lr * omega) / vx, 1.0);

        // Update velocities
        vx = vx + dt * (1 - (Ffy * sin(delta) / m) + vy * omega);
        vy = vy + dt * ((Fry + Ffy * cos(delta)) / m - vx * omega);
        omega += dt * (Ffy * Lf * cos(delta) - Fry * Lr) / Iz;

    }

    void update_linear(float vx, float delta) {
        float Lf = 1.14;
        float Lr = L - Lf;
        float beta = atan2((Lr / L) * tan(delta), 1.0);

        // Update state
        x += vx * cos(yaw+beta) * dt;
        y += vx * sin(yaw+beta) * dt;
        yaw += vx / L * cos(beta) * tan(delta) * dt;
        yaw = normalize_angle(yaw);
    }
};

int main() {
    Vehicle vehicle;
    ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";

    float initial_speed = 5;  // Initial speed [m/s]
    float steering_angle = ((PI / 180) * 20)/20;  // Steering angle [rad]

    for (int i = 0; i < (3 + initial_speed/5)/dt; ++i) {
        vehicle.update_nonlinear(initial_speed, steering_angle);
        outFile << vehicle.x << "," << vehicle.y << "\n";
        if (vehicle.x < 0 ) break;
    }

    outFile.close();
    cout << "Trajectory saved to vehicle_trajectory.csv\n";

    return 0;
}
