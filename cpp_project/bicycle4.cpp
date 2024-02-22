#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>

using namespace std;
// Define the vehicle state
struct VehicleState {
    float x;      // x-coordinate
    float y;      // y-coordinate
    float yaw;    // yaw angle
    float vx;     // longitudinal speed
    float vy;     // lateral speed
    float omega;  // angular speed
};

// Define the vehicle parameters
const float max_steer = M_PI / 6.0;  // 30 degrees in radians
const float L = 2.9;                 // Wheelbase of vehicle
const float dt = 0.1;                // Time step
const float Lr = L / 2.0;            // Distance from CG to rear wheel
const float Lf = L - Lr;             // Distance from CG to front wheel
const float Cf = 3200.0;             // Front cornering stiffness
const float Cr = 3400.0;             // Rear cornering stiffness
const float Iz = 2250.0;             // Yaw inertia
const float m = 1500.0;              // Mass of the vehicle

// Function to normalize an angle to [-pi, pi]
float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Update the state of the vehicle using non-linear bicycle model
void updateState(VehicleState& state, float throttle, float delta) {
    delta = max(min(delta, max_steer), -max_steer);
    
    state.x += state.vx * cos(state.yaw) * dt - state.vy * sin(state.yaw) * dt;
    state.y += state.vx * sin(state.yaw) * dt + state.vy * cos(state.yaw) * dt;
    state.yaw += state.omega * dt;
    state.yaw = normalize_angle(state.yaw);
    
    float Ffy = -Cf * atan2(((state.vy + Lf * state.omega) / state.vx - delta), 1.0);
    float Fry = -Cr * atan2((state.vy - Lr * state.omega) / state.vx, 1.0);
    
    float R_x = 0.01 * state.vx;
    float F_aero = 1.36 * state.vx * state.vx;
    float F_load = F_aero + R_x;
    
    state.vx += (throttle - Ffy * sin(delta) / m - F_load / m + state.vy * state.omega) * dt;
    state.vy += (Fry / m + Ffy * cos(delta) / m - state.vx * state.omega) * dt;
    state.omega += (Ffy * Lf * cos(delta) - Fry * Lr) / Iz * dt;
}

int main() {
    // Example usage
    VehicleState state{0.0, 0.0, 0.0, 0.01, 0.0, 0.0}; // Initial state
    float throttle = 11; // Example throttle
    float delta = 5*M_PI/180;    // Example steering angle (radians)
    
    std::ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";
    
    int steps = 100; // Number of steps to simulate
    for (int i = 0; i < steps; ++i) {
        updateState(state, throttle, delta);
        outFile << state.x << "," << state.y << "\n";
    }
    
    outFile.close();
    std::cout << "Trajectory saved to vehicle_trajectory.csv\n";

    return 0;
}
