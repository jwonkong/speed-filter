#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace std;

struct VehicleState {
    float x;     // x 좌표
    float y;     // y 좌표
    float psi;   // 방향(heading)
    float v;     // 속도
    float delta; // 조향각
};

float tireForceNonLinear(float slipAngle, float stiffness, float maxForce) {
    float force = stiffness * slipAngle;
    // 포화 조건 적용
    if (abs(force) > maxForce) {
        force = (force > 0) ? maxForce : -maxForce;
    }
    return force;
}

void updateState(VehicleState& state, float Cf, float Cr, float L, float dt) {
    float slipFront = state.delta - atan((state.v * sin(state.psi) + L * state.delta) / (state.v * cos(state.psi)));
    float slipRear = -atan((state.v * sin(state.psi)) / (state.v * cos(state.psi)));

    float Ff = tireForceNonLinear(slipFront, Cf, 3000);
    float Fr = tireForceNonLinear(slipRear, Cr, 3000);
    cout << "Ff :" << Ff << endl;
    cout << "Fr :" << Fr << endl;

    float x_dot = state.v * cos(state.psi) - state.v * sin(state.psi) * state.delta;
    float y_dot = state.v * sin(state.psi) + state.v * cos(state.psi) * state.delta;
    float psi_dot = state.v / L * (Ff * cos(state.delta) - Fr) / (Cf + Cr);

    state.x += x_dot * dt;
    state.y += y_dot * dt;
    state.psi += psi_dot * dt;
}

int main() {
    float L = 2.85;  
    float Cf = 40000; 
    float Cr = 40000; 
    float dt = 0.2;
    float T = 3 + 11/5;  
    int N = static_cast<int>(T / dt); 
    float degree = 30;

    // 초기 상태 설정
    VehicleState state = {0.0, 0.0, 0, 11, (M_PI / 180)*degree}; // 초기 위치(0,0), 방향 위쪽, 속도  m/s, 조향각 5도

    std::ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";

    for (int i = 0; i < N; ++i) {
        updateState(state, Cf, Cr, L, dt);
        outFile << state.x << "," << state.y << "\n";
    }

    outFile.close();
    return 0;
}