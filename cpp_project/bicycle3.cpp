#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

struct VehicleState {
    float x;     // x 좌표
    float y;     // y 좌표
    float psi;   // 방향(heading)
    float v;     // 속도
    float delta; // 조향각
};

void updateState(VehicleState& state, float lf, float lr, float dt) {
    float alpha_f = 0.0;
    float alpha_r = 0.0; 

    float beta = atan((-lf * tan(alpha_r) + lr * tan(state.delta - alpha_f)) / (lf + lr));

    float x_dot = state.v * cos(beta + state.psi);
    float y_dot = state.v * sin(beta + state.psi);
    float psi_dot = state.v * cos(beta) * (tan(state.delta - alpha_f) + tan(alpha_r)) / (lf + lr);

    state.x += x_dot * dt;
    state.y += y_dot * dt;
    state.psi += psi_dot * dt;
}

int main() {
    float lf = 1.2;  // 전륜에서 CG까지의 거리
    float lr = 1.2;  // 후륜에서 CG까지의 거리
    float dt = 0.1;  // 시간 간격
    float totalTime = 10.0;  // 총 시간
    int steps = totalTime / dt;  // 업데이트 횟수
    float initialSpeed = 10.0;  // 초기 속도 m/s
    float initialDelta = M_PI / 18;  // 초기 조향각 (10도)

    VehicleState state = {0.0, 0.0, 0.0, initialSpeed, initialDelta}; // 초기 상태

    ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";

    for (int i = 0; i < steps; ++i) {
        updateState(state, lf, lr, dt);
        outFile << state.x << "," << state.y << "\n";
    }

    outFile.close();
    cout << "Trajectory saved to vehicle_trajectory.csv\n";

    return 0;
}
