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

// 비선형 타이어 힘 계산을 위한 Pacejka 모델
float tireForcePacejka(float slipAngle, float B, float C, float D, float E) {
    // Pacejka의 마법 공식 사용
    float force = D * sin(C * atan(B * slipAngle - E * (B * slipAngle - atan(B * slipAngle))));
    return force;
}

void updateState(VehicleState& state, float Cf, float Cr, float L, float dt, float Bf, float Cf_model, float Df, float Ef, float Br, float Cr_model, float Dr, float Er) {
    float slipFront = state.delta - atan((state.v * sin(state.psi) + L * state.delta) / (state.v * cos(state.psi)));
    float slipRear = -atan((state.v * sin(state.psi)) / (state.v * cos(state.psi)));

    float Ff = tireForcePacejka(slipFront, Bf, Cf_model, Df, Ef);
    float Fr = tireForcePacejka(slipRear, Br, Cr_model, Dr, Er);

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
    float Cf = 100000; 
    float Cr = 100000; 
    float dt = 0.2;
    float T = 3 + 11/5;  
    int N = static_cast<int>(T / dt); 
    float degree = -26;

    // Pacejka 모델 파라미터 설정
    float Bf = 10.0, Cf_model = 1.9, Df = 70000, Ef = 0.97;
    float Br = 10.0, Cr_model = 1.9, Dr = 70000, Er = 0.97;

    // 초기 상태 설정
    VehicleState state = {0.0, 0.0, 0, 11, (M_PI / 180)*degree};

    std::ofstream outFile("vehicle_trajectory.csv");
    outFile << "x,y\n";

    for (int i = 0; i < N; ++i) {
        updateState(state, Cf, Cr, L, dt, Bf, Cf_model, Df, Ef, Br, Cr_model, Dr, Er);
        outFile << state.x << "," << state.y << "\n";
    }

    outFile.close();
    return 0;
}
