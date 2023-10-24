#include <iostream>
#include <vector>
#include <cmath>
#include <qpOASES.hpp>

// 小车状态
struct State {
    double x;
    double y;
    double theta;
    double v;
};

// 小车控制输入
struct Control {
    double v;       // 速度
    double omega;   // 转向角速度
};

// 小车动态模型
State update(const State& state, const Control& control, double dt) {
    State next_state;
    next_state.x = state.x + control.v * cos(state.theta) * dt;
    next_state.y = state.y + control.v * sin(state.theta) * dt;
    next_state.theta = state.theta + control.omega * dt;
    next_state.v = control.v;
    return next_state;
}

// MPC控制器
Control MPC(const State& current_state) {
    using namespace qpOASES;
    
    // TODO: Define the QP problem for MPC
    // This is a very basic example and should be elaborated upon for real applications.
    
    // Define QP matrices. For example:
    real_t H[2*2] = { 1.0, 0.0, 0.0, 1.0 }; // Cost matrix: we want to minimize both v and omega
    real_t A[2*2] = { 1.0, 0.0, 0.0, 1.0 }; // Constraints matrix
    real_t g[2] = { -current_state.x, -current_state.y };  // Linear term: we want to go to origin
    real_t lb[2] = { 0.0, -M_PI/4 }; // Lower bounds for v and omega
    real_t ub[2] = { 2.0, M_PI/4 };  // Upper bounds for v and omega

    // Create and setup QP solver
    QProblem qp(2, 2);
// 使用这个init调用，只传递我们确实定义了的参数
int nWSR = 1000; // 假设最大迭代次数为1000
qp.init(H, g, A, lb, ub, NULL, NULL, nWSR);

    // Get the optimal control inputs
    real_t solution[2];
    qp.getPrimalSolution(solution);

    Control optimal_control;
    optimal_control.v = solution[0];
    optimal_control.omega = solution[1];
    
    return optimal_control;
}

int main() {
    State current_state = {2.0, 2.0, 2.0, 1.0};
    double dt = 0.1;

    for (int i = 0; i < 100; ++i) {
        Control control = MPC(current_state);
        current_state = update(current_state, control, dt);

        std::cout << "x: " << current_state.x 
                  << ", y: " << current_state.y 
                  << ", theta: " << current_state.theta 
                  << ", v: " << current_state.v << std::endl;
    }

    return 0;
}
