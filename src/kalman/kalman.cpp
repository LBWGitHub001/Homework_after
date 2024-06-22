#include "kalman/kalman.hpp"

// sateSize状态量个数
// uSize输入的维度
using namespace std;

Kalman::Kalman(double _time) : T(_time) {//初始化A P H
    A << 1, T,
         0, 1;
    H << 1, 0;
    P.setIdentity();
    x.setZero();
}

void Kalman::Q_set(double qx) {//计算Q矩阵，过程噪声
    Eigen::Vector2d G;
    G <<  T * T / 2.0, T;
    Q = G * qx * G.transpose();
}

void Kalman::R_set(double rx) {//
    R << rx;
}

Eigen::VectorXd Kalman::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
    return x;
}

Eigen::VectorXd Kalman::update(const Eigen::VectorXd &z_meas) {
    Eigen::MatrixXd K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    if(abs(z_meas(0) - x(0))>0.5)
        return x;
    x = x + K * (z_meas - H * x);
    P = (Eigen::Matrix2d::Identity() - K * H) * P;
    return x;
}