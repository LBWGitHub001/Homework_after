#ifndef KALMAN_HPP
#define KALMAN_HPP

#define QUEEN_LENGTH 20 //计算方差的窗口，窗口越大，滞后越大

#include <cmath>

#include <iostream>
#include <vector>
#include <atomic>

#include <Eigen/Dense>
#include <angles/angles.h>

#include <rclcpp/rclcpp.hpp>


class Kalman {
private:

    //温馨提示,匀加速和匀速模型的维数不一样哦

    Eigen::Matrix2d A;              //模型
    Eigen::Matrix<double,1,2> H;    //测量模型

    Eigen::Matrix2d P;              //预测变量的协方差

    Eigen::Matrix2d Q;              //过程噪声协方差
    Eigen::Matrix<double,1,1> R;    //观测噪声协方差

    Eigen::Vector2d x;              //系统三个参数


    double T;

public:
    //初始化A，H，P矩阵等
    Kalman(double _time);
    void Q_set(double qx);
    void R_set(double rx);

    Eigen::VectorXd predict();
    Eigen::VectorXd update(const Eigen::VectorXd &z_meas);
};

#endif