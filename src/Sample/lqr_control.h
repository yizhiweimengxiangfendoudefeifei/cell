#ifndef __LQR_CONTROL__
#define __LQR_CONTROL__

#pragma once

#include "control.h"

class lqrControl : public control
{
public:
    lqrControl();
    ~lqrControl() = default;

    double calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar) override;

    // 计算前轮转角
    double theta_angle(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas,
        std::vector<double>& trj_kappas, double currentPositionX, double currentPositionY, double car_yaw);

    // 计算误差 ed ed' ephi ephi'
    std::array<double, 5> cal_err_k(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas, 
        std::vector<double>& trj_kappas, double current_post_x, double current_post_y, double car_yaw);

    // 计算lqr的k1 k2 k3 k4
    Eigen::Matrix<double, 1, 4> cal_k(std::array<double, 5> err_k);

    // 计算dlqr
    Eigen::Matrix<double, 1, 4> cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double, 4, 1> B,
        Eigen::Matrix4d Q, Eigen::Matrix<double, 1, 1> R);

    // 计算前馈
    double cal_forword_angle(Eigen::Matrix<double, 1, 4> k, std::array<double, 5> err_k);

    // 计算角度
    double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle, std::array<double, 5> err_k);


private:
    // 纵向速度
    double vx;
    // 横向速度
    double vy;
    // 轮胎侧偏刚度
    double cf, cr;

    // 前后悬架载荷
    double m;

    // 最大纵向加速度
    double max_lateral_acceleration;
    // 最大制动减速度
    double standstill_acceleration;
    // 轴距
    double wheel_base;
    // 前轴中心到质心的距离
    double a;
    // 后轴中心到质心的距离
    double b;

    // 车辆绕z轴转动的转动惯量
    double Iz;

    // 轮胎最大转角(rad)
    double wheel_max_degree;
};



#endif __LQR_CONTROL__