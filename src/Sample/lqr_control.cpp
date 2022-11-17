#include "lqr_control.h"

// find nearest point
// int lqr_control::findTrajref(const std::vector<std::pair<double, double>> &targetPath, PanoSimBasicsBus::Ego *pEgo){

// }

lqr_control::lqr_control()
{
    // 纵向速度
    vx = 0.1;
    // 横向速度
    vy = 0;

    //质心侧偏角视为不变
    cf = -774.73*2;
    // 轮胎侧偏刚度
    cr = -115494.663;

    m = 240.1;

    // 最大纵向加速度
    max_lateral_acceleration = 2.0;
    // 最大制动减速度
    standstill_acceleration = -3.0;
    // 轴距
    wheel_base = 1.55;
    // 前轴中心到质心的距离
    a = 0.855;
    // 后轴中心到质心的距离
    b = 0.695;

    // 车辆绕z轴转动的转动惯量
    Iz = 1110.9;

    // 轮胎最大转角(rad)，暂时没用上
    wheel_max_degree = 0.6;
}


double lqr_control::calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
	this->vx = pEgo->speed * pEgo->yaw;
	// 变量再分配
	std::vector<std::pair<double, double>> trj_point_array;
	for (auto Point : targetPath) {
		trj_point_array.emplace_back(Point.x, Point.y);
	}
	std::vector<double> trj_thetas;
	for (auto Point : targetPath) {
		trj_thetas.push_back(Point.theta);
	}
	std::vector<double> trj_kappas;
	for (auto Point : targetPath) {
		trj_kappas.push_back(Point.kappa);
	}
	// 相对坐标，故车辆位置为0
	double currentPositionX = 0;
	double currentPositionY = 0;
	double car_yaw = pEgo->yaw;// ???
	double out_angle = theta_angle(trj_point_array, trj_thetas, trj_kappas, currentPositionX, currentPositionY, car_yaw);
}

double lqr_control::theta_angle(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas,
	std::vector<double>& trj_kappas, double currentPositionX, double currentPositionY, double car_yaw)
{

	std::array<double, 5> err_k = cal_err_k(trj_point_array, trj_thetas, trj_kappas, currentPositionX, currentPositionY, car_yaw);
	Eigen::Matrix<double, 1, 4> k = cal_k(err_k);

	double forword_angle = cal_forword_angle(k, err_k);
	double angle = cal_angle(k, forword_angle, err_k);
	// printf("angle,forword_angle=%.3f,%.3f\n", angle, forword_angle);
	return angle;
}


std::array<double, 5> lqr_control::cal_err_k(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas,
    std::vector<double>& trj_kappas, double current_post_x, double current_post_y, double car_yaw)
{
    std::array<double, 5> err_k;
    int index = 0;// 最近的点
    // 找到index后，开始求解投影点
    // Eigen::Vector2f tor;
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(trj_thetas[index]), sin(trj_thetas[index]);
    // Eigen::Vector2f nor;
    Eigen::Matrix<double, 2, 1> nor;
    nor << -sin(trj_thetas[index]), cos(trj_thetas[index]);

    // Eigen::Vector2f d_err;
    Eigen::Matrix<double, 2, 1> d_err;
    d_err << current_post_x - trj_point_array[index].first, current_post_y - trj_point_array[index].second;

    double phi = car_yaw;

    // nor.transpose()对nor转置
    double ed = nor.transpose() * d_err;
    // double ed = -vx*sin();

    double es = tor.transpose() * d_err;

    // 投影点的threat角度
    double projection_point_threat = trj_thetas[index] + trj_kappas[index] * es;

    // double phi = trj_thetas[index];
    double ed_d = vy * cos(phi - projection_point_threat) +
        vx * sin(phi - projection_point_threat);
    // 计算ephi
     double ephi = sin(phi - projection_point_threat);
    //double ephi = phi - projection_point_threat;

    // 计算s_d
    double s_d = (vx * cos(phi - projection_point_threat) -
        vy * sin(phi - projection_point_threat)) /
        (1 - trj_kappas[index] * ed);
    double phi_d = vx * trj_kappas[index];
    double ephi_d = phi_d - trj_kappas[index] * s_d;

    // 计算投影点曲率k
    double projection_point_curvature = trj_kappas[index];

    err_k[0] = ed;
    err_k[1] = ed_d;
    err_k[2] = ephi;
    err_k[3] = ephi_d;
    err_k[4] = projection_point_curvature;

    return err_k;
}


Eigen::Matrix<double, 1, 4> lqr_control::cal_k(std::array<double, 5> err_k)
{
    Eigen::Matrix4d A;
    A << 0, 1, 0, 0,
        0, (cf + cr) / (m * vx), -(cf + cr) / m, (a * cf - b * cr) / (m * vx),
        0, 0, 0, 1,
        0, (a * cf - b * cr) / (Iz * vx), -(a * cf - b * cr) / Iz, (a * a * cf + b * b * cr) / (Iz * vx);

    // Eigen::Vector4f B;
    Eigen::Matrix<double, 4, 1> B;
    B << 0, -cf / m, 0, -a * cf / Iz;

    // Eigen::Matrix4f Q;
    // // 设置成单位矩阵
    Eigen::Matrix4d Q;
    // Q.setIdentity(4, 4);
    Q(0, 0) = 60;
    Q(1, 1) = 1;
    Q(2, 2) = 1;
    Q(3, 3) = 1;

    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = 35.0;
    // MatrixXd矩阵只能用(),VectorXd不仅能用()还能用[]
    Eigen::Matrix<double, 1, 4> k = cal_dlqr(A, B, Q, R);

    return k;
}


Eigen::Matrix<double, 1, 4> lqr_control::cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double, 4, 1> B,
    Eigen::Matrix4d Q, Eigen::Matrix<double, 1, 1> R)
{
    // 设置最大循环迭代次数
    int numLoop = 200;
    // 设置目标极小值
    double minValue = 10e-10;
    Eigen::Matrix4d p_old;
    p_old = Q;

    // 离散化状态方程
    double ts = 0.001;
    Eigen::Matrix4d eye;
    eye.setIdentity(4, 4);

    Eigen::Matrix4d Ad;
    Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
    Eigen::Matrix<double, 4, 1> Bd;
    Bd = B * ts;
    for (int i = 0; i < numLoop; i++)
    {
        // B.inverse()求逆
        Eigen::Matrix4d p_new = Ad.transpose() * p_old * Ad -
            Ad.transpose() * p_old * Bd *
            (R + Bd.transpose() * p_old * Bd).inverse() *
            Bd.transpose() * p_old * Ad +
            Q;
        // p.determinant()求行列式
        // if (std::abs((p_old - p_new).determinant()) <= minValue) {
        // cwiseAbs()求绝对值、maxCoeff()求最大系数
        if (fabs((p_new - p_old).maxCoeff()) < minValue)
        {
            p_old = p_new;
            break;
        }
        p_old = p_new;
    }
    Eigen::Matrix<double, 1, 4> k;
    // Eigen::RowVector4f;
    // 当两个超出范围的浮点数（即INF）进行运算时，运算结果会成为NaN。
    k = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
    return k;
}

double lqr_control::cal_forword_angle(Eigen::Matrix<double, 1, 4> k,
    std::array<double, 5> err_k)
{
    double k3 = k[2];
    // 不足转向系数
    double kv = a * m / (cr * wheel_base) - b * m / (cf * wheel_base);

    //投影点的曲率final_path.k[index]
    double point_curvature = err_k[4];
    double forword_angle =
        wheel_base * point_curvature + kv * vx * vx * point_curvature -
        k3 * (b * point_curvature + a * m * vx * vx * point_curvature / cr / (a + b));
    // double forword_angle =
    //     wheel_base * point_curvature + kv * vx * vx * point_curvature -
    //     k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);
    // double forword_angle =
    //     projection_point_curvature *
    //     (a + b - b * k3 -
    //      (m * std::pow(vx, 2) / (a + b)) * (b / cf + a * k3 / cr - a / cr));
    return forword_angle;
}

double lqr_control::cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle,
    std::array<double, 5> err_k)
{
    Eigen::Matrix<double, 4, 1> err;
    err << err_k[0], err_k[1], err_k[2], err_k[3];
    double angle = -(-k * err + forword_angle) * 3.67;
    
    if (angle > 120) {
        angle = 120;
    }
    else if (angle < -120) {
        angle = -120;
    }
    return angle;
}
