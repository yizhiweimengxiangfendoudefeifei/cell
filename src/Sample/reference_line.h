#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <vector>
#include <Eigen\Dense>
#include <SensorBusDef.h>
/*功能：订阅锥桶位置，生成车辆行驶参考线
*/
struct Point3d_s
{
	double x;
	double y;
	double z;
};

struct Point2d_s
{
	double x;
	double y;
};

struct RefPoint
{
	double x;
	double y;
	double kappa;
};


class referenceLine
{
public:
	referenceLine() = default;
	~referenceLine() = default;
	/*根据锥桶颜色对内圈和外圈的锥桶分别存储坐标
	*/
	void shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	/*根据外圈的锥桶寻找距离最近的内圈锥桶的index
	*/
	void findIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	// 存储中心点的索引
	void calcCenterPoint() {
		//std::cout << "inner num: " << this->inner << std::endl;
		for (int i = 0; i < this->inner; ++i) {
			center_point_xy.emplace_back((out_xy[i].first + in_xy[match_point_index_set[i]].first) / 2,
				(out_xy[i].second + in_xy[match_point_index_set[i]].second) / 2);
		}
		//std::cout << center_point_xy[127].first << std::endl;
	}
	void average_interpolation(Eigen::MatrixXd& input, std::vector<std::pair<double, double>>& output, double interval_dis,
		double distance);

	std::vector<std::pair<double, double>> get_center_point_xy() {
		return center_point_xy;
	}

	std::vector<std::pair<double, double>> get_center_point_xy_final() {
		return center_point_xy_final;
	}

	double calculate_kappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);//利用三点法计算该点的曲率

	void get_kappa(std::vector<std::pair<double, double>> center_point_xy_final);

	double calculate_velocity(double k);

private:
	std::vector<std::pair<double, double>> center_point_xy; // 存储参考线的x y
	std::vector<std::pair<double, double>> center_point_xy_final; // 存储经过插值参考线的x y
	std::vector<std::pair<double, double>> in_xy;// 将内圈坐标存储下来
	std::vector<std::pair<double, double>> out_xy;// 将外圈坐标存储下来
	std::vector<int> match_point_index_set;// 外圈对应距离最近的内圈index
	std::vector<RefPoint> RefMsg;//存储最终的参考线信息，包括下x、y坐标和该点的曲率
	int inner = 0;// 内圈锥桶数
	int outter = 0;// 外圈锥桶数
	int RefPointCounter;//最终的参考线点集中点的数量
};



#endif // !__reference_line__

