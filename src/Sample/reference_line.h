#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <vector>
#include <Eigen\Dense>
#include <SensorBusDef.h>
/*功能：订阅锥桶位置，生成车辆行驶参考线
*/
class referenceLine
{
public:
	referenceLine()=default;
	~referenceLine()=default;
	/*根据锥桶颜色对内圈和外圈的锥桶分别存储坐标
	*/
	void shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	/*根据外圈的锥桶寻找距离最近的内圈锥桶的index
	*/
	void findIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	// 存储中心点的索引
	void calcCenterPoint() {
		for (int i = 0; i < in_xy.size(); ++i) {
			center_point_xy.emplace_back((out_xy[i].first + in_xy[match_point_index_set[i]].first) / 2, 
				(out_xy[i].second + in_xy[match_point_index_set[i]].second) / 2);
		}
	}
	void average_interpolation(Eigen::MatrixXd &input, std::vector<std::pair<double, double>> &output, double interval_dis,
		double distance);

	std::vector<std::pair<double, double>> get_in_xy() {
		return in_xy;
	}

	std::vector<std::pair<double, double>> get_center_point_xy() {
		return center_point_xy;
	}

	std::vector<std::pair<double, double>> get_center_point_xy_final() {
		return center_point_xy_final;
	}

	void set_center_point_xy_final(std::vector<std::pair<double, double>> center) {
		center_point_xy_final = center;
	}
private:
	std::vector<std::pair<double, double>> center_point_xy; // 存储参考线的x y
	std::vector<std::pair<double, double>> center_point_xy_final; // 存储经过插值参考线的x y
	std::vector<std::pair<double, double>> in_xy;// 将内圈坐标存储下来
	std::vector<std::pair<double, double>> out_xy;// 将外圈坐标存储下来
	int match_point_index_set[200];// 外圈对应距离最近的内圈index
	int inner = 0;// 内圈锥桶数
	int outter = 0;// 外圈锥桶数

};

struct Point3d_s
{
	double x;
	double y;
	double z;
};

#endif // !__reference_line__

