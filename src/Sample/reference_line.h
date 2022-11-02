#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <vector>
#include <Eigen\Dense>
#include <SensorBusDef.h>
/*���ܣ�����׶Ͱλ�ã����ɳ�����ʻ�ο���
*/
class referenceLine
{
public:
	referenceLine()=default;
	~referenceLine()=default;
	/*����׶Ͱ��ɫ����Ȧ����Ȧ��׶Ͱ�ֱ�洢����
	*/
	void shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	/*������Ȧ��׶ͰѰ�Ҿ����������Ȧ׶Ͱ��index
	*/
	void findIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	// �洢���ĵ������
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
	std::vector<std::pair<double, double>> center_point_xy; // �洢�ο��ߵ�x y
	std::vector<std::pair<double, double>> center_point_xy_final; // �洢������ֵ�ο��ߵ�x y
	std::vector<std::pair<double, double>> in_xy;// ����Ȧ����洢����
	std::vector<std::pair<double, double>> out_xy;// ����Ȧ����洢����
	int match_point_index_set[200];// ��Ȧ��Ӧ�����������Ȧindex
	int inner = 0;// ��Ȧ׶Ͱ��
	int outter = 0;// ��Ȧ׶Ͱ��

};

struct Point3d_s
{
	double x;
	double y;
	double z;
};

#endif // !__reference_line__

