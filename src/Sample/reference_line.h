#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <vector>
#include <Eigen\Dense>
#include <SensorBusDef.h>
#include <BasicsBusDef.h>
/*���ܣ�����׶Ͱλ�ã����ɳ�����ʻ�ο���
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
	/*����׶Ͱ��ɫ����Ȧ����Ȧ��׶Ͱ�ֱ�洢����
	*/
	void shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	// �洢���ĵ������
	void calcCenterPoint();
	/*������Ȧ��׶ͰѰ�Ҿ����������Ȧ׶Ͱ��index
	*/
	void sortIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar, PanoSimBasicsBus::Ego* pEgo);
	/*排序后的数组存在一个新的容器中
	*/
	void centerPoint();
	
	void average_interpolation(Eigen::MatrixXd& input, std::vector<std::pair<double, double>>& output, double interval_dis,
		double distance);

	std::vector<std::pair<double, double>> get_center_point_xy_sort() {
		return center_point_xy_sort;
	}

	std::vector<std::pair<double, double>> get_center_point_xy_final() {
		return center_point_xy_final;
	}

	double calculate_kappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);//�������㷨����õ������

	void get_kappa(std::vector<std::pair<double, double>> center_point_xy_final);

	double calculate_velocity(double k);

private:
	std::vector<std::pair<double, double>> center_point_xy; // �洢�ο��ߵ�x y
	std::vector<std::pair<double, double>> center_point_xy_sort;
	std::vector<std::pair<double, double>> center_point_xy_final; // �洢������ֵ�ο��ߵ�x y
	std::vector<std::pair<double, double>> in_xy;// ����Ȧ����洢����
	std::vector<std::pair<double, double>> out_xy;// ����Ȧ����洢����
	std::vector<int> match_point_index_set;// ��Ȧ��Ӧ�����������Ȧindex
	std::vector<RefPoint> RefMsg;//�洢���յĲο�����Ϣ��������x��y����͸õ������
	int RefPointCounter;//���յĲο��ߵ㼯�е������
	std::vector<int> match_point_index_set_cen;// ����Ȧ׶Ͱ��Ӧ������ֵ
	std::vector<int> match_point_index_set_inner;
	int inner = 0;// ����Ȧ׶Ͱ��
	int outter = 0;

};



#endif // !__reference_line__

