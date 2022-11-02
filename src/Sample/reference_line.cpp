

#include <iostream>
#include "reference_line.h"

/*����׶Ͱ��ɫ����Ȧ����Ȧ��׶Ͱ�ֱ�洢����
*/
void referenceLine::shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	for (int i = 0; i < pLidar->header.width; ++i) {
		if (pLidar->items[i].shape == 2) {
			// �洢��Ȧ������
			this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->outter++;
		}
		else {
			// �洢��Ȧ������
			this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->inner++;
		}
	}
}

/*������Ȧ��׶ͰѰ�Ҿ����������Ȧ׶Ͱ��index
*/
void referenceLine::findIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	/*std::cout << "inner num: " << this->inner << std::endl;
	std::cout << "outter num: " << this->outter << std::endl;*/
	for (int i = 0; i < out_xy.size(); ++i) {
		double min_dis = std::numeric_limits<int>::max();
		for (int j = 0; j < in_xy.size(); ++j) {
			double dis = pow(this->out_xy[i].first-this->in_xy[j].first,2)
				+pow(this->out_xy[i].second - this->in_xy[j].second, 2);
			if (dis < min_dis) {
				min_dis = dis;
				this->match_point_index_set[i] = j;
			}
		}
	}
}

/*������õ������ĵ��ֵ�����
*/
void referenceLine::average_interpolation(Eigen::MatrixXd &input,
	std::vector<std::pair<double, double>> &output, 
	double interval_dis,
	double distance) {
	// 1.����һ������������ΪPoint3d_s,����x,y,z��
	std::vector<Point3d_s> vec_3d;
	std::vector<Point3d_s> n_vec;
	Point3d_s p;
	// 2.����
	for (int i = 0; i < input.size() - 1; ++i) {
		double dis = (input.row(i + 1) - input.row(i)).norm();
		// ����������в��
		if (dis >= distance) {
			//����(x,y)����ľ���
			double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
				(input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
			//����Ƕ�
			double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
			double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
			//����֮��Ҫ��ֵ�Ĳ�ֵ�������
			int num = dis / interval_dis;  //�ָ�
			//�����
			for (int j = 0; j < num; j++)
			{
				// i=0,j=0��ʱ����ʵ�ǲ������
				p.x = input(i, 0) + j * interval_dis * cos_a;
				p.y = input(i, 1) + j * interval_dis * sin_a;
				p.z = input(i, 2);
				vec_3d.push_back(p);
			}
		}
		else {
			// ��Щ��ԭ���ȽϽ�������Ҫ��㣬����ҲҪ����ȥ����Ȼ��ȱʧ,dis >= 1��ֹ��̫�ܼ�
			p.x = input(i, 0);
			p.y = input(i, 1);
			p.z = input(i, 2);
			vec_3d.push_back(p);
		}
	}
	// 4.©���յ㣬��Ҫ����
	p.x = input(input.rows() - 1, 0);
	p.y = input(input.rows() - 1, 1);
	p.z = input(input.rows() - 1, 2);
	vec_3d.push_back(p);
	// 5.output
	for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++) {
		output.emplace_back((*it).x, (*it).y);
		/*std::cout << "output[0].x:" << output[0].first << std::endl;
		std::cout << "output[end].x:" << output[output.size()-1].first << std::endl;*/
	}
		
}
