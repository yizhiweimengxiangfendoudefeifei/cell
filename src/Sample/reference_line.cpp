

#include <iostream>
#include "reference_line.h"

/*����׶Ͱ��ɫ����Ȧ����Ȧ��׶Ͱ�ֱ�洢
*/
void referenceLine::shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	// �洢ȫ������ϵ������Ȧ׶Ͱ����
	//std::cout << "************************" << std::endl;
	for (int i = 0; i < pLidar->header.width; ++i) {
		//
		/*if (heading < 0) {
			continue;
		}*/
		if (pLidar->items[i].shape == 2) {
			// �洢��Ȧ������
			this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			//std::cout << "outter: " << this->out_xy[this->outter].first << "  " << this->out_xy[this->outter].second << std::endl;
			this->outter++;
				
		}
		else if(pLidar->items[i].shape == 11){
			// �洢��Ȧ������
			this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			//std::cout << "innner: " << this->in_xy[this->inner].first << "  " << this->in_xy[this->inner].second << std::endl;
			this->inner++;
		}

		
	}
}

void referenceLine::calcCenterPoint(){
	// �ҵ�ƥ�������
	for (int i = 0; i < this->out_xy.size(); ++i) {
		double min_dis = (std::numeric_limits<int>::max)();
		int k = 0;
		for (int j = 0; j < this->in_xy.size(); ++j) {
			double dis = pow(this->out_xy[i].first - this->in_xy[j].first, 2)
				+ pow(this->out_xy[i].second - this->in_xy[j].second, 2);
			if (dis < min_dis) {
				min_dis = dis;
				k = j;
			}
		}
		this->match_point_index_set.push_back(k);
	}

	int num_selected = this->in_xy.size() < this->out_xy.size() ? this->in_xy.size() : this->out_xy.size();
	for (int i = 0; i < num_selected; ++i) {
		center_point_xy.emplace_back((this->out_xy[i].first + this->in_xy[this->match_point_index_set[i]].first) / 2,
									 (this->out_xy[i].second + this->in_xy[this->match_point_index_set[i]].second) / 2);
		//std::cout << center_point_xy[i].first << "  " << center_point_xy[i].second << std::endl;
	}
}
/*������Ȧ��׶ͰѰ�Ҿ����������Ȧ׶Ͱ��index
*/
void referenceLine::sortIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar, PanoSimBasicsBus::Ego* pEgo) {
	/*std::cout << "inner num: " << this->inner << std::endl;
	std::cout << "outter num: " << this->outter << std::endl;*/
	// ֱ�Ӷ���������׶Ͱ��������������
	double min_dis = (std::numeric_limits<int>::max)();
	int index_cen = 0;
	for (size_t i = 0; i < this->center_point_xy.size(); ++i) {
		double dis = pow(this->center_point_xy[i].first, 2) + pow(this->center_point_xy[i].second, 2);
		std::cout << "pEgo->yaw: " << pEgo->yaw << std::endl;
		double heading = std::cos(pEgo->yaw) * center_point_xy[i].first + (std::sin(pEgo->yaw)) * center_point_xy[i].second;
		std::cout << "heading: " << heading << std::endl;
		if (dis < min_dis && heading >= 0) {
			// ��֤���ĵ��ڳ���ǰ��
			min_dis = dis;
			index_cen = i;
		}
	}
	std::vector<int> cen_selected(this->center_point_xy.size()); // ȫ����ʼ��Ϊ0
	cen_selected[index_cen] = 1;
	this->match_point_index_set_cen.push_back(index_cen);
	
	int num = 0, j = 0;
	while (this->match_point_index_set_cen.size() < this->center_point_xy.size())
	{
		min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < this->center_point_xy.size(); ++i) {
			double dis = pow(this->center_point_xy[i].first - this->center_point_xy[this->match_point_index_set_cen[num]].first, 2) +
				pow(this->center_point_xy[i].second - this->center_point_xy[this->match_point_index_set_cen[num]].second, 2);
			if (dis < min_dis && cen_selected[i] != 1) {
				j = i;
				min_dis = dis;
			}
		}
		this->match_point_index_set_cen.push_back(j);
		cen_selected[j] = 1;
		num++;
	}

	// ����Ȧ׶Ͱ��������
	/*min_dis = (std::numeric_limits<int>::max)();
	int index_in = 0;
	for (size_t i = 0; i < this->in_xy.size(); ++i) {
		double dis = pow(this->in_xy[i].first, 2) + pow(this->in_xy[i].second, 2);
		if (dis < min_dis) {
			min_dis = dis;
			index_in = i;
		}
	}
	std::vector<int> in_selected;
	for (int i = 0; i < this->in_xy.size(); ++i) {
		in_selected.push_back(0);
	}
	in_selected[index_in] = 1;
	this->match_point_index_set_inner.push_back(index_in);
	num = 0, j = 0;
	while (this->match_point_index_set_inner.size() < this->in_xy.size())
	{
		min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < this->in_xy.size(); ++i) {
			double dis = pow(this->in_xy[i].first - this->in_xy[this->match_point_index_set_inner[num]].first, 2) +
				pow(this->in_xy[i].second - this->in_xy[this->match_point_index_set_inner[num]].second, 2);
			if (dis < min_dis && in_selected[i] != 1) {
				j = i;
				min_dis = dis;
			}
		}
		this->match_point_index_set_inner.push_back(j);
		in_selected[j] = 1;
		num++;
	}*/
}

/*��
*/
void referenceLine::centerPoint() {
	std::cout << "====================" << std::endl;
	for (int i = 0; i < this->center_point_xy.size(); ++i) {
		center_point_xy_sort.emplace_back(this->center_point_xy[this->match_point_index_set_cen[i]].first,
			this->center_point_xy[this->match_point_index_set_cen[i]].second);
		std::cout << center_point_xy_sort[i].first << "  " << center_point_xy_sort[i].second << std::endl;
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


double referenceLine::calculate_kappa(Point2d_s p1, Point2d_s p2, Point2d_s p3)
{
	double a, b, c, sinA, cosA, r, k;
	a = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	b = sqrt((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y));
	c = sqrt((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y));
	cosA = (b * b + c * c - a * a) / (2 * b * c);
	sinA = sqrt(1 - cosA * cosA);
	r = 0.5 * a / sinA;//�����������ڽ�Բ�����Բ�Ĺ�ϵ�������Ҷ��������Բ�İ뾶�������õ�����k
	k = 1 / r;
	return k;
}


void referenceLine::get_kappa(std::vector<std::pair<double, double>> center_point_xy_final)
{
	Point2d_s p1, p2, a, b, c;
	p1.x = center_point_xy_final[0].first;
	p1.y = center_point_xy_final[0].second;
	p2.x = center_point_xy_final[1].first;
	p2.y = center_point_xy_final[1].second;
	RefPoint r;
	for (int i = 0; i < this->RefPointCounter - 2; ++i)
	{
		a.x = center_point_xy_final[i].first;
		a.y = center_point_xy_final[i].second;
		b.x = center_point_xy_final[i + 1].first;
		b.y = center_point_xy_final[i + 1].second;
		c.x = center_point_xy_final[i + 2].first;
		c.y = center_point_xy_final[i + 2].second;
		double k = referenceLine::calculate_kappa(a, b, c);
		RefPoint r;
		r.x = a.x;
		r.y = a.y;
		r.kappa = k;
		RefMsg.emplace_back(r);
	}//��ǰn-2�������Ϣ����vector
	a.x = center_point_xy_final[RefPointCounter - 2].first;
	a.y = center_point_xy_final[RefPointCounter - 2].second;
	b.x = center_point_xy_final[RefPointCounter - 1].first;
	b.y = center_point_xy_final[RefPointCounter - 1].second;
	double k1 = calculate_kappa(a, b, p1);
	r.x = a.x;
	r.y = a.y;
	r.kappa = k1;
	RefMsg.emplace_back(r);
	double k2 = calculate_kappa(b, p1, p2);
	r.x = b.x;
	r.y = b.y;
	r.kappa = k2;
	RefMsg.emplace_back(r);
	//���òο�������������������������㣬���ÿ��RefPoint�����ʲ�����RefMsg
}