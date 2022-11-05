#include <iostream>
#include "reference_line.h"


void referenceLine::shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	// 存储内外圈锥桶坐标
	for (int i = 0; i < pLidar->header.width; ++i) {
		if (pLidar->items[i].shape == 2) {
			this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->outter++;
				
		}
		else if(pLidar->items[i].shape == 11){
			this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->inner++;
		}
		else {
			this->yellow_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
		}
	}
}

void referenceLine::calcCenterPoint(){
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
	}
}


void referenceLine::sortIndex() {
	int index_cen = 0;
	std::vector<int> have_seen(this->center_point_xy.size());	
	double min_dis = (std::numeric_limits<int>::max)();
	for (size_t i = 0; i < this->center_point_xy.size(); ++i) {
		double dis = pow(this->center_point_xy[i].first, 2) + pow(this->center_point_xy[i].second, 2);
		if (dis < min_dis && center_point_xy[i].first > 0) {
			// 确保在自车前方
			min_dis = dis;
			index_cen = i;
		}
	}
	have_seen[index_cen] = 1;// 拍好序的点标记为1
	this->match_point_index_set_cen.push_back(index_cen);
	
	
	int num = 0, j = 0, flag = -1;
	while (this->match_point_index_set_cen.size() < this->center_point_xy.size())
	{
		min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < this->center_point_xy.size(); ++i) {
			double dis = pow(this->center_point_xy[i].first - this->center_point_xy[this->match_point_index_set_cen[num]].first, 2) +
				pow(this->center_point_xy[i].second - this->center_point_xy[this->match_point_index_set_cen[num]].second, 2);
			if (dis < min_dis && have_seen[i] != 1 && (flag >0 ? 1: center_point_xy[i].first > 0)) {
				j = i;
				min_dis = dis;
			}
		}
		flag++;
		this->match_point_index_set_cen.push_back(j);
		have_seen[j] = 1;
		num++;
	}
}


void referenceLine::centerPoint() {
	for (int i = 0; i < this->center_point_xy.size(); ++i) {
		this->center_point_xy_sort.emplace_back(this->center_point_xy[this->match_point_index_set_cen[i]].first,
			this->center_point_xy[this->match_point_index_set_cen[i]].second);
	}
}


void referenceLine::average_interpolation(Eigen::MatrixXd &input,
	std::vector<std::pair<double, double>> &output, 
	double interval_dis,
	double distance) {
	// 1.定义一个容器，类型为Point3d_s,即（x,y,z）
	std::vector<Point3d_s> vec_3d;
	Point3d_s p;
	// 2.遍历
	for (int i = 0; i < input.rows() - 1; ++i) {
		double dis = (input.row(i + 1) - input.row(i)).norm();
		// 距离过长进行插点
		if (dis >= distance) {
			// 计算(x,y)两点的距离
			double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
				(input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
			// 计算角度
			double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
			double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
			// 两点之间要插值的插值点的数量
			int num = dis / interval_dis;
			// 插入点
			for (int j = 0; j < num; j++)
			{
				// i=0,j=0
				p.x = input(i, 0) + j * interval_dis * cos_a;
				p.y = input(i, 1) + j * interval_dis * sin_a;
				p.z = input(i, 2);
				vec_3d.push_back(p);
			}
		}
		else if(dis < distance){
			// 有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
			p.x = input(i, 0);
			p.y = input(i, 1);
			p.z = input(i, 2);
			vec_3d.push_back(p);
		}
	}
	// 3.漏了终点，需要加上
	p.x = input(input.rows() - 1, 0);
	p.y = input(input.rows() - 1, 1);
	p.z = input(input.rows() - 1, 2);
	vec_3d.push_back(p);
	// 4.output
	for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++) {
		output.emplace_back((*it).x, (*it).y);
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
	r = 0.5 * a / sinA;//
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
	}// vector
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
	// 
}