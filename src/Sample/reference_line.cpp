

#include <iostream>
#include "reference_line.h"

/*根据锥桶颜色对内圈和外圈的锥桶分别存储
*/
void referenceLine::shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar, PanoSimBasicsBus::Ego* pEgo) {
	// 存储正前方方圆10m的锥桶位置
	std::cout << "************************" << std::endl;
	for (int i = 0; i < pLidar->header.width; ++i) {
		double heading = std::cos(pEgo->yaw) * pLidar->items[i].OBJ_S_X + (-std::sin(pEgo->yaw)) * pLidar->items[i].OBJ_S_Y;
		if (heading < -0.85) {
			continue;
		}
		if (pLidar->items[i].OBJ_S_Dist < 30) {
			if (pLidar->items[i].shape == 2) {
				// 存储外圈的坐标
				this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
				std::cout << "outter: " << this->out_xy[this->outter].first << "  " << this->out_xy[this->outter].second << std::endl;
				this->outter++;
				
			}
			else if(pLidar->items[i].shape == 11){
				// 存储内圈的坐标
				this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
				std::cout << "innner: " << this->in_xy[this->inner].first << "  " << this->in_xy[this->inner].second << std::endl;
				this->inner++;
			}
		}
		
	}
}

/*根据外圈的锥桶寻找距离最近的内圈锥桶的index
*/
void referenceLine::findIndex() {
	/*std::cout << "inner num: " << this->inner << std::endl;
	std::cout << "outter num: " << this->outter << std::endl;*/
	// 将外圈锥桶重新排序：先找到最近的锥桶，然后一次找到距离现在锥桶最近的锥桶的索引存到match_point_index_set_outter中，内圈同理
	double min_dis = (std::numeric_limits<int>::max)();
	int index_out = 0;
	for (size_t i = 0; i < this->out_xy.size(); ++i) {
		double dis = pow(this->out_xy[i].first, 2) + pow(this->out_xy[i].second, 2);
		if (dis < min_dis) {
			min_dis = dis;
			index_out = i;
		}
	}

	std::vector<int> out_selected; // 这里只填0,1表示bool类型
	for (int i = 0; i < this->out_xy.size(); ++i) {
		out_selected.push_back(0);
	}
	out_selected[index_out] = 1;
	// 根据找到的最近的锥桶与其它锥桶的距离，从小到大进行排序
	this->match_point_index_set_outter.push_back(index_out);
	int num = 0, j = 0;
	while (this->match_point_index_set_outter.size() < this->out_xy.size())
	{
		min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < this->out_xy.size(); ++i) {
			double dis = pow(this->out_xy[i].first - this->out_xy[this->match_point_index_set_outter[num]].first, 2) +
				pow(this->out_xy[i].second - this->out_xy[this->match_point_index_set_outter[num]].second, 2);
			if (dis < min_dis && out_selected[i] != 1) {
				j = i;
				min_dis = dis;
			}
		}
		this->match_point_index_set_outter.push_back(j);
		out_selected[j] = 1;
		num++;
	}

	// 将内圈锥桶重新排序
	min_dis = (std::numeric_limits<int>::max)();
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
	}
}

/*将计算得到的中心点插值后输出
*/
void referenceLine::average_interpolation(Eigen::MatrixXd &input,
	std::vector<std::pair<double, double>> &output, 
	double interval_dis,
	double distance) {
	// 1.定义一个容器，类型为Point3d_s,即（x,y,z）
	std::vector<Point3d_s> vec_3d;
	std::vector<Point3d_s> n_vec;
	Point3d_s p;
	// 2.遍历
	for (int i = 0; i < input.size() - 1; ++i) {
		double dis = (input.row(i + 1) - input.row(i)).norm();
		// 距离过长进行插点
		if (dis >= distance) {
			//计算(x,y)两点的距离
			double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
				(input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
			//计算角度
			double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
			double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
			//两点之间要插值的插值点的数量
			int num = dis / interval_dis;  //分割
			//插入点
			for (int j = 0; j < num; j++)
			{
				// i=0,j=0的时候其实是插入起点
				p.x = input(i, 0) + j * interval_dis * cos_a;
				p.y = input(i, 1) + j * interval_dis * sin_a;
				p.z = input(i, 2);
				vec_3d.push_back(p);
			}
		}
		else {
			// 有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
			p.x = input(i, 0);
			p.y = input(i, 1);
			p.z = input(i, 2);
			vec_3d.push_back(p);
		}
	}
	// 4.漏了终点，需要加上
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
	r = 0.5 * a / sinA;//利用三角形内接圆和外接圆的关系和正余弦定理，求得圆的半径，进而得到曲率k
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
	}//将前n-2个点的信息存入vector
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
	//利用参考线中最后两个点和最初的两个点，获得每个RefPoint的曲率并存入RefMsg
}