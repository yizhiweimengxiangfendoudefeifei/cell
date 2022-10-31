

#include <iostream>
#include "reference_line.h"

/*根据锥桶颜色对内圈和外圈的锥桶分别存储坐标
*/
void referenceLine::shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	for (int i = 0; i < pLidar->header.width; ++i) {
		if (pLidar->items[i].shape == 2) {
			// 存储外圈的坐标
			this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->outter++;
		}
		else {
			// 存储内圈的坐标
			this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
			this->inner++;
		}
	}
}

/*根据外圈的锥桶寻找距离最近的内圈锥桶的index
*/
void referenceLine::findIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	/*std::cout << "inner num: " << this->inner << std::endl;
	std::cout << "outter num: " << this->outter << std::endl;*/
	for (int i = 0; i < this->outter; ++i) {
		double min_dis = std::numeric_limits<int>::max();
		for (int j = 0; j < this->inner; ++j) {
			double dis = pow(this->out_xy[i].first-this->in_xy[j].first,2)
				+pow(this->out_xy[i].second - this->in_xy[j].second, 2);
			if (dis < min_dis) {
				min_dis = dis;
				this->match_point_index_set.push_back(j);
			}
		}
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
	}
		
}
