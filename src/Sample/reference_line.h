#ifndef __reference_line__
#define __reference_line__
#pragma once

#include <vector>
#include <Eigen\Dense>
#include <SensorBusDef.h>
#include <BasicsBusDef.h>

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
	/**
	 * @brief save (x,y) according to color
	 * 
	 * @param pLidar 
	 */
	void shape(PanoSimSensorBus::Lidar_ObjList_G* pLidar);
	/**
	 * @brief calc centerpoint
	 * 
	 */
	void calcCenterPoint();
	/**
	 * @brief sort centerpoint
	 * 
	 * @param pLidar 
	 * @param pEgo 
	 */
	void sortIndex(PanoSimSensorBus::Lidar_ObjList_G* pLidar, PanoSimBasicsBus::Ego* pEgo);
	/* save centerpoint (x,y) according to sort index
	*/
	void centerPoint();
	/**
	 * @brief interpolation
	 * 
	 * @param input enter_point_xy_sort
	 * @param output enter_point_xy_final
	 * @param interval_dis 
	 * @param distance max dis
	 */
	void average_interpolation(Eigen::MatrixXd& input, std::vector<std::pair<double, double>>& output, double interval_dis,
		double distance);

	std::vector<std::pair<double, double>> get_center_point_xy_sort() {
		return center_point_xy_sort;
	}

	std::vector<std::pair<double, double>> get_center_point_xy_final() {
		return center_point_xy_final;
	}

	void set_center_point_xy_final(std::vector<std::pair<double, double>> input) {
		this->center_point_xy_final = input;
	}
	double calculate_kappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);// 

	void get_kappa(std::vector<std::pair<double, double>> center_point_xy_final);

	double calculate_velocity(double k);
	std::vector<RefPoint> getRefMsg() {
		return this->RefMsg;
	}
private:
	std::vector<std::pair<double, double>> center_point_xy; 
	std::vector<std::pair<double, double>> center_point_xy_sort;// centerpoint after sort
	std::vector<std::pair<double, double>> center_point_xy_final; // centerpoint after interpolation
	std::vector<std::pair<double, double>> in_xy;// (x,y)
	std::vector<std::pair<double, double>> out_xy;
	std::vector<std::pair<double, double>> yellow_xy;
	std::vector<int> match_point_index_set;// sort index in "calcCenterPoint"
	std::vector<RefPoint> RefMsg;//
	int RefPointCounter;//
	std::vector<int> match_point_index_set_cen;// sort index  in "sortIndex"
	int inner = 0;// num of inner bucket
	int outter = 0;
	int yellower = 0;

};



#endif // !__reference_line__

