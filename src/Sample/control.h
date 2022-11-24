#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"

/**
 * @brief control class, lateral && longitudinal
 * 
 */
static double value_i = 0;
static double value_last = 0;
class control
{
public:
	control()=default;
	virtual ~control()=default;

	// virtual function, base, lqrControl and pure_suit is inherited
	virtual double calculateCmd(const std::vector<RefPoint> &targetPath, PanoSimSensorBus::Lidar_ObjList_G * pLidar, PanoSimBasicsBus::Ego* pEgo) = 0;

public:
	// calc forwardindex
	static int calc_forwardIndex(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		
		int index = 0;
		double min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < targetPath.size(); ++i) {
			double dis = pow(targetPath[i].x, 2) + pow(targetPath[i].y, 2);
			if (dis < min_dis) {

				min_dis = dis;
				index = i;
			}
		}
		int forwardIndex = 0;
		double minProgDist = 3.0;
		double progTime = 0.8;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].x, 2) +
				pow((double)targetPath[index].y, 2));
			if (distance >= progDist) {
				return forwardIndex;
			}
		}
		return 0;
	}

	static double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo, size_t forwardIndex) {
		int index = 0;
		double min_dis = (std::numeric_limits<int>::max)();
		for (int i = 0; i < targetPath.size(); ++i) {
			double dis = pow(targetPath[i].x, 2) + pow(targetPath[i].y, 2);
			if (dis < min_dis) {

				min_dis = dis;
				index = i;
			}
		}

		auto nearKappa = calculateKappa(targetPath, index);
		auto farKappa = calculateKappa(targetPath, forwardIndex);
		auto lastKappa = calculateKappa(targetPath, 0);
		double this_kappa = 0.01;
		if (nearKappa > farKappa) {
			this_kappa = nearKappa;
		}
		else {
			this_kappa = farKappa;
		}
		if (lastKappa > 0.20) this_kappa = lastKappa ;
		this_kappa = this_kappa < 0.012 ? 0.012 : this_kappa;

		auto max_v = sqrt(2.0 / this_kappa);
		std::cout << "longtitude forwardIndex: " << forwardIndex << std::endl;
		std::cout << "nearKappa : " << nearKappa << "\t farKappa : " << farKappa << "\t lastKappa :" << lastKappa << std::endl;
		std::cout << "max_v is :" << max_v << "\tand pEgo->speed is : " << pEgo->speed << std::endl;
		std::cout << "targetPath.size() is :" << targetPath.size() << std::endl;
		std::cout << "this_kappa is :" << this_kappa << std::endl;
		return PID_Control(max_v > 7.0 ? 7.0 : max_v, pEgo->speed);
	}

	static double PID_Control(double value_target, double value_now) {
		double dt = 0.01;
		double kp = 0.30;
		double ki = 0.1;
		double kd = 0.01;

		double value_p = (value_target - value_now) / value_target;
		value_i += (value_target - value_now) * dt / value_target;
		double value_d = (value_now - value_last) / dt;

		double control_value = kp * value_p + ki * value_i + kd * value_d;
		std::cout << "control_value is : " << control_value << std::endl;
		if (control_value > 1) control_value = 1;
		if (control_value < -1) control_value = -1;
		std::cout << "control_value after limit is : " << control_value << std::endl;
		value_last = value_now;
		return control_value;

	}

	static double calculateKappa(const std::vector<RefPoint>& targetPath, int idx) {
		Point2d_s p1, p2, p3;
		if (idx + 10 < targetPath.size()) {
			p1.x = targetPath[idx].x;
			p1.y = targetPath[idx].y;
			p2.x = targetPath[idx + 5].x;
			p2.y = targetPath[idx + 5].y;
			p3.x = targetPath[idx + 10].x;
			p3.y = targetPath[idx + 10].y;
		}
		else {
			int num = idx + 10 - targetPath.size();
			p1.x = targetPath[idx].x;
			p1.y = targetPath[idx].y;
			p2.x = targetPath[num].x;
			p2.y = targetPath[num].y;
			p3.x = targetPath[num + 5].x;
			p3.y = targetPath[num + 5].y;
		}


		double a, b, c, sinA, cosA, r, k;

		a = sqrt(abs((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)));
		b = sqrt(abs((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y)));
		c = sqrt(abs((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y)));

		cosA = (b * b + c * c - a * a) / (2 * b * c > 0.01 ? 2 * b * c : 0.01);
		sinA = sqrt(1 - cosA * cosA);
		r = 0.5 * a / (sinA > 0.005 ? sinA : 0.005);
		k = 1 / (r > 0.01 ? r : 0.01);
		return k;

	}
};





#endif // !_CONTROL_



