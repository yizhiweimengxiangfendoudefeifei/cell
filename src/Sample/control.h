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
	virtual double calculateCmd(const std::vector<RefPoint> &targetPath, PanoSimSensorBus::Lidar_ObjList_G * pLidar) = 0;

public:
	// calc forwardindex
	static int calc_forwardIndex(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		// Nearest point index after sort is 0
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
		double minProgDist = 1.0;
		double progTime = 0.5;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].x, 2) +
				pow((double)targetPath[index].y, 2));
			if (distance >= progDist) {
				//std::cout << "dis: " << distance << "  progdis: " << progDist << std::endl;
				return forwardIndex;
			}
		}
		return 0;
	}

	static double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo, int forwardIndex) {

		auto this_kappa = 0.01;
		//std::cout << "all kappa: ";
		for (int i = 0; i <= forwardIndex; i++) {
			this_kappa = this_kappa > abs(targetPath[i].kappa) ? this_kappa : abs(targetPath[i].kappa);
			//std::cout << "\t" << this_kappa;
		}
		//std::cout << std::endl;
		this_kappa = this_kappa < 0.012 ? 0.012 : this_kappa;

		auto max_v = sqrt( 2.0 / this_kappa);
		/*std::cout << "this_kappa: " << this_kappa << std::endl;
		std::cout << "longtitude forwardIndex: " << forwardIndex << std::endl;*/
		// std::cout << "nearKappa : " << nearKappa << "\t farKappa : " << farKappa << "\t lastKappa :" << lastKappa << std::endl;
		/*std::cout << "max_v is :" << max_v  << "\tand pEgo->speed is : " << pEgo->speed << std::endl;
		std::cout << "this_kappa is :" << this_kappa << std::endl;*/
		return PID_Control(max_v > 5 ? 5 : max_v, pEgo->speed);
	}

	static double PID_Control(double value_target, double value_now) {
		double dt = 0.01;
		double kp = 0.50;
		double ki = 0.00;
		double kd = 0.00;

		double value_p = value_target - value_now;

		value_i += (value_target - value_now) * dt;
		double value_d = (value_now - value_last) / dt;
		
		double control_value = kp * value_p + ki * value_i + kd * value_d;
		control_value = (1 / (1 + exp(-(control_value))) - 0.55) * 2;
		//std::cout << "control_value is : " << control_value << std::endl;
		if (control_value > 1) control_value = 1;
		if (control_value < -1) control_value = -1;
		//std::cout << "control_value after limit is : " << control_value << std::endl;
		value_last = value_now;
		return control_value;

	}
};





#endif // !_CONTROL_



