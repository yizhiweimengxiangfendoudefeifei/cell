#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"
#include <corecrt_math_defines.h>
/**
 * @brief control class, lateral && longitudinal
 * 
 */

class control
{
public:
	control()=default;
	~control()=default;

	static void get_nearest_point_idx(const std::vector<RefPoint>& targetPath) {
		std::vector<double> pts;
		for (size_t i = 0; i < targetPath.size(); ++i) {
			pts.push_back(pow((double)targetPath[i].x, 2) + pow((double)targetPath[i].y, 2));
		}
		point_index = std::min_element(pts.begin(), pts.end()) - pts.begin();
	}
	static double calculate_yellowdist(const std::vector<RefPoint>& targetPath) {
		double yellodist=pow(((double)targetPath[0].x+ (double)targetPath[1].x)/2, 2) + pow(((double)targetPath[0].y + (double)targetPath[1].y) / 2, 2);
		return yellodist;
	}
	static double calculateSteering(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		// Nearest point index after sort is 0
		size_t index = 0;
		
		size_t forwardIndex = 0;
		double minProgDist = 2;
		double progTime = 0.3;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		
		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].first, 2) +
				pow((double)targetPath[index].second, 2));
			if (distance >= progDist) {
				break;
			}
		}

		std::cout << "forwardIndex: " << forwardIndex << std::endl;
		double deltaAlfa = atan2(targetPath[forwardIndex].second,
			targetPath[forwardIndex].first);// alfa
		double ld = sqrt(pow(targetPath[forwardIndex].second, 2) +
			pow(targetPath[forwardIndex].first, 2)); // distance 
		double steer = atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180 * 3.33 / (1 * M_PI);
		if (steer > 120) {
			steer = 120;
		}
		else if (steer < -120) {
			steer = -120;
		}
		return steer;
	}

	static double calculateThrottleBreak(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo) {

		size_t forwardIndex = 0;
		size_t thisIndex = 0;
		double minProgDist = 4;
		double progTime = 1.5;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		for (int index = 0; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].first, 2) +
				pow((double)targetPath[index].second, 2));
			if (distance > 1) thisIndex = index;
			if (distance >= progDist) {
				break;
			}
		}
		auto nearKappa = calculateKappa(targetPath, 0);
		auto farKappa = calculateKappa(targetPath, forwardIndex);
		auto lastKappa = calculateKappa(targetPath, targetPath.size() - 5);
		auto this_kappa = nearKappa > farKappa ? nearKappa : farKappa;
		if (lastKappa > 0.27) this_kappa = lastKappa * 1.5;
		this_kappa = this_kappa < 0.012 ? 0.012 : this_kappa;

		auto max_v = sqrt( 2.7 / this_kappa);
		std::cout << "nearKappa : " << nearKappa << "\t farKappa : " << farKappa << std::endl;
		std::cout << "max_v is :" << max_v  << "\t and pEgo->speed is : " << pEgo->speed << std::endl;
		std::cout << "targetPath.size() is :" << targetPath.size() << std::endl;
		std::cout << "this_kappa is :" << this_kappa << std::endl;
		std::cout << "-----------------" << std::endl;
		return PID_Control(max_v, pEgo->speed);
	}

	static double PID_Control(double value_target, double value_now) {
		double dt = 0.01;
		double kp = 2;
		double ki = 0.25;
		double kd = 1;

		double value_p = (value_target - value_now) / value_target;
		value_i += (value_target - value_now) * dt / value_target;
		//double value_d = (value_now - value_last) / dt;

		double control_value = kp * value_p + ki * value_i;
		std::cout << "control_value is : " << control_value << std::endl;
		if (control_value > 1) control_value = 1;
		if (control_value < -1) control_value = -1;
		std::cout << "control_value after limit is : " << control_value << std::endl;

		return control_value;

	}

	static double calculateKappa(const std::vector<std::pair<double, double>>& targetPath, int idx) {
		Point2d_s p1, p2, p3;
		p1.x = targetPath[idx].first;
		p1.y = targetPath[idx].second;
		p2.x = targetPath[idx+1].first;
		p2.y = targetPath[idx+1].second;
		p3.x = targetPath[idx + 2].first;
		p3.y = targetPath[idx + 2].second;
		
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

public:
	static size_t point_index;
	static double value_i;
	static double value_last;

};

size_t control::point_index = 0;
double control::value_i = 0;
double control::value_last = 0;



#endif // !_CONTROL_



