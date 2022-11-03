#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"
#include <corecrt_math_defines.h>
/*���ܣ����Ĳο�����Ϣ��������λ��������š�ɲ����ת�Ǹ�������
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

	static double calculateSteering(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		std::vector<double> pts;
		for (size_t i = 0; i < targetPath.size(); ++i) {
			pts.push_back(pow((double)targetPath[i].first, 2) + pow((double)targetPath[i].second, 2));
		}
		size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();// ����������������ĵ������
		std::cout << "index: " << index << std::endl;
		size_t forwardIndex = 0;
		double minProgDist = 0.5f;
		double progTime = 0.1f;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;// Ԥ�����

		// �ҵ�Ԥ����index
		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].first, 2) +
				pow((double)targetPath[index].second, 2));
			if (distance >= progDist) {
				break;
			}
		}
		double psi = (double)pEgo->yaw;// �����
		double deltaAlfa = atan2(targetPath[forwardIndex].second,
			targetPath[forwardIndex].first) - psi;// ����ƫ��
		double ld = sqrt(pow(targetPath[forwardIndex].second, 2) +
			pow(targetPath[forwardIndex].first, 2)); // ����ƫ��
		double steer = atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180 * 5 / (1 * M_PI);
		if (steer > 30) {
			steer = 30;
		}
		else if (steer < -30) {
			steer = -30;
		}
		return steer;
	}

	static double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		auto this_kappa = targetPath[point_index].kappa;
		double future_kappa;
		int forward_point = 2;
		if (point_index + forward_point < targetPath.size()) {
			future_kappa = targetPath[point_index + forward_point].kappa;
		}
		else {
			future_kappa = targetPath[point_index + forward_point - targetPath.size()].kappa;
		}
		
		//auto max_v = sqrt((2 * 9.8 / (this_kappa > future_kappa ? this_kappa : future_kappa)));
		//auto max_v = sqrt(2 * 9.8 / (this_kappa < 0.01 ? 0.01 : this_kappa));
		auto max_v = 100;
		return PID_Control(max_v, pEgo->speed);
	}

	static double PID_Control(double value_target, double value_now) {
		double dt = 0.01;
		double kp = 0.1;
		double ki = 0.01;
		double kd = 1;

		double value_p = value_target - value_now;
		value_i += (value_target - value_now) * dt;
		//double value_d = (value_now - value_last) / dt;

		double control_value = kp * value_p + ki * value_i;
		if (control_value > 1) control_value = 1;
		if (control_value < -1) control_value = -1;

		return control_value;

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



