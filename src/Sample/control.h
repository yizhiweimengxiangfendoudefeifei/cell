#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"
#include <corecrt_math_defines.h>
/*功能：订阅参考线消息，车辆定位，输出油门、刹车、转角给主函数
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
	static double calculateSteering(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		//std::vector<double> pts;
		//for (size_t i = 0; i < targetPath.size(); ++i) {
		//	pts.push_back(pow((double)targetPath[i].first, 2) + pow((double)targetPath[i].second, 2));
		//}
		size_t index = point_index;// 距离主车距离最近的点的索引
		std::cout << "index: " << index << std::endl;
		size_t forwardIndex = 0;
		double minProgDist = 0.5f;
		double progTime = 0.1f;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;// 预瞄距离

		// 找到预瞄点的index
		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].x, 2) +
				pow((double)targetPath[index].y, 2));
			if (distance >= progDist) {
				break;
			}
		}
		double psi = (double)pEgo->yaw;// 航向角
		double deltaAlfa = atan2(targetPath[forwardIndex].y,
			targetPath[forwardIndex].x);// 航向偏差
		double ld = sqrt(pow(targetPath[forwardIndex].y, 2) + 
			pow(targetPath[forwardIndex].x, 2)); // 横向偏差
		double steer = -atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180. / (1. * M_PI);

		return steer;
	}

	static double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		auto this_kappa = targetPath[point_index].kappa;
		double future_kappa;
		if (point_index + 5 < targetPath.size()) {
			future_kappa = targetPath[point_index + 5].kappa;
		}
		else {
			future_kappa = targetPath[point_index + 5 - targetPath.size()].kappa;
		}
		
		auto max_v = sqrt( (2 * 9.8 / (this_kappa > future_kappa ? this_kappa : future_kappa)));

		if (pEgo->speed > max_v) {
			return -1;
		}
		else {
			return 1;
		}
	}


public:
	static size_t point_index;
};

size_t control::point_index = 0;

#endif // !_CONTROL_



