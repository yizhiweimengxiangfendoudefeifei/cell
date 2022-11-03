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
	static double calculateSteering(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		std::vector<double> pts;
		for (size_t i = 0; i < targetPath.size(); ++i) {
			pts.push_back(pow((double)targetPath[i].first, 2) + pow((double)targetPath[i].second, 2));
		}
		size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();// 距离主车距离最近的点的索引
		//std::cout << "index: " << index << std::endl;
		size_t forwardIndex = 0;
		double minProgDist = 3;
		double progTime = 0.8;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;// 预瞄距离

		// 找到预瞄点的index
		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].first, 2) +
				pow((double)targetPath[index].second, 2));
			if (distance >= progDist) {
				break;
			}
		}
		double psi = (double)pEgo->yaw;// 航向角
		double deltaAlfa = atan2(targetPath[forwardIndex].second,
			targetPath[forwardIndex].first) - psi;// 航向偏差
		double ld = sqrt(pow(targetPath[forwardIndex].second, 2) + 
			pow(targetPath[forwardIndex].first, 2)); // 横向偏差
		double steer = atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180 * 5 / (1 * M_PI);
		return steer;
	}


private:

};

#endif // !_CONTROL_



