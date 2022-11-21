#include "pure_puresuit_control.h"

double purePursuit::calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar) {
	int index = 0;

	int forwardIndex = 0;
	double minProgDist = 1.5;
	double progTime = 0.5;
	double mainVehicleSpeed = -pLidar->items->OBJ_Ego_Vx;
	double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

	for (; index < targetPath.size(); ++index) {
		forwardIndex = index;
		double distance = sqrtf((double)pow(targetPath[index].x, 2) +
			pow((double)targetPath[index].y, 2));
		if (distance >= progDist) {
			break;
		}
	}
	// std::cout << "forwardIndex: " << forwardIndex << std::endl;
	double deltaAlfa = atan2(targetPath[forwardIndex].y,
		targetPath[forwardIndex].x);// alfa
	double ld = sqrt(pow(targetPath[forwardIndex].y, 2) +
		pow(targetPath[forwardIndex].x, 2)); // distance 
	double steer = atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180 * 3.67 / (1 * M_PI);

	if (steer > 135) {
		steer = 135;
	}
	else if (steer < -135) {
		steer = -135;
	}
	return steer;
}