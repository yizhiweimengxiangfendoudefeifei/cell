#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once
#include "control.h"

class purePursuit : public control
{
public:
	purePursuit(const double kp, const double ki, const double kd);
	~purePursuit() = default;

	double calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar, 
		PanoSimBasicsBus::Ego* pEgo) override;
};

#endif // !__PURE_PURSUIT__

