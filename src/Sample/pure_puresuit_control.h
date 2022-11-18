#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once
#include "control.h"

class purePursuit : public control
{
public:
	purePursuit() = default;
	~purePursuit() = default;

	double calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar) override;
};

#endif // !__PURE_PURSUIT__

