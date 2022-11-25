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
	control(const double kp, const double ki, const double kd);
	virtual ~control()=default;

	// virtual function, base, lqrControl and pure_suit is inherited
	virtual double calculateCmd(const std::vector<RefPoint> &targetPath, PanoSimSensorBus::Lidar_ObjList_G * pLidar, 
		PanoSimBasicsBus::Ego* pEgo) = 0;

public:
	// calc forwardindex
	int calc_forwardIndex(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo);

	double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo, size_t forwardIndex);

	double PID_Control(double value_target, double value_now);

	double calculateKappa(const std::vector<RefPoint>& targetPath, int idx);

	void reset();

protected:
	double kp_ = 0.0;
	double ki_ = 0.0;
	double kd_ = 0.0;
	double error_sub_ = 0.0;
	double previous_error_ = 0.0;
	double integral_ = 0.0;
	double differential_ = 0.0;
	bool first_init_ = false;
};

#endif // !_CONTROL_



