#include "control.h"

control::control(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
	error_sub_ = 0.0;
	previous_error_ = 0.0;
	integral_ = 0.0;
	differential_ = 0.0;
	first_init_ = true;
}

// calc forwardindex
int control::calc_forwardIndex(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo) {

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
	double minProgDist = 3.5;// 3.5
	double progTime = 0.8; // 0.8s
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

double control::calculateThrottleBreak(const std::vector<RefPoint>& targetPath, PanoSimBasicsBus::Ego* pEgo, size_t forwardIndex) {
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
	if (lastKappa > 0.10) {
		this_kappa = lastKappa;
	}
	this_kappa = this_kappa < 0.04 ? 0.04 : this_kappa;// 0.04 7m/s

	auto max_v = sqrt(2.0 / this_kappa);
	/*std::cout << "longtitude forwardIndex: " << forwardIndex << std::endl;
	std::cout << "nearKappa : " << nearKappa << "\t farKappa : " << farKappa << "\t lastKappa :" << lastKappa << std::endl;
	std::cout << "max_v is :" << max_v << "\tand pEgo->speed is : " << pEgo->speed << std::endl;
	std::cout << "targetPath.size() is :" << targetPath.size() << std::endl;
	std::cout << "this_kappa is :" << this_kappa << std::endl;*/
	std::cout << pEgo->speed;
	return PID_Control(max_v > 5.0 ? 5.0 : max_v, pEgo->speed);
}

double control::PID_Control(double value_target, double value_now) {
	
	double dt = 0.01;
	if (fabs(integral_) > 5) {
		reset();
	}
	else if (integral_ < -3 && this->integral_ + dt * (value_target - value_now) < this->integral_) {

	}
	else {
		this->integral_ = this->integral_ + dt * (value_target - value_now); //积分环节
	}
	if (this->first_init_) {
		this->first_init_ = false;
	}
	else {
		this->differential_ = this->error_sub_ / dt;
	}

	double control_value = this->kp_ * (value_target - value_now) + this->ki_ * this->integral_ + this->kd_ * this->differential_;
	//更新误差
	std::cout << " target v:" << value_target << " now v:" << value_now << " control_value:" << control_value << " integral_:" << integral_ 
		 << std::endl;

	this->previous_error_ = value_target - value_now;

	return control_value;

	// 以下设计存在错误，但是在实际使用中效果还比较好
	//double dt = 0.01;
	//double kp = 0.30;
	//double ki = 0.1;
	//double kd = 0.01;

	//double value_p = (value_target - value_now) / value_target;
	//value_i += (value_target - value_now) * dt / value_target;
	//double value_d = (value_now - value_last) / dt;

	//double control_value = kp * value_p + ki * value_i + kd * value_d;
	////std::cout << "control_value is : " << control_value << std::endl;
	//if (control_value > 1) control_value = 1;
	//if (control_value < -1) control_value = -1;
	////std::cout << "control_value after limit is : " << control_value << std::endl;
	//std::cout << " target v:" << value_target << " now v:" << value_now << " control_value:" << control_value << std::endl;
	//value_last = value_now;
	//return control_value;
}

double control::calculateKappa(const std::vector<RefPoint>& targetPath, int idx) {
	Point2d_s p1, p2, p3;
	if (idx + 25 < targetPath.size()) {
		p1.x = targetPath[idx].x;
		p1.y = targetPath[idx].y;
		p2.x = targetPath[idx + 12].x;
		p2.y = targetPath[idx + 12].y;
		p3.x = targetPath[idx + 25].x;
		p3.y = targetPath[idx + 25].y;
	}
	else {
		int num = idx + 25 - targetPath.size();
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

void control::reset() {
	integral_ = 0;
	previous_error_ = 0;
	first_init_ = true;
}

