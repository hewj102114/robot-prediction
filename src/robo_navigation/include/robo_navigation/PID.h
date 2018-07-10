#include <math.h>
#include <iostream>

class PIDctrl
{
  public:
	double Kp;	 //比例系数
	double Ki;	 //积分系数
	double Kd;	 //微分系数
	double preErr; //前一拍误差
	double sumErr; //误差累积
	double dErr;
	double output;
	double outMax; //输出最大值
	bool stop;

	PIDctrl()
	{
		preErr = 0.0;
		sumErr = 0.0;
	};
	void init(double Kp, double Ki, double Kd, double max);
	double calc(double &curErr);
	void reset();
};

void PIDctrl::init(double _Kp, double _Ki, double _Kd, double _max)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	outMax = _max;
	stop = false;
}

void PIDctrl::reset()
{
	preErr = 0.0;
	sumErr = 0.0;
	dErr = 0.0;
}

double PIDctrl::calc(double &curErr)
{
	sumErr += curErr;
	dErr = curErr - preErr;
	preErr = curErr;
	double Kisum = Ki * sumErr;
	if (Kisum > 0.8)
		Kisum = 0.8;
	if (Kisum < -0.8)
		Kisum = -0.8;
	output = Kp * curErr + Kisum + Kd * dErr;
	if (abs(output) > outMax)
	{
		output = outMax * output / abs(output);
	}
	if (stop == true)
		output = output / 5;
	return output;
}
