#include "HeadingAutopilot.h"
#include <numbers>

namespace
{
	MotorValues operator - (const MotorValues& l, const MotorValues& r)
	{
		return { l.angle - r.angle, l.speed - r.speed };
	}

	bool zeroDifference(const MotorValues& diff)
	{
		return diff.angle == 0.0 && diff.speed == 0.0 && diff.sway == 0.0;
	}

	std::pair<MotorValues, MotorValues> thrusterallocation(double X, double Y, double N) 
	{
		// Source: https://gitlab.abo.fi/mast/aboat-ros/-/blob/machine_setup_dev/ros_ws/src/heading_autopilot/src/heading_autopilot.cpp
		//Thruster allocation
		//konstants
		double Tmax = 24.95 * 9.81;  //motor max thrusts
		double l_m = 1.875;  //length from midpoint of ship to motor

		// Spliting target forces even between motors
		double X1 = X / 2;
		double X2 = X / 2;

		double Ys1 = Y / 2;
		double Ys2 = Y / 2;

		double N1 = N / 2;

		//Calculating torque to sway force
		double Yn1 = N1 / l_m;
		double Yn2 = N1 / (-l_m);
		// Calculating totalt sway force
		double Y1_tot = Ys1 + Yn1;
		double Y2_tot = Ys2 + Yn2;
		// Calculating motor angle
		double anglemotor1 = atan2(Y1_tot, X1) * (180 / std::numbers::pi);
		double anglemotor2 = atan2(Y2_tot, X2) * (180 / std::numbers::pi);
		//Calculating motor thrust
		double Thrust_motor1 = sqrt(X1 * X1 + Y1_tot * Y1_tot);
		double Thrust_motor2 = sqrt(X2 * X2 + Y2_tot * Y2_tot);
		//Calculating motor speed signal
		double Speed_motor1 = Thrust_motor1 / Tmax * 100;

		//cannot exceed 100
		if (Speed_motor1 > 100) {
			Speed_motor1 = 100;
		}
		double Speed_motor2 = Thrust_motor2 / Tmax * 100;

		//cannot exceed 100
		if (Speed_motor2 > 100) {
			Speed_motor2 = 100;
		}

		return { { anglemotor1, 20.0 /* Speed hardcoded */ }, { anglemotor2, 20.0 /* Speed hardcoded */ } };
	}
}

std::pair<MotorValues, MotorValues> HeadingAP::calculateMotorValues(const MotorValues& currentValues, const MotorValues& targetValues)
{
	MotorValues _curr = currentValues;
	MotorValues _target = targetValues;
	
	_curr.angle = _curr.angle * (std::numbers::pi / 180.0);
	_target.angle = _target.angle * (std::numbers::pi / 180.0);

	const MotorValues diff = _target - _curr;

	return thrusterallocation(1.0 /* Speed hardcoded */, diff.sway, diff.angle);
}
