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

		return { { anglemotor1, Speed_motor1 }, { anglemotor2, Speed_motor2 } };
	}
}

std::pair<MotorValues, MotorValues> HeadingAP::calculateMotorValues(const MotorValues& currentValues, const MotorValues& targetValues)
{
	const MotorValues diff = targetValues - currentValues;

	return zeroDifference(diff) ? std::pair<MotorValues, MotorValues>{ { 0.0, 0.0 }, { 0.0, 0.0 } } : 
		thrusterallocation(diff.speed, diff.sway, diff.angle);
}
