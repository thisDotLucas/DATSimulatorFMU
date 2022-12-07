#include <tuple>

struct BoatSpecification
{
	const double maxThrust;
	const double lengthFromMidPointToMotor;
	const double maxTurnRateRadianPerSec;
	const double motorMaxForceInSurgeAndSwayDirections;
	const double motorMinForceInSurgeAndSwayDirections;
	const double motorMaxTorque;
	const double motorMinTorque;
};

struct MotorValues
{
	MotorValues(const double _angle, const double _speed) : angle(_angle), speed(_speed) {}

	double angle;
	double speed;
	const double sway{ 0 }; // Always zero
};

class HeadingAP
{
public:
	HeadingAP(const BoatSpecification& boatSpecification) : m_boatSpecifications(boatSpecification) {}
	
	std::pair<MotorValues, MotorValues> calculateMotorValues(const MotorValues& currentValues, const MotorValues& targetValues);
private:
	BoatSpecification m_boatSpecifications;
};