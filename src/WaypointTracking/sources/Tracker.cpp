#include "Tracker.h"
#include <cmath>
#include <numbers>

namespace
{
    constexpr double pi() { return std::numbers::pi; }

    constexpr double toRadians(const double degrees) { return degrees * pi() / 180; }

	double distance(const GPSCoordinate& c1, const GPSCoordinate& c2)
	{
        // Calculates the distance between two points using the Haversine formula
        constexpr double RADIUS_OF_EARTH_METRES = 6371e3;

        const double phi1 = toRadians(c1.latitude);
        const double phi2 = toRadians(c2.latitude);

        const double delta_phi = toRadians(c2.latitude - c1.latitude);
        const double delta_lambda = toRadians(c2.longitude - c1.longitude);

        const double a = std::sin(delta_phi / 2) * std::sin(delta_phi / 2) + std::cos(phi1) * std::cos(phi2) * std::sin(delta_lambda / 2) * std::sin(delta_lambda / 2);
        const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return RADIUS_OF_EARTH_METRES * c;
	}

    double bearing(const GPSCoordinate& from, const GPSCoordinate& to)
    {
        const double phiA = toRadians(from.latitude);
        const double lambdaA = toRadians(from.longitude);
        const double phiB = toRadians(to.latitude);
        const double lambdaB = toRadians(to.longitude);

        // Bearing calculations
        const double Y = std::sin((lambdaB - lambdaA)) * cos(phiB);
        const double X = std::cos(phiA) * sin(phiB) - sin(phiA) * cos(phiB) * cos((lambdaB - lambdaA));

        return std::fmod((std::atan2(Y, X) * 180 / pi() + 360), 360);
    }
}

std::pair<double, double> Tracker::track(const GPSCoordinate& currentCoordinate)
{
    if (m_route.empty() || m_targetCoordinate >= m_route.size())
        return { 0.0, 0.0 }; // Route has not been defined or last traget ha been reached -> Stay still/Stop

    constexpr double TOLERANCE = 1.0; // metres
    if (distance(currentCoordinate, m_route.at(m_targetCoordinate)) < TOLERANCE)
        m_targetCoordinate++; // We have reached our current target -> Go to next target

    if (m_targetCoordinate >= m_route.size())
        return { 0.0, 0.0 }; // We have reached the last target -> Stop

    constexpr double SPEED = 20.0; // Hardcoded at 20% thrust
	return { SPEED, bearing(currentCoordinate, m_route.at(m_targetCoordinate)) };
}
