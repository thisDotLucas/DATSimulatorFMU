#include <gtest/gtest.h>
#include "../src/HeadingAutopilot/sources/HeadingAutopilot.h"
#include "../src/WaypointTracking/sources/Tracker.h"

static const BoatSpecification Aboat
{
    // Source: https://gitlab.abo.fi/mast/aboat-ros/-/blob/machine_setup_dev/ros_ws/src/heading_autopilot/src/heading_autopilot.cpp
    .maxThrust = 24.95 * 9.81,
    .lengthFromMidPointToMotor = 1.875,
    .maxTurnRateRadianPerSec = 0.1,
    .motorMaxForceInSurgeAndSwayDirections = 300.0,
    .motorMinForceInSurgeAndSwayDirections = -300.0,
    .motorMaxTorque = 2 * 24.95 * 9.81 * 1.875, // 2 * maxThrust * lengthFromMidPointToMotor
    .motorMinTorque = -2 * 24.95 * 9.81 * 1.875 // -2 * maxThrust * lengthFromMidPointToMotor
};

TEST(HeadingAutopilot, test1)
{
    HeadingAP ap{ Aboat };
    ap.calculateMotorValues({ 270.0, 20.0 }, { 270.0, 20.01 });
    ASSERT_TRUE(0 == 0);
}

TEST(WaypointTracking, test1)
{
    std::vector<DecimalDegree> rectangle{ { 60.0001, 20.0 }, { 60.0001, 19.9993 }, { 60.000, 19.9993 }, { 60.000, 20.0 } };

    Tracker tracker{ rectangle };

    // 60.0001, 20.0 -> 60.0001, 19.9993
    const auto& [speed, angle] = tracker.track({ 60.0001, 20.0 });
    ASSERT_TRUE(std::round(angle) == 270.0);

    // 60.0001, 19.9993 -> 60.000, 19.9993
    const auto& [speed2, angle2] = tracker.track({ 60.0001, 19.9993 });
    ASSERT_TRUE(std::round(angle2) == 180.0);

    // 60.000, 19.9993 -> 60.000, 20.0000
    const auto& [speed3, angle3] = tracker.track({ 60.000, 19.9993 });
    ASSERT_TRUE(std::round(angle3) == 90.0);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}