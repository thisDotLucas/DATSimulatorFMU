#include "fmu-uuid.h"
#include "HeadingAutopilot.h"

#include <cppfmu_cs.hpp>
#include <cstring>
#include <stdexcept>

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

class HeadingAutopilot : public cppfmu::SlaveInstance
{
public:
    HeadingAutopilot() : m_headingAutopilot(Aboat) { HeadingAutopilot::Reset(); }

    void Reset() override 
    { 
        targetBoatHeadingIn = 0.0;
        targetBoatSpeedIn = 0.0;
        surgeIn = 0.0;
        swayIn = 0.0;
        yawRateIn = 0.0;
        motorRearHeadingOut = 0.0;
        motorRearSpeedOut = 0.0;
        motorFrontHeadingOut = 0.0;
        motorFrontSpeedOut = 0.0;
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override 
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0)
                targetBoatHeadingIn = value[i];
            else if (vr[i] == 1)
                targetBoatSpeedIn = value[i];
            else if (vr[i] == 2)
                surgeIn = value[i];
            else if (vr[i] == 3)
                swayIn = value[i];
            else if (vr[i] == 4)
                yawRateIn = value[i];
            else if (vr[i] == 5)
                motorRearHeadingOut = value[i];
            else if (vr[i] == 6)
                motorRearSpeedOut = value[i];
            else if (vr[i] == 7)
                motorFrontHeadingOut = value[i];
            else if (vr[i] == 8)
                motorFrontSpeedOut = value[i];
            else 
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0)
                value[i] = targetBoatHeadingIn;
            else if (vr[i] == 1)
                value[i] = targetBoatSpeedIn;
            else if (vr[i] == 2)
                value[i] = surgeIn;
            else if (vr[i] == 3)
                value[i] = swayIn;
            else if (vr[i] == 4)
                value[i] = yawRateIn;
            else if (vr[i] == 5)
                value[i] = motorRearHeadingOut;
            else if (vr[i] == 6)
                value[i] = motorRearSpeedOut;
            else if (vr[i] == 7)
                value[i] = motorFrontHeadingOut;
            else if (vr[i] == 8)
                value[i] = motorFrontSpeedOut;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
                cppfmu::FMIReal /*communicationStepSize*/,
                cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal & /*endOfStep*/) override 
    {
        const auto& [motorFront, motorRear] = m_headingAutopilot.calculateMotorValues({ yawRateIn, surgeIn }, { targetBoatHeadingIn, targetBoatSpeedIn });

        return true;
    }

private:
    HeadingAP m_headingAutopilot;

    cppfmu::FMIReal targetBoatHeadingIn;
    cppfmu::FMIReal targetBoatSpeedIn;
    cppfmu::FMIReal surgeIn;
    cppfmu::FMIReal swayIn;
    cppfmu::FMIReal yawRateIn;
    cppfmu::FMIReal motorRearHeadingOut;
    cppfmu::FMIReal motorRearSpeedOut;
    cppfmu::FMIReal motorFrontHeadingOut;
    cppfmu::FMIReal motorFrontSpeedOut;
};

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString fmuGUID,
    cppfmu::FMIString /*fmuResourceLocation*/, cppfmu::FMIString /*mimeType*/,
    cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory,
    cppfmu::Logger /*logger*/) 
{
    if (std::strcmp(fmuGUID, FMU_UUID) != 0)
        throw std::runtime_error("FMU GUID mismatch");

    return cppfmu::AllocateUnique<HeadingAutopilot>(memory);
}
