#include "fmu-uuid.h"

#include <cppfmu_cs.hpp>

#include <cstring>
#include <stdexcept>
#include <numbers>

class MotorController : public cppfmu::SlaveInstance 
{
public:
    MotorController() { MotorController::Reset(); }

    void Reset() override 
    { 
        motorFrontAngleIn = 0.0;
        motorFrontSpeedIn = 0.0;
        motorRearAngleIn = 0.0;
        motorRearSpeedIn = 0.0;
        motorFrontAngleOut = 0.0;
        motorFrontSpeedOut = 0.0;
        motorRearAngleOut = 0.0;
        motorRearSpeedOut = 0.0;
        deltaLatitudeOut = 0.0;
        deltaLongitudeOut = 0.0;
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override 
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0) 
                motorFrontAngleIn = value[i];
            else if (vr[i] == 1)
                motorFrontSpeedIn = value[i];
            else if (vr[i] == 2)
                motorRearAngleIn = value[i];
            else if (vr[i] == 3)
                motorRearSpeedIn = value[i];
            else if (vr[i] == 4)
                motorFrontAngleOut = value[i];
            else if (vr[i] == 5)
                motorFrontSpeedOut = value[i];
            else if (vr[i] == 6)
                motorRearAngleOut = value[i];
            else if (vr[i] == 7)
                motorRearSpeedOut = value[i];
            else if (vr[i] == 8)
                deltaLatitudeOut = value[i];
            else if (vr[i] == 9)
                deltaLongitudeOut = value[i];
            else if (vr[i] == 10)
                heading = value[i];
            else 
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0)
                value[i] = motorFrontAngleIn;
            else if (vr[i] == 1)
                value[i] = motorFrontSpeedIn;
            else if (vr[i] == 2)
                value[i] = motorRearAngleIn;
            else if (vr[i] == 3)
                value[i] = motorRearSpeedIn;
            else if (vr[i] == 4)
                value[i] = motorFrontAngleOut;
            else if (vr[i] == 5)
                value[i] = motorFrontSpeedOut;
            else if (vr[i] == 6)
                value[i] = motorRearAngleOut;
            else if (vr[i] == 7)
                value[i] = motorRearSpeedOut;
            else if (vr[i] == 8)
                value[i] = deltaLatitudeOut;
            else if (vr[i] == 9)
                value[i] = deltaLongitudeOut;
            else if (vr[i] == 10)
                value[i] = heading;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
                cppfmu::FMIReal stepSize /*communicationStepSize*/,
                cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal & /*endOfStep*/) override 
    {
        motorFrontAngleOut = motorFrontAngleIn;
        motorFrontSpeedOut = motorFrontSpeedIn;
        motorRearAngleOut = motorRearAngleIn;
        motorRearSpeedOut = motorRearSpeedIn;

        // 0.00001 is 1.1 m 
        const double maxVelocity = 0.00002 /* per second */ * stepSize; // Arbitrary

        deltaLatitudeOut = (motorFrontSpeedOut / 100) * maxVelocity * cos(heading * (std::numbers::pi / 180));
        deltaLongitudeOut = (motorFrontSpeedOut / 100) * maxVelocity * sin(heading * (std::numbers::pi / 180));

        return true;
    }

private:
    cppfmu::FMIReal motorFrontAngleIn;
    cppfmu::FMIReal motorFrontSpeedIn;
    cppfmu::FMIReal motorRearAngleIn;
    cppfmu::FMIReal motorRearSpeedIn;
    cppfmu::FMIReal motorFrontAngleOut;
    cppfmu::FMIReal motorFrontSpeedOut;
    cppfmu::FMIReal motorRearAngleOut;
    cppfmu::FMIReal motorRearSpeedOut;

    cppfmu::FMIReal deltaLatitudeOut;
    cppfmu::FMIReal deltaLongitudeOut;
    cppfmu::FMIReal heading;
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

    return cppfmu::AllocateUnique<MotorController>(memory);
}
