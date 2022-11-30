#include "fmu-uuid.h"

#include <cppfmu_cs.hpp>

#include <cstring>
#include <stdexcept>

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
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
                cppfmu::FMIReal /*communicationStepSize*/,
                cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal & /*endOfStep*/) override 
    {
        motorFrontAngleOut = motorFrontAngleIn;
        motorFrontSpeedOut = motorFrontSpeedIn;
        motorRearAngleOut = motorRearAngleIn;
        motorRearSpeedOut = motorRearSpeedIn;

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
