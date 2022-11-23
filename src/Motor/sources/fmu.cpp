#include "fmu-uuid.h"

#include <cppfmu_cs.hpp>

#include <cstring>
#include <stdexcept>
#include <cmath>

class Slave : public cppfmu::SlaveInstance 
{
public:
    Slave() { Slave::Reset(); }

    void Reset() override 
    { 
        angleIn = 0.0;
        torqueIn = 0.0;
        angleOut = 0.0;
        torqueOut = 0.0;
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override 
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0) 
                angleIn = value[i];
            else if (vr[i] == 1)
                torqueIn = value[i];
            else if (vr[i] == 2)
                angleOut = value[i];
            else if (vr[i] == 3)
                torqueOut = value[i];
            else 
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0)
                value[i] = angleIn;
            else if (vr[i] == 1)
                value[i] = torqueIn;
            else if (vr[i] == 2)
                value[i] = angleOut;
            else if (vr[i] == 3)
                value[i] = torqueOut;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
                cppfmu::FMIReal stepSize/*communicationStepSize*/,
                cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal & /*endOfStep*/) override 
    {
        const double angleChangePerStep = 1 * stepSize;
        if (std::abs(angleIn - angleOut) >= angleChangePerStep)
            angleOut += (angleIn >= angleOut ? 1 : -1) * angleChangePerStep;

        const double torqueChangePerStep = 1 * stepSize;
        if (std::abs(torqueIn - torqueOut) >= torqueChangePerStep)
            torqueOut += (torqueIn >= torqueOut ? 1 : -1) * torqueChangePerStep;

        return true;
    }

private:
    cppfmu::FMIReal angleIn;
    cppfmu::FMIReal torqueIn;
    cppfmu::FMIReal angleOut;
    cppfmu::FMIReal torqueOut;
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

    return cppfmu::AllocateUnique<Slave>(memory);
}
