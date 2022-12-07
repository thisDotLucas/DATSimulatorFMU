#include "fmu-uuid.h"

#include <cppfmu_cs.hpp>

#include <cstring>
#include <stdexcept>

class NavigationService : public cppfmu::SlaveInstance
{
public:
    NavigationService() { NavigationService::Reset(); }

    void Reset() override
    {
        deltaLatitudeIn = 0.0;
        deltaLongitudeIn = 0.0;
        latitudeOut = 60.0;
        longitudeOut = 19.9993;
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override
    {
        for (std::size_t i = 0; i < nvr; i++)
        {
            if (vr[i] == 0)
                deltaLatitudeIn = value[i];
            else if (vr[i] == 1)
                deltaLongitudeIn = value[i];
            else if (vr[i] == 2)
                latitudeOut = value[i];
            else if (vr[i] == 3)
                longitudeOut = value[i];
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++)
        {
            if (vr[i] == 0)
                value[i] = deltaLatitudeIn;
            else if (vr[i] == 1)
                value[i] = deltaLongitudeIn;
            else if (vr[i] == 2)
                value[i] = latitudeOut;
            else if (vr[i] == 3)
                value[i] = longitudeOut;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
        cppfmu::FMIReal stepSize /*communicationStepSize*/,
        cppfmu::FMIBoolean /*newStep*/,
        cppfmu::FMIReal& /*endOfStep*/) override
    {
        latitudeOut += deltaLatitudeIn;
        longitudeOut += deltaLongitudeIn;

        return true;
    }

private:
    cppfmu::FMIReal deltaLatitudeIn;
    cppfmu::FMIReal deltaLongitudeIn;
    cppfmu::FMIReal latitudeOut;
    cppfmu::FMIReal longitudeOut;
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

    return cppfmu::AllocateUnique<NavigationService>(memory);
}
