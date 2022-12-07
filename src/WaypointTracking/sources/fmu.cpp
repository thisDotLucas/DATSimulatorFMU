#include "fmu-uuid.h"

#include <cppfmu_cs.hpp>

#include <cstring>
#include <stdexcept>
#include <array>

#include "Tracker.h"
#include <ranges>

std::vector<DecimalDegree> rectangle{ { 60.0001, 20.0 }, { 60.0001, 19.9993 }, { 60.000, 19.9993 }, { 60.000, 20.0000 } };

class WaypointTracking : public cppfmu::SlaveInstance
{
public:
    WaypointTracking() : m_tracker(rectangle) { WaypointTracking::Reset(); }

    void Reset() override 
    { 
        latitudeIn = 0.0;
        longitudeIn = 0.0;
        targetBoatHeadingOut = 0.0;
        targetBoatSpeedOut = 0.0;
        status = "";
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override 
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0) 
                latitudeIn = value[i];
            else if (vr[i] == 1)
                longitudeIn = value[i];
            else if (vr[i] == 2)
                targetBoatHeadingOut = value[i];
            else if (vr[i] == 3)
                targetBoatSpeedOut = value[i];
            else 
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++) 
        {
            if (vr[i] == 0)
                value[i] = latitudeIn;
            else if (vr[i] == 1)
                value[i] = longitudeIn;
            else if (vr[i] == 2)
                value[i] = targetBoatHeadingOut;
            else if (vr[i] == 3)
                value[i] = targetBoatSpeedOut;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override
    {
        for (std::size_t i = 0; i < nvr; i++)
        {
            if (vr[i] == 0)
                status = value[i];
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override
    {
        for (std::size_t i = 0; i < nvr; i++)
        {
            if (vr[i] == 0)
                value[i] = status;
            else
                throw std::logic_error("Invalid value reference");
        }
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/,
                cppfmu::FMIReal /*communicationStepSize*/,
                cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal & /*endOfStep*/) override 
    {
        auto [speed, heading] = m_tracker.track({ latitudeIn, longitudeIn });

        targetBoatSpeedOut = speed;
        targetBoatHeadingOut = heading;

        status = m_tracker.status({ latitudeIn, longitudeIn });

        return true;
    }

private:
    Tracker m_tracker;

    cppfmu::FMIReal latitudeIn;
    cppfmu::FMIReal longitudeIn;
    cppfmu::FMIReal targetBoatHeadingOut;
    cppfmu::FMIReal targetBoatSpeedOut;
    
    cppfmu::FMIString status;
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
    
    return cppfmu::AllocateUnique<WaypointTracking>(memory);
}
