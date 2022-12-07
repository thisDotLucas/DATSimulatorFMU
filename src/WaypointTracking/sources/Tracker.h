#include <vector>
#include <string>

struct DecimalDegree
{
    DecimalDegree(const double _latitude, const double _longitude) : latitude(_latitude), longitude(_longitude) {}

    double latitude;
    double longitude;
};

class Tracker
{
public:
    Tracker(std::vector<DecimalDegree> route) : m_route(route) {}

    std::pair<double, double> track(const DecimalDegree& currentCoordinate);
    const char* status(const DecimalDegree& currentCoordinate) const;

private:
    size_t m_targetCoordinate{};
    std::vector<DecimalDegree> m_route;
};