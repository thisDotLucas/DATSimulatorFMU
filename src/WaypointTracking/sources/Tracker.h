#include <vector>


struct GPSCoordinate
{
    GPSCoordinate(const double _latitude, const double _longitude) : latitude(_latitude), longitude(_longitude) {}

    double latitude;
    double longitude;
};

class Tracker
{
public:
    Tracker(std::vector<GPSCoordinate> route) : m_route(route) {}

    std::pair<double, double> track(const GPSCoordinate& currentCoordinate);

private:
    size_t m_targetCoordinate{};
    std::vector<GPSCoordinate> m_route;
};