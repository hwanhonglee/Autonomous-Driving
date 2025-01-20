#ifndef PVD_VEHICLE_MODEL_HPP_
#define PVD_VEHICLE_MODEL_HPP_

#include <cmath>

namespace pvd_vehicle
{
namespace helper
{
namespace Vehicle
{
namespace Model
{
    inline double swa2rwa(const double steeringWheelAngle,
                          const double maxSteeringWheelAngle,
                          const double maxRoadWheelAngle)
    {
        return steeringWheelAngle / (maxSteeringWheelAngle / maxRoadWheelAngle);
    }

    inline double rwa2swa(const double roadWheelAngle,
                          const double maxSteeringWheelAngle,
                          const double maxRoadWheelAngle)
    {
        return roadWheelAngle * (maxSteeringWheelAngle / maxRoadWheelAngle);
    }

    inline double deg2rad(const double deg) { return deg * 3.14159265359 / 180.0; }
    inline double rad2deg(const double rad) { return rad * 180.0 / 3.1415; }
    inline double mps2kph(const double mps) { return mps * 3.60; }
    inline double kph2mps(const double kph) { return kph / 3.60; }
} // namespace Model
} // namespace Vehicle
} // namespace helper
} // namespace pvd_vehicle
#endif // PVD_VEHICLE_MODEL_HPP_