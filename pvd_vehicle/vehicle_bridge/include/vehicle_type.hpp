#ifndef VEHICLE_TYPE_HPP_
#define VEHICLE_TYPE_HPP_

#include <cstdint>

namespace VehicleType {
    enum : uint16_t {
        NONE = 0x0000,
        UNKNOWN = 0x0001,
        SPECIAL = 0x0002,
        MOTO = 0x0003,
        CAR = 0x0004,
        CAR_OTHER = 0x0005,
        BUS = 0x0006,
        AXLE_CNT_2 = 0x0007,
        AXLE_CNT_3 = 0x0008,
        AXLE_CNT_4 = 0x0009,
        AXLE_CNT_4_TRAILER = 0x000a,
        AXLE_CNT_5_TRAILER = 0x000b,
        AXLE_CNT_6_TRAILER = 0x000c,
        AXLE_CNT_5_MULTI_TRAILER = 0x000d,
        AXLE_CNT_6_MULTI_TRAILER = 0x000e,
        AXLE_CNT_7_MULTI_TRAILER = 0x000f,
    };
}

#endif // VEHICLE_TYPE_HPP_
