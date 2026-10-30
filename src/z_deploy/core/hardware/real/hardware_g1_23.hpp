#pragma once

#include <hardware/real/hardware_g1.hpp>

// G1 23-DOF uses the same HG protocol as G1 29-DOF. Its base.yaml order is
// compact, but SDK motor IDs retain gaps for the absent waist/wrist joints.
class HardwareG1_23 : public HardwareG1
{
public:
    explicit HardwareG1_23(const RobotConfig &config)
        : HardwareG1(config,
                     {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                      15, 16, 17, 18, 19, 22, 23, 24, 25, 26})
    {
    }
};

Z_REGISTER_HARDWARE("real", "g1_23", HardwareG1_23)
