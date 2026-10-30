#pragma once

#include "hardware_g1.hpp"

class G1_23HardwareReceiver : public G1HardwareReceiver
{
public:
    explicit G1_23HardwareReceiver(const RobotConfig &config)
        : G1HardwareReceiver(config, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                                      15, 16, 17, 18, 19, 22, 23, 24, 25, 26})
    {
    }
};
