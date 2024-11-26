#pragma once
#include <stdint.h>

enum class systemState
{
    ERROR = -1,
    RUNNING = 0,
    EMS = 1,
    ACK = 2
};

struct DataPlatform
{
    int32_t ticks_l;
    int32_t ticks_r;
    uint16_t voltage;
    uint16_t temp;
    systemState state;
    uint8_t reset_done;
};

struct DataROS
{
    double v_l;
    double v_r;
    uint8_t ACK;
    uint8_t encoder_reset;
};