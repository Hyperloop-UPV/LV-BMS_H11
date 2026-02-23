#ifndef DATA_HPP
#define DATA_HPP

#include "BMS.hpp"
#include "ST-LIB.hpp"
#include "Communications/Packets/DataPackets.hpp"

#define N_BATTERIES 1
#define READING_PERIOD_US 17000
#define CAPACITY_AH 20.0f
#define REST_THRESHOLD 0.1f

#define RESISTANCE_REFERENCE 3900.0  // Ohm
#define VOLTAGE_REFERENCE 3.0        // V
#define R0 100.0                     // Ohm
#define TCR 0.00385

// network
#define LVBMS_MAC_ADDRESS "00:00:00:00:01:FE"
#define LVBMS_IP_ADDRESS "192.168.1.11"

#define USE_24BitMask 1
#if USE_24BitMask
#define LVBMS_IP_MASK "255.255.255.0"
#else
#define LVBMS_IP_MASK "255.255.0.0"
#endif

#endif // DATA_HPP