#ifndef DATA_HPP
#define DATA_HPP

#include "BMS.hpp"
#include "ST-LIB.hpp"
#include "Communications/Packets/DataPackets.hpp"

#ifndef glue
#define glue_(a,b) a##b
#define glue(a,b) glue_(a,b)
#endif

#ifndef stringify
#define stringify_(x) #x
#define stringify(x) stringify_(x)
#endif

#define N_BATTERIES 1
#define READING_PERIOD_US 25000
#define CAPACITY_AH 20.0f
#define REST_THRESHOLD 0.1f

#define RESISTANCE_REFERENCE 3900.0  // Ohm
#define VOLTAGE_REFERENCE 3.0        // V
#define R0 100.0                     // Ohm
#define TCR 0.00385

#if !defined(LV_BMS_VERSION_MAJOR)
#if defined(USE_H11_CODE)
#define LV_BMS_VERSION_MAJOR 11
#else
#if defined(USE_PHY_KSZ8041)
#define LV_BMS_VERSION_MAJOR 11
#elif defined(USE_PHY_LAN8742) || defined(USE_PHY_LAN8700)
#define LV_BMS_VERSION_MAJOR 10
#else
#error Could not get LV_BMS version automatically from PHY used
#endif
#endif
#endif

#define LV_BMS_VERSION_CSTR glue("H", stringify(LV_BMS_VERSION_MAJOR))

// network
#define LVBMS_MAC_ADDRESS "00:00:00:00:01:FE"
#define LVBMS_IP_ADDRESS "192.168.1.11"

#define IP_BITMASK_BITS 24
#include "ip_bitmask.h"
#define LVBMS_IP_MASK IP_BITMASK_CSTR

#endif // DATA_HPP