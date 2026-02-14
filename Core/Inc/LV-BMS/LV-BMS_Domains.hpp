#ifndef LV_BMS_DOMAINS_HPP
#define LV_BMS_DOMAINS_HPP

#include "ST-LIB.hpp"
#include "LV-BMS_Data.hpp"

inline constexpr ST_LIB::TimerDomain::Timer timer_us_tick_def{{
  .request = ST_LIB::TimerRequest::GeneralPurpose32bit_5,
}};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput operational_led_def{ST_LIB::LED_OPERATIONAL};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput fault_led_def{ST_LIB::LED_FAULT};

static float adc_current_def;
inline constexpr ST_LIB::ADCDomain::ADC current_adc(
    ST_LIB::CURRENT_SENSOR, adc_current_def
);

#ifdef STLIB_ETH
#if defined(USE_PHY_LAN8742)
inline constexpr auto eth =
  ST_LIB::EthernetDomain::Ethernet(ST_LIB::EthernetDomain::PINSET_H10, LVBMS_MAC_ADDRESS, LVBMS_IP_ADDRESS, LVBMS_IP_MASK);
#elif defined(USE_PHY_LAN8700)
inline constexpr auto eth =
  ST_LIB::EthernetDomain::Ethernet(ST_LIB::EthernetDomain::PINSET_H10, LVBMS_MAC_ADDRESS, LVBMS_IP_ADDRESS, LVBMS_IP_MASK);
#elif defined(USE_PHY_KSZ8041)
inline constexpr auto eth =
  ST_LIB::EthernetDomain::Ethernet(ST_LIB::EthernetDomain::PINSET_H11, LVBMS_MAC_ADDRESS, LVBMS_IP_ADDRESS, LVBMS_IP_MASK);
#else
#error "No PHY selected for Ethernet pinset selection"
#endif
#endif // STLIB_ETH

#if STLIB_ETH
  using lvBMS_Board = ST_LIB::Board<eth, timer_us_tick_def, operational_led_def, fault_led_def, current_adc>;
#else
  using lvBMS_Board = ST_LIB::Board<timer_us_tick_def, operational_led_def, fault_led_def, current_adc>;
#endif

#endif // LV_BMS_DOMAINS_HPP