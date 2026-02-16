#ifndef LV_BMS_DOMAINS_HPP
#define LV_BMS_DOMAINS_HPP

#include "ST-LIB.hpp"
#include "LV-BMS_Data.hpp"

//////////////////////////////////////////////////////////
// Timers
//////////////////////////////////////////////////////////

inline constexpr ST_LIB::TimerDomain::Timer timer_us_tick_def{{
  .request = ST_LIB::TimerRequest::GeneralPurpose32bit_5,
}};

//////////////////////////////////////////////////////////
// LEDs
//////////////////////////////////////////////////////////

inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput operational_led_def{ST_LIB::LED_OPERATIONAL};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput fault_led_def{ST_LIB::LED_FAULT};

//////////////////////////////////////////////////////////
// ADC
//////////////////////////////////////////////////////////

static float adc_current_def;
inline constexpr ST_LIB::ADCDomain::ADC current_adc_def(
  ST_LIB::CURRENT_SENSOR, adc_current_def
);

//////////////////////////////////////////////////////////
// SPI
//////////////////////////////////////////////////////////

inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput spi_cs_def{ST_LIB::PD3};

static consteval ST_LIB::SPIDomain::SPIConfig get_spi_config() {
  ST_LIB::SPIDomain::SPIConfig conf {
    ST_LIB::SPIDomain::ClockPolarity::HIGH,
    ST_LIB::SPIDomain::ClockPhase::SECOND_EDGE,
    ST_LIB::SPIDomain::BitOrder::MSB_FIRST,
    ST_LIB::SPIDomain::NSSMode::SOFTWARE  // Manejamos CS manualmente
  };
  conf.data_size = ST_LIB::SPIDomain::DataSize::SIZE_8BIT;
  return conf;
}

inline constexpr auto spi_def = 
  ST_LIB::SPIDomain::Device<ST_LIB::DMA_Domain::Stream::dma1_stream5, 
                            ST_LIB::DMA_Domain::Stream::dma1_stream6>(
    ST_LIB::SPIDomain::SPIMode::MASTER, 
    ST_LIB::SPIDomain::SPIPeripheral::spi3, 1000000, 
    ST_LIB::PC10, ST_LIB::PC11, ST_LIB::PC12, get_spi_config());

//////////////////////////////////////////////////////////
// Ethernet
//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////
// Board
//////////////////////////////////////////////////////////

using lvBMS_Board = ST_LIB::Board<
#if STLIB_ETH
    eth, 
#endif
    timer_us_tick_def, 
    operational_led_def, 
    fault_led_def, 
    current_adc_def,
    spi_def,
    spi_cs_def>;

#endif // LV_BMS_DOMAINS_HPP