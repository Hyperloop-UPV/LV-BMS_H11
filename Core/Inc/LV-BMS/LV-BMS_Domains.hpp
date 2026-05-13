#ifndef LV_BMS_DOMAINS_HPP
#define LV_BMS_DOMAINS_HPP

#include "ST-LIB.hpp"
#include "LV-BMS_Data.hpp"
#include "LV-BMS_StateMachine.hpp"
#include "LV-BMS.hpp"

//////////////////////////////////////////////////////////
// Timers
//////////////////////////////////////////////////////////

inline constexpr ST_LIB::TimerDomain::Timer timer_us_tick_def{{
  .request = ST_LIB::TimerRequest::Any32bit,
}};

inline constexpr ST_LIB::TimerDomain::Timer timeout_timer_def{{
  .request = ST_LIB::TimerRequest::Any32bit,
}};

//////////////////////////////////////////////////////////
// LEDs
//////////////////////////////////////////////////////////

inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput operational_led_def{ST_LIB::LED_OPERATIONAL};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput fault_led_def{ST_LIB::LED_FAULT};

// Used to test LVBMS-H11 board
//inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput pg12_led_def{ST_LIB::PG12};

//////////////////////////////////////////////////////////
// ADC
//////////////////////////////////////////////////////////

#if LV_BMS_VERSION_MAJOR == 10

static float adc_current_def;
inline constexpr ST_LIB::ADCDomain::ADC current_adc_def(
  ST_LIB::CURRENT_SENSOR, adc_current_def
);

#endif

//////////////////////////////////////////////////////////
// SPI
//////////////////////////////////////////////////////////


#if LV_BMS_VERSION_MAJOR == 10
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

inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput spi_cs_def{ST_LIB::PD3};

inline constexpr auto spi_def =
  ST_LIB::SPIDomain::Device<ST_LIB::DMA_Domain::Stream::dma1_stream5, 
                            ST_LIB::DMA_Domain::Stream::dma1_stream6>(
    ST_LIB::SPIDomain::SPIMode::MASTER, 
    ST_LIB::SPIDomain::SPIPeripheral::spi3, 1000000, 
    ST_LIB::PC10, ST_LIB::PC11, ST_LIB::PC12, get_spi_config());

#elif LV_BMS_VERSION_MAJOR == 11
static consteval ST_LIB::SPIDomain::SPIConfig get_spi_config() {
  ST_LIB::SPIDomain::SPIConfig conf {
    ST_LIB::SPIDomain::ClockPolarity::LOW,
    ST_LIB::SPIDomain::ClockPhase::SECOND_EDGE,
    ST_LIB::SPIDomain::BitOrder::MSB_FIRST,
    ST_LIB::SPIDomain::NSSMode::SOFTWARE  // Manejamos CS manualmente
  };
  conf.data_size = ST_LIB::SPIDomain::DataSize::SIZE_8BIT;
  return conf;
}

inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput spi_cs_def{ST_LIB::PF6};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput bms_rst_def{ST_LIB::PF1};

inline constexpr auto spi_def =
  ST_LIB::SPIDomain::Device<ST_LIB::DMA_Domain::Stream::dma1_stream5, 
                            ST_LIB::DMA_Domain::Stream::dma1_stream6>(
    ST_LIB::SPIDomain::SPIMode::MASTER, 
    ST_LIB::SPIDomain::SPIPeripheral::spi5, 1000000, 
    ST_LIB::PF7, ST_LIB::PF8, ST_LIB::PF11, get_spi_config());

#endif

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
// Protections
//////////////////////////////////////////////////////////

inline constexpr auto soc_protection =
  Protections::protection<"SOC", LV_BMS::SOC>(
    Protections::Rules::range(20.0f, 80.0f, 25.0f, 75.0f)
  );

//////////////////////////////////////////////////////////
// Board
//////////////////////////////////////////////////////////

using lvBMS_faultpolicy =
  ST_LIB::FaultPolicy<LV_BMS_SM::State_Machine, FaultState_OnEnter>;

using lvBMS_Board = ST_LIB::Board<
  lvBMS_faultpolicy,
#if STLIB_ETH
  eth, 
#endif
  timer_us_tick_def, 
  //pg12_led_def,
  operational_led_def, 
  fault_led_def,
#if LV_BMS_VERSION_MAJOR == 10
  current_adc_def,
#elif LV_BMS_VERSION_MAJOR == 11
  timeout_timer_def,
#endif
    spi_def,
    spi_cs_def,
    bms_rst_def>;

#endif // LV_BMS_DOMAINS_HPP