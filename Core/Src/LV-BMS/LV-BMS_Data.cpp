#include "LV-BMS/LV-BMS_Data.hpp"

#include "LV-BMS/LV-BMS_Pinout.hpp"

//--------------- BMS CONFIG FOR LTC6810-DRIVER ----------------

void Data::BMSConfig::SPI_transmit(const span<uint8_t> data) {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  HAL_SPI_Transmit(spi->hspi, data.data(), data.size(), 10);
}

void Data::BMSConfig::SPI_receive(span<uint8_t> buffer) {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  HAL_SPI_Receive(spi->hspi, buffer.data(), buffer.size(), 10);
}

void Data::BMSConfig::SPI_CS_turn_off() {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  SPI::turn_off_chip_select(spi);
}

void Data::BMSConfig::SPI_CS_turn_on() {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  SPI::turn_on_chip_select(spi);
}

int32_t Data::BMSConfig::get_tick() { return Data::us_counter; }

//---------------------------------------------------------------

void Data::init() {
  current = new float;
  current_sensor =
    new LinearSensor<float>(CURRENT_SENSOR, 10.236f, -0.581f, current);

  LED_Operational = new DigitalOutput(LED_OPERATIONAL);
  LED_Fault = new DigitalOutput(LED_FAULT);

  BMSConfig::spi_id = SPI::inscribe(SPI::spi3);
}

void Data::start() {
  Time::register_high_precision_alarm(500, +[]() { ++us_counter; });

  last_reading_time = HAL_GetTick();
}

void Data::read_temperature(const float voltage, float* temperature) {
  auto resistance =
    (voltage * RESISTANCE_REFERENCE) / (VOLTAGE_REFERENCE - voltage);
  *temperature = (resistance - R0) / (TCR * R0);
}

float Data::coulomb_counting_SOC(float current) {
  uint32_t current_time = HAL_GetTick();

  float delta_time = (current_time - last_reading_time) / 1000.0f;
  last_reading_time = current_time;

  float delta_SOC = current * delta_time / CAPACITY_AH * 3600.0f;
  return delta_SOC;
}

float Data::ocv_battery_SOC() {
  float total_voltage = cells[0].get() + cells[1].get() + cells[2].get() +
                        cells[3].get() + cells[4].get() + cells[5].get();
  float x =
    total_voltage - 20.0;  // to get a bigger difference between values so
                           // the polynomial is more accurate
  float result = -62.5 + (14.9 * x) + (21.9 * x * x) + (-4.18 * x * x * x);
  return result;
}

void Data::update_SOC() {
  /* if (first_soc_flag == true) {
    SOC = ocv_battery_SOC();
    first_soc_flag = false;
  } else {
    if (std::abs(*current) < REST_THRESHOLD) {
      SOC += coulomb_counting_SOC(0 - *current);
    } else { */
      SOC = ocv_battery_SOC();
   /*  }
  } */
}

void Data::get_max_min_cells() {
  max_cell = *std::max_element(cells.begin(), cells.end());
  min_cell = *std::min_element(cells.begin(), cells.end());
}

void Data::get_max_min_temperatures() {
  max_temp = *std::max_element(temperature.begin(), temperature.end());
  min_temp = *std::min_element(temperature.begin(), temperature.end());
}

void Data::read() {
  bms.update();
  get_max_min_cells();
  current_sensor->read();
  update_SOC();
  read_temperature(GPIO_voltage_1, &temperature[0]);
  read_temperature(GPIO_voltage_2, &temperature[1]);
  read_temperature(GPIO_voltage_3, &temperature[2]);
  read_temperature(GPIO_voltage_4, &temperature[3]);
  get_max_min_temperatures();
}
