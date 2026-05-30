#define BCC_STLIB_IMPLEMENTATION
#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_StateMachine.hpp"
#include "LV-BMS/LV-BMS_Domains.hpp"

#if LV_BMS_VERSION_MAJOR == 11
#include "LV-BMS/BCC_ST-LIB/bcc_stlib.h"
#endif

#include <float.h>

ST_LIB::DigitalOutputDomain::Instance *spi_cs;
ST_LIB::DigitalOutputDomain::Instance *bms_rst;
ST_LIB::SPIDomain::SPIWrapper<spi_def> *spi_wrapper;

#if LV_BMS_VERSION_MAJOR == 10
void LV_BMS::BMSConfig::SPI_transmit(const span<uint8_t> data) {
  spi_wrapper->send(data);
}
void LV_BMS::BMSConfig::SPI_receive(span<uint8_t> buffer) {
  spi_wrapper->receive(buffer);
}

void LV_BMS::BMSConfig::SPI_CS_turn_off() {
  spi_cs->turn_off();
}
void LV_BMS::BMSConfig::SPI_CS_turn_on() {
  spi_cs->turn_on();
}

int32_t LV_BMS::BMSConfig::get_tick() {
  return GetMicroseconds();
}
#elif LV_BMS_VERSION_MAJOR == 11

static bcc_status_t Init_BCC_Registers()
{
  bcc_status_t status;

  for(uint8_t cid = 1; cid <= LV_BMS::bcc_config.devicesCnt; cid++) {
    for(uint8_t i = 0; i < (uint8_t)ARRAY_LENGTH(bcc_init_regs); i++) {
      if(bcc_init_regs[i].value != bcc_init_regs[i].defaultVal) {
        status = BCC_Reg_Write(&LV_BMS::bcc_config, (bcc_cid_t)cid,
                    bcc_init_regs[i].address, bcc_init_regs[i].value);
        if(status != BCC_STATUS_SUCCESS) {
          return status;
        }
      }
    }
  }

  return BCC_STATUS_SUCCESS;
}

static bcc_status_t Clear_BCC_FaultRegisters()
{
  bcc_status_t status;

  for(uint8_t cid = 1; cid <= LV_BMS::bcc_config.devicesCnt; cid++) {
    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CELL_OV);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CELL_UV);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_COMM);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT1);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT2);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT3);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }
  }

  return BCC_STATUS_SUCCESS;
}

static inline bool bcc_start_measurements() {
  bcc_status_t status = 
    BCC_Meas_StartConversion(&LV_BMS::bcc_config, (bcc_cid_t)1, BCC_AVG_1);
  if(status != BCC_STATUS_SUCCESS) {
    WARNING("Could not start bcc measurements: %s", get_bcc_error_str(status));
    return false;
  }
  return true;
}

static void Init_BCC_Driver()
{
  LV_BMS::bcc_config.drvInstance = 0U;
  LV_BMS::bcc_config.commMode = BCC_MODE_SPI;
  LV_BMS::bcc_config.devicesCnt = 1U;
  LV_BMS::bcc_config.device[0] = BCC_DEVICE_MC33772C;
  LV_BMS::bcc_config.cellCnt[0] = 6U;
  bcc_status_t status = BCC_Init(&LV_BMS::bcc_config);
  if(status != BCC_STATUS_SUCCESS) {
    FAULT("Could not init BCC: %s", get_bcc_error_str(status));
    return;
  }

  status = Init_BCC_Registers();
  if(status != BCC_STATUS_SUCCESS) {
    FAULT("Could not init BCC registers: %s", 
                 get_bcc_error_str(status));
    return;
  }

  status = Clear_BCC_FaultRegisters();
  if(status != BCC_STATUS_SUCCESS) {
    FAULT("Could not clear BCC fault registers: %s", 
                 get_bcc_error_str(status));
    return;
  }

  uint64_t guid;
  status = BCC_GUID_Read(&LV_BMS::bcc_config, (bcc_cid_t)1, &guid);
  if(status != BCC_STATUS_SUCCESS) {
    WARNING("Could not read device guid: %s", get_bcc_error_str(status));
    return;
  } else {
    INFO("BCC device guid: %02X%04X%04X",
        (uint16_t)((guid >> 32) & 0x001FU),
        (uint16_t)((guid >> 16) & 0xFFFFU),
        (uint16_t)(guid & 0xFFFFU));
  }

  bcc_start_measurements();
}

static void bcc_get_measurements() {
  uint16_t measurements[BCC_MEAS_CNT];
  bcc_status_t status;

  bool completed = false;
  // NOTE: For HV-BMS (@JorgeCanut), probably make a round robin with the bcc_cids
  status = BCC_Meas_IsConverting(&LV_BMS::bcc_config, (bcc_cid_t)1, &completed);
  if(!completed) {
    // Wait until next call
    return;
  }

  status = BCC_Meas_GetRawValues(&LV_BMS::bcc_config, (bcc_cid_t)1, measurements);
  if(status != BCC_STATUS_SUCCESS) {
    WARNING("Could not get bcc measurements: %s", get_bcc_error_str(status));
    return;
  }

  // TODO: Store these measurements

  /* Content of CC registers (raw values only).
   * CC registers resets on read. */
  int32_t coulomb_counter = 
    BCC_GET_COULOMB_CNT(measurements[BCC_MSR_COULOMB_CNT1],
                        measurements[BCC_MSR_COULOMB_CNT2]);

#if 0
  // NOTE: We don't use isense for LV-BMS
  int32_t isense_microvolts =
    BCC_GET_ISENSE_VOLT(measurements[BCC_MSR_ISENSE1],
                        measurements[BCC_MSR_ISENSE2]);
  int32_t isense_miliamps =
    BCC_GET_ISENSE_AMP(LV_BMS_RSHUNT_RESISTANCE, measurements[BCC_MSR_ISENSE1],
                       measurements[BCC_MSR_ISENSE2]);

  (void)isense_microvolts;
  (void)isense_miliamps;
#endif

  /* In micro volts */
  uint32_t stack_voltage = BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]);

  uint32_t cell1_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1]);
  uint32_t cell2_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT2]);
  uint32_t cell3_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT3]);
  uint32_t cell4_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT4]);
  uint32_t cell5_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT5]);
  uint32_t cell6_voltage = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT6]);

  uint16_t analog1 = measurements[BCC_MSR_AN1];
  //uint16_t analog2 = measurements[BCC_MSR_AN2]; /* I don't think this one is useful for MC33772C (?) */
  uint16_t analog3 = measurements[BCC_MSR_AN3];
  uint16_t analog4 = measurements[BCC_MSR_AN4];
  uint16_t analog5 = measurements[BCC_MSR_AN5];
  uint16_t analog6 = measurements[BCC_MSR_AN6];

  /* IC temperature measurement in ºC*10 */
  int32_t temp_times_ten = BCC_GET_IC_TEMP_C(measurements[BCC_MSR_ICTEMP]);

  /* ADCIA and ADCIB Band Gap Reference measurements, in micro volts */
  uint32_t ADCIA_v = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1A]);
  uint32_t ADCIB_v = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1B]);

  (void)coulomb_counter;
  (void)stack_voltage;

  (void)cell1_voltage;
  (void)cell2_voltage;
  (void)cell3_voltage;
  (void)cell4_voltage;
  (void)cell5_voltage;
  (void)cell6_voltage;

  (void)analog1;
  //(void)analog2;
  (void)analog3;
  (void)analog4;
  (void)analog5;
  (void)analog6;

  (void)temp_times_ten;
  (void)ADCIA_v;
  (void)ADCIB_v;
}

// end LV_BMS_VERSION_MAJOR == 11
#elif LV_BMS_VERSION_MAJOR == 10

static void read_temperature(const float voltage, float* temperature) {
  auto resistance =
    (voltage * RESISTANCE_REFERENCE) / (VOLTAGE_REFERENCE - voltage);
  *temperature = (resistance - R0) / (TCR * R0);
}

#endif

//---------------------------------------------------------------

// connecting
void ConnectingState_Update(void)
{
  LV_BMS::operational_led->toggle();
}

// operational
void OperationalState_OnEnter(void)
{
  LV_BMS::operational_led->turn_on();
}

void OperationalState_OnExit(void)
{
  LV_BMS::operational_led->turn_off();
}

// fault
void FaultState_OnEnter(void)
{
  LV_BMS::fault_led->turn_on();
}

// NOTE: This is to be implemented, there is no fault recovery yet!
//void FaultState_OnExit(void)
//{
//  LV_BMS::fault_led->turn_off();
//}


//---------------------------------------------------------------

void LV_BMS::init() {
  /* Comms init */ {
    DataPackets::Battery_Voltages_init(
      battery[0].cells[0], battery[0].cells[1], battery[0].cells[2],
      battery[0].cells[3], battery[0].cells[4], battery[0].cells[5], 
      min_cell, max_cell, total_voltage);

    DataPackets::Battery_Temperatures_init(
      temperature[0], temperature[1], 
      temperature[2], temperature[3], 
      min_temperature, max_temperature);

    DataPackets::State_of_Charge_init(SOC);
    DataPackets::Battery_Current_init(current);

    DataPackets::Current_State_init(state);
  }
  DataPackets::start();

  OrderPackets::Brake_init();
  OrderPackets::start();

#if LV_BMS_VERSION_MAJOR == 10
  Scheduler::register_task((READING_PERIOD_US / 2) - 100, []() {
    bms.update();
  });
#elif LV_BMS_VERSION_MAJOR == 11
  Init_BCC_Driver();
#endif

  //LV_BMS::add_protections();
  //DCLV::init();
}

void LV_BMS::start() {
  last_reading_time = HAL_GetTick();

  FaultController::start();
  Scheduler::register_task(1000, []() {
    BMS_State prev_state = LV_BMS::state;
    FaultController::check_transitions();
    if(FaultController::is_faulted()) {
      LV_BMS::state = BMS_State::FAULT;
    } else {
      LV_BMS::state = LV_BMS_SM::State_Machine.get_current_state();
    }
    if(LV_BMS::state != prev_state) [[unlikely]] {
#if STLIB_ETH
      DataPackets::control_station_udp->send_packet(*DataPackets::Current_State_packet);
#endif
    }
  });

  Scheduler::register_task(100'000, []() {
    read();
  });
}

//------------------- SOC ------------------------

float LV_BMS::coulomb_counting_SOC(float current) {
  uint32_t current_time = HAL_GetTick();

  float delta_time = (current_time - last_reading_time) / 1000.0f;
  last_reading_time = current_time;

  float delta_SOC = current * delta_time / CAPACITY_AH * 3600.0f;
  return delta_SOC;
}

float LV_BMS::ocv_battery_SOC() {
  float total_voltage = battery[0].cells[0] + battery[0].cells[1] + battery[0].cells[2] +
                        battery[0].cells[3] + battery[0].cells[4] + battery[0].cells[5];
  float x =
    total_voltage - 20.0;  // to get a bigger difference between values so
                           // the polynomial is more accurate
  float result = -62.5 + (14.9 * x) + (21.9 * x * x) + (-4.18 * x * x * x);
  return result;
}

void LV_BMS::update_SOC() {
  /* if(first_soc_flag == true) {
    SOC = ocv_battery_SOC();
    first_soc_flag = false;
  } else {
    if(std::abs(*current) < REST_THRESHOLD) {
      SOC += coulomb_counting_SOC(0 - *current);
    } else { */
      SOC = ocv_battery_SOC();
#if LV_BMS_VERSION_MAJOR == 10
      if((SOC > -100.0f) && (SOC < 200.0f) && ((SOC < 20.0f) || (SOC > 80.0f))) [[unlikely]] {
        FAULT("Fault: LV battery State of charge not in range [20, 80]");
        LV_BMS_SM::State_Machine.force_change_state(static_cast<size_t>(BMS_State::FAULT));
      }
#endif
   /*  }
  } */
}

//------------------------------------------------

void LV_BMS::get_max_min_cells() {
  float maximum = FLT_MIN;
  float minimum = FLT_MAX;
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::battery[0].cells); i++) {
    float v = LV_BMS::battery[0].cells[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }

  max_cell = maximum;
  min_cell = minimum;
}

void LV_BMS::get_max_min_temperatures() {
  float maximum = FLT_MIN;
  float minimum = FLT_MAX;
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::temperature); i++) {
    float v = LV_BMS::temperature[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }

  max_temperature = maximum;
  min_temperature = minimum;
}

void LV_BMS::read() {
  update_SOC();
#if LV_BMS_VERSION_MAJOR == 10
  current_sensor.read();
  read_temperature(GPIO_voltage_1, &temperature[0]);
  read_temperature(GPIO_voltage_2, &temperature[1]);
  read_temperature(GPIO_voltage_3, &temperature[2]);
  read_temperature(GPIO_voltage_4, &temperature[3]);
#elif LV_BMS_VERSION_MAJOR == 11
  bcc_get_measurements();
#endif
  get_max_min_cells();
  get_max_min_temperatures();
}
