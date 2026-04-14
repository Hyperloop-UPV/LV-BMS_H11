/*
 * Copyright 2016 - 2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
** @file main.c
** @version 2.2
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"

  volatile int exit_code = 0;

/* User includes (#include below this line is not maintained by Processor Expert) */
#include "utils/nxp_console.h" /* PRINTF */
#include "bcc/bcc.h"
#include "bcc_s32k144/bcc_peripheries.h"

/*******************************************************************************
 * Definitions (depends on the utilized HW setup)
 ******************************************************************************/

/* Used BCC device */
#define MC33771C    /* Define MC33771C if RD33771CDSTEVB's are used. */
//#define MC33772C    /* Define MC33772C if KIT33772CTPLEVB's are used. */

#if !defined(MC33771C) && !defined(MC33772C)
    #error "Select used BCC device by defining MC33771C or MC33772C."
#endif

#if defined(MC33771C) && defined(MC33772C)
    #error "Select only one type of BCC device."
#endif

/* Number of MC3377xC devices connected in the chain. */
#define BCC_DEMO_BCC_CNT        2U

/*******************************************************************************
 * Global variables
 ******************************************************************************/

bcc_drv_config_t bccConfig;   /* BCC driver configuration. */
bool bccInitialized = false;  /* Indicates whether all BCCs in the chain are
                                 initialized. Only then is it possible to check
                                 data received from the loop-back TPL interface. */
bool loopBackOk = true;       /* True if data received from both MC33664
                                 is equivalent. */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Definition of power modes indexes, as configured in Power Manager Component
 * Refer to the Reference Manual for details about the power modes. */
#define RUN            0U /* Run */
#define HSRUN          1U /* High speed run */

/* Red on-board LED. */
#define RED_LED_PORT   PTD
#define RED_LED_PIN    15U

/* LPSPI_TX configuration. */
#define BCC_TX_LPSPI_DELAY_PCS_TO_SCLK         3U  /* 3us (f >= 1.75us) */
#define BCC_TX_LPSPI_DELAY_SCLK_TO_PCS         1U  /* 1us (g >= 0.60us) */
#define BCC_TX_LPSPI_DELAY_BETWEEN_TRANSFERS   5U  /* 5us (t_MCU_RES >= 4us) */

/*******************************************************************************
 * Initial BCC configuration
 ******************************************************************************/

/*! @brief  Number of MC33771 registers configured in the initialization with
 * user values. */
#define BCC_INIT_CONF_REG_CNT     5U

#define MC33771C_SYS_CFG1_VALUE ( \
    MC33771C_SYS_CFG1_CYCLIC_TIMER(MC33771C_SYS_CFG1_CYCLIC_TIMER_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_DIAG_TIMEOUT(MC33771C_SYS_CFG1_DIAG_TIMEOUT_1S_ENUM_VAL) | \
    MC33771C_SYS_CFG1_I_MEAS_EN(MC33771C_SYS_CFG1_I_MEAS_EN_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_CB_DRVEN(MC33771C_SYS_CFG1_CB_DRVEN_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_GO2DIAG(MC33771C_SYS_CFG1_GO2DIAG_EXIT_ENUM_VAL) | \
    MC33771C_SYS_CFG1_CB_MANUAL_PAUSE(MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_SOFT_RST(MC33771C_SYS_CFG1_SOFT_RST_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_FAULT_WAVE(MC33771C_SYS_CFG1_FAULT_WAVE_DISABLED_ENUM_VAL) | \
    MC33771C_SYS_CFG1_WAVE_DC_BITX(MC33771C_SYS_CFG1_WAVE_DC_BITX_500US_ENUM_VAL) \
)

/* Initial value of SYS_CFG2 register. */
#define MC33771C_SYS_CFG2_VALUE ( \
    MC33771C_SYS_CFG2_FLT_RST_CFG(MC33771C_SYS_CFG2_FLT_RST_CFG_OSC_MON_RESET_ENUM_VAL) | \
    MC33771C_SYS_CFG2_TIMEOUT_COMM(MC33771C_SYS_CFG2_TIMEOUT_COMM_256MS_ENUM_VAL) | \
    MC33771C_SYS_CFG2_NUMB_ODD(MC33771C_SYS_CFG2_NUMB_ODD_EVEN_ENUM_VAL) | \
    MC33771C_SYS_CFG2_HAMM_ENCOD(MC33771C_SYS_CFG2_HAMM_ENCOD_DECODE_ENUM_VAL) \
)

/* Initial value of ADC_CFG register. */
#define MC33771C_ADC_CFG_VALUE ( \
    MC33771C_ADC_CFG_AVG(MC33771C_ADC_CFG_AVG_NO_AVERAGING_ENUM_VAL) | \
    MC33771C_ADC_CFG_PGA_GAIN(MC33771C_ADC_CFG_PGA_GAIN_AUTO_ENUM_VAL) | \
    MC33771C_ADC_CFG_ADC1_A_DEF(MC33771C_ADC_CFG_ADC1_A_DEF_16_BIT_ENUM_VAL) | \
    MC33771C_ADC_CFG_ADC1_B_DEF(MC33771C_ADC_CFG_ADC1_B_DEF_16_BIT_ENUM_VAL) | \
    MC33771C_ADC_CFG_ADC2_DEF(MC33771C_ADC_CFG_ADC2_DEF_16_BIT_ENUM_VAL) \
)

/* Initial value of ADC2_OFFSET_COMP register. */
#define MC33771C_ADC2_OFFSET_COMP_VALUE (\
    MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG(MC33771C_ADC2_OFFSET_COMP_CC_RST_CFG_CC_RESET_ENUM_VAL) | \
    MC33771C_ADC2_OFFSET_COMP_FREE_CNT(MC33771C_ADC2_OFFSET_COMP_FREE_CNT_CLAMP_ENUM_VAL) | \
    MC33771C_ADC2_OFFSET_COMP_ALLCBOFFONSHORT(MC33771C_ADC2_OFFSET_COMP_ALLCBOFFONSHORT_SHORTED_ENUM_VAL) | \
    MC33771C_ADC2_OFFSET_COMP_ADC2_OFFSET_COMP(BCC_GET_ADC2_OFFSET(0)) \
)

/* Initial value of OV_UV_EN register.
 * Note: MC33772C does not have OV_UV_EN[CTx_OVUV], x = {7 .. 14}.
 *       However, they are set to '0' in the macro below --> The macro
 *       can be used also for MC33772C. */
#define MC33771C_OV_UV_EN_VALUE ( \
    MC33771C_OV_UV_EN_COMMON_UV_TH(MC33771C_OV_UV_EN_COMMON_UV_TH_COMMON_ENUM_VAL) | \
    MC33771C_OV_UV_EN_COMMON_OV_TH(MC33771C_OV_UV_EN_COMMON_OV_TH_COMMON_ENUM_VAL) | \
    /* Disable ADC data to be compared with thresholds for OV/UV. */ \
    MC33771C_OV_UV_EN_CT14_OVUV_EN(MC33771C_OV_UV_EN_CT14_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT13_OVUV_EN(MC33771C_OV_UV_EN_CT13_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT12_OVUV_EN(MC33771C_OV_UV_EN_CT12_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT11_OVUV_EN(MC33771C_OV_UV_EN_CT11_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT10_OVUV_EN(MC33771C_OV_UV_EN_CT10_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT9_OVUV_EN(MC33771C_OV_UV_EN_CT9_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT8_OVUV_EN(MC33771C_OV_UV_EN_CT8_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT7_OVUV_EN(MC33771C_OV_UV_EN_CT7_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT6_OVUV_EN(MC33771C_OV_UV_EN_CT6_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT5_OVUV_EN(MC33771C_OV_UV_EN_CT5_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT4_OVUV_EN(MC33771C_OV_UV_EN_CT4_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT3_OVUV_EN(MC33771C_OV_UV_EN_CT3_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT2_OVUV_EN(MC33771C_OV_UV_EN_CT2_OVUV_EN_DISABLED_ENUM_VAL) | \
    MC33771C_OV_UV_EN_CT1_OVUV_EN(MC33771C_OV_UV_EN_CT1_OVUV_EN_DISABLED_ENUM_VAL) \
)

/* Structure containing a register name and its address. */
typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const uint16_t value;
} bcc_init_reg_t;

/* Initial register configuration for MC33771C/MC33772C for this example. */
static const bcc_init_reg_t s_initRegs[BCC_INIT_CONF_REG_CNT] = {
    {MC33771C_OV_UV_EN_OFFSET, MC33771C_OV_UV_EN_POR_VAL, MC33771C_OV_UV_EN_VALUE},
    {MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_POR_VAL, MC33771C_SYS_CFG1_VALUE},
    {MC33771C_SYS_CFG2_OFFSET, MC33771C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
    {MC33771C_ADC_CFG_OFFSET, MC33771C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
    {MC33771C_ADC2_OFFSET_COMP_OFFSET, MC33771C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
};

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

static bcc_status_t initRegisters();
static bcc_status_t clearFaultRegs();
static void initDemo(status_t *error, bcc_status_t *bccError);
static uint8_t getVoltMeasurements(uint32_t measurements[2][15]);
static void doVoltMeasurements();
static bcc_status_t startApp(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*!
 * @brief Initializes all BCC device registers in the chain according to BCC_INIT_CONF.
 * Registers having the wanted content already after POR are not rewritten.
 */
static bcc_status_t initRegisters()
{
    bcc_status_t status;
    uint8_t i;

    for (i = 0; i < BCC_INIT_CONF_REG_CNT; i++)
    {
        if (s_initRegs[i].value != s_initRegs[i].defaultVal)
        {
            status = BCC_Reg_WriteGlobal(&bccConfig, s_initRegs[i].address, s_initRegs[i].value);
            if (status != BCC_STATUS_SUCCESS)
            {
                return status;
            }
        }
    }

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief Clears all fault registers of BCC devices.
 */
static bcc_status_t clearFaultRegs()
{
    bcc_status_t status;
    uint8_t cid;

    for (cid = 1; cid <= BCC_DEMO_BCC_CNT; cid++)
    {
        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_CELL_OV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_CELL_UV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_COMM);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_FAULT1);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_FAULT2);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&bccConfig, (bcc_cid_t)cid, BCC_FS_FAULT3);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }
    }

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief MCU and BCC initialization.
 */
static void initDemo(status_t *error, bcc_status_t *bccError)
{
    uint8_t i;

    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
            g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    /* Pin-muxing + GPIO pin direction and initial value settings. */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Initialize Power Manager
     * -   See PowerSettings component for more info
     */
    POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT,
            &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);
    *error = POWER_SYS_SetMode(HSRUN, POWER_MANAGER_POLICY_AGREEMENT);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize LPUART instance */
    *error = DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_TYPE,
            &lpuart1_State, &lpuart1_InitConfig0);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize LPSPI instance(s) */
    *error = LPSPI_DRV_MasterInit(LPSPITPLTX, &lpspiTplTxState, &lpspiTplTx_MasterConfig0);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    *error = LPSPI_DRV_MasterSetDelay(LPSPITPLTX, BCC_TX_LPSPI_DELAY_BETWEEN_TRANSFERS,
            BCC_TX_LPSPI_DELAY_SCLK_TO_PCS, BCC_TX_LPSPI_DELAY_PCS_TO_SCLK);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    *error = LPSPI_DRV_SlaveInit(LPSPITPL1RX, &lpspiTpl1RxState, &lpspiTpl1Rx_SlaveConfig0);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    *error = LPSPI_DRV_SlaveInit(LPSPITPL2RX, &lpspiTpl2RxState, &lpspiTpl2Rx_SlaveConfig0);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize LPIT. */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);

    /* Initialize LPIT channel 3 and configure it as a periodic counter
     * which is used to generate an interrupt. */
    *error = LPIT_DRV_InitChannel(INST_LPIT1, 3U, &lpit1_ChnConfig3);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    /* Enable second MC33664 device. */
    *bccError = BCC_TPL_Enable(1U);
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize BCC driver configuration structures.
     * Note: MC33664 is enabled automatically. */
    bccConfig.commMode = BCC_MODE_TPL;
    bccConfig.drvInstance = 0U;
    bccConfig.devicesCnt = BCC_DEMO_BCC_CNT;
    for (i = 0; i < BCC_DEMO_BCC_CNT; i++)
    {
#ifdef MC33771C
        bccConfig.device[i] = BCC_DEVICE_MC33771C;
        bccConfig.cellCnt[i] = 14U;
#else
        bccConfig.device[i] = BCC_DEVICE_MC33772C;
        bccConfig.cellCnt[i] = 6U;
#endif
    }
    bccConfig.loopBack = true;

    /* Initialize BCC devices. */
    *bccError = BCC_Init(&bccConfig);
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }
    bccInitialized = true;

    /* Initialize registers of BCC devices. */
    *bccError = initRegisters();
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Clear fault registers. */
    *bccError = clearFaultRegs();
}


/*!
 * @brief This function starts on-demand conversion and reads measured
 * stack and cell voltages.
 *
 * @param cid          Cluster Identification Address.
 * @param measurements [cid-1][0] Stack voltage in [uV].
 *                     [cid-1][1] Cell 1 voltage in [uV].
 *                     [cid-1][6/14] Cell 6/14 voltage in [uV].
 *
 * @return One in case of communication error with #CID=1 device,
 *         Two in case of communication error with #CID=2 device,
 *         Zero in case of no error. */
static uint8_t getVoltMeasurements(uint32_t measurements[BCC_DEMO_BCC_CNT][15])
{
    bool convCompl;
    uint8_t i;
    bcc_status_t status;

    /* Start conversion. */
    status = BCC_Meas_StartConversionGlobal(&bccConfig, MC33771C_ADC_CFG_VALUE);
    if (status != BCC_STATUS_SUCCESS)
    {
        return 1U;
    }

    /* Wait for completion. */
    BCC_MCU_WaitUs(600);

    /* Check the conversion is complete. */
    status = BCC_Meas_IsConverting(&bccConfig, BCC_CID_DEV1, &convCompl);
    if ((status != BCC_STATUS_SUCCESS) || (!convCompl))
    {
        return 1U;
    }

    for (i = 0; i < BCC_DEMO_BCC_CNT; i++)
    {
        /* Read measured values. */
        status = BCC_Meas_GetStackVoltage(&bccConfig, (bcc_cid_t)(i + 1U), &measurements[i][0]);
        if (status != BCC_STATUS_SUCCESS)
        {
            return i + 1U;
        }

        status = BCC_Meas_GetCellVoltages(&bccConfig, (bcc_cid_t)(i + 1U), &measurements[i][1]);
        if (status != BCC_STATUS_SUCCESS)
        {
            return i + 1U;
        }
    }

    return 0U;
}

/*!
 * @brief This function prints measures stack and cell voltages and prints
 * the results to serial console output.
 *
 * @return bcc_status_t Error code.
 */
static void doVoltMeasurements()
{
    uint32_t measurement[BCC_DEMO_BCC_CNT][15];
    uint8_t error, row, device;
    bool is771cInTheChain = false;

    error = getVoltMeasurements(measurement);

    /* Print header of the table. */
    PRINTF("          -");
    for (device = 1U; device <= BCC_DEMO_BCC_CNT; device++)
    {
        PRINTF("-----------");
    }
    PRINTF("\r\n          |");
    for (device = 1U; device <= BCC_DEMO_BCC_CNT; device++)
    {
        PRINTF("  CID #%u  |", device);
    }
    PRINTF("\r\n-----------");
    for (device = 1U; device <= BCC_DEMO_BCC_CNT; device++)
    {
        PRINTF("-----------");
    }
    PRINTF("\r\n");

    /* Print content of the table. */
    for (device = 0U; device < BCC_DEMO_BCC_CNT; device++)
    {
        if (bccConfig.device[device] == BCC_DEVICE_MC33771C)
        {
            is771cInTheChain = true;
            break;
        }
    }


    for (row = 0; row <= (is771cInTheChain ? MC33771C_MAX_CELLS : MC33772C_MAX_CELLS); row++)
    {
        /* Print header of the row. */
        if (row == 0)
        {
            PRINTF("| STACK   |");
        }
        else
        {
             PRINTF("| CELL %-2u |", row);
        }

        /* Print values. */
        for (device = 0; device < BCC_DEMO_BCC_CNT; device++)
        {
            if ((row > MC33772C_MAX_CELLS) && (bccConfig.device[device] == BCC_DEVICE_MC33772C))
            {
                PRINTF("          |");
            }
            else if ((error == 0U) || (error > (device + 1U)))
            {
                PRINTF(" %5d mV |", measurement[device][row] / 1000U);
            }
            else
            {
                PRINTF("      N/A |");
            }
        }

        PRINTF("\r\n");
    }

    PRINTF("-----------");
    for (device = 1U; device <= BCC_DEMO_BCC_CNT; device++)
    {
        PRINTF("-----------");
    }
    PRINTF("\r\n");

    /* Print footnotes in case of errors. */
    if (error)
    {
        PRINTF("An error in the communication with #CID=%u occurred!\r\n", error);
    }

    if (!loopBackOk)
    {
        PRINTF("Data received by the loop-back TPL doesn't correspond to the expected one!\r\n");
        loopBackOk = true;
    }
    PRINTF("\r\n");
}

/*!
 * @brief This function periodically wakes-up the BCC devices, does stack and
 * cell voltage measurements, prints the results to the serial console output
 * and moves BCC devices into the sleep mode.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t startApp(void)
{
    uint32_t i;

    /* Do a measurement and print the measured values. */
    while (1)
    {
        doVoltMeasurements();

        BCC_Sleep(&bccConfig);

        /* Wait a while.. */
        for (i = 0; i < 5000000; i++);

        BCC_WakeUp(&bccConfig);
    }

    return BCC_STATUS_SUCCESS;
}

/*! 
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */
  status_t error;
  bcc_status_t bccError;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    initDemo(&error, &bccError);
    if (error != STATUS_SUCCESS)
    {
        PRINTF("An error occurred during initialization of MCU peripherals.\r\n");
        PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
    }
    else if (bccError != BCC_STATUS_SUCCESS)
    {
        PRINTF("An error occurred during BCC initialization: (0x%04x)\r\n", bccError);
        PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
    }
    else
    {
        if ((bccError = startApp()) != BCC_STATUS_SUCCESS)
        {
            PRINTF("An error occurred during the BCC_S32K144_LoopBack example run (0x%04x).\r\n)", bccError);
            PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
        }
    }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP S32K series of microcontrollers.
**
** ###################################################################
*/
