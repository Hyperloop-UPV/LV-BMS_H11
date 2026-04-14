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
#include "common.h"
#include "monitoring.h"

/*******************************************************************************
 * Global variable
 ******************************************************************************/

bcc_data_t g_bccData;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

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

/* Number of BCC configuration registers set during the initialization. */
#define MC33771C_INIT_CONF_REG_CNT     59U
#define MC33772C_INIT_CONF_REG_CNT     43U

/* Structure containing a register name and its address. */
typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const uint16_t value;
} bcc_init_reg_t;

/* Initial configuration of Battery Cell Controller devices.
 * INIT: Initialized by the BCC driver.
 * SYS_CFG_GLOBAL, SYS_DIAG and TPL_CFG: Kept POR values. */
static const bcc_init_reg_t s_initRegsMc33771c[MC33771C_INIT_CONF_REG_CNT] = {
    {MC33771C_GPIO_CFG1_OFFSET, MC33771C_GPIO_CFG1_POR_VAL, MC33771C_GPIO_CFG1_VALUE},
    {MC33771C_GPIO_CFG2_OFFSET, MC33771C_GPIO_CFG2_POR_VAL, MC33771C_GPIO_CFG2_VALUE},
    {MC33771C_TH_ALL_CT_OFFSET, MC33771C_TH_ALL_CT_POR_VAL, MC33771C_TH_ALL_CT_VALUE},
    {MC33771C_TH_CT14_OFFSET, MC33771C_TH_CT14_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT13_OFFSET, MC33771C_TH_CT13_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT12_OFFSET, MC33771C_TH_CT12_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT11_OFFSET, MC33771C_TH_CT11_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT10_OFFSET, MC33771C_TH_CT10_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT9_OFFSET, MC33771C_TH_CT9_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT8_OFFSET, MC33771C_TH_CT8_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT7_OFFSET, MC33771C_TH_CT7_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT6_OFFSET, MC33771C_TH_CT6_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT5_OFFSET, MC33771C_TH_CT5_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT4_OFFSET, MC33771C_TH_CT4_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT3_OFFSET, MC33771C_TH_CT3_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT2_OFFSET, MC33771C_TH_CT2_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT1_OFFSET, MC33771C_TH_CT1_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_AN6_OT_OFFSET, MC33771C_TH_AN6_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN5_OT_OFFSET, MC33771C_TH_AN5_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN4_OT_OFFSET, MC33771C_TH_AN4_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN3_OT_OFFSET, MC33771C_TH_AN3_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN2_OT_OFFSET, MC33771C_TH_AN2_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN1_OT_OFFSET, MC33771C_TH_AN1_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN0_OT_OFFSET, MC33771C_TH_AN0_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN6_UT_OFFSET, MC33771C_TH_AN6_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN5_UT_OFFSET, MC33771C_TH_AN5_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN4_UT_OFFSET, MC33771C_TH_AN4_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN3_UT_OFFSET, MC33771C_TH_AN3_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN2_UT_OFFSET, MC33771C_TH_AN2_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN1_UT_OFFSET, MC33771C_TH_AN1_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN0_UT_OFFSET, MC33771C_TH_AN0_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_ISENSE_OC_OFFSET, MC33771C_TH_ISENSE_OC_POR_VAL, MC33771C_TH_ISENSE_OC_VALUE},
    {MC33771C_TH_COULOMB_CNT_MSB_OFFSET, MC33771C_TH_COULOMB_CNT_MSB_POR_VAL, MC33771C_TH_COULOMB_CNT_MSB_VALUE},
    {MC33771C_TH_COULOMB_CNT_LSB_OFFSET, MC33771C_TH_COULOMB_CNT_LSB_POR_VAL, MC33771C_TH_COULOMB_CNT_LSB_VALUE},
    {MC33771C_CB1_CFG_OFFSET, MC33771C_CB1_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB2_CFG_OFFSET, MC33771C_CB2_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB3_CFG_OFFSET, MC33771C_CB3_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB4_CFG_OFFSET, MC33771C_CB4_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB5_CFG_OFFSET, MC33771C_CB5_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB6_CFG_OFFSET, MC33771C_CB6_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB7_CFG_OFFSET, MC33771C_CB7_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB8_CFG_OFFSET, MC33771C_CB8_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB9_CFG_OFFSET, MC33771C_CB9_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB10_CFG_OFFSET, MC33771C_CB10_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB11_CFG_OFFSET, MC33771C_CB11_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB12_CFG_OFFSET, MC33771C_CB12_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB13_CFG_OFFSET, MC33771C_CB13_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB14_CFG_OFFSET, MC33771C_CB14_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_OV_UV_EN_OFFSET, MC33771C_OV_UV_EN_POR_VAL, MC33771C_OV_UV_EN_VALUE},
    {MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_POR_VAL, MC33771C_SYS_CFG1_VALUE},
    {MC33771C_SYS_CFG2_OFFSET, MC33771C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
    {MC33771C_ADC_CFG_OFFSET, MC33771C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
    {MC33771C_ADC2_OFFSET_COMP_OFFSET, MC33771C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
    {MC33771C_FAULT_MASK1_OFFSET, MC33771C_FAULT_MASK1_POR_VAL, MC33771C_FAULT_MASK1_VALUE},
    {MC33771C_FAULT_MASK2_OFFSET, MC33771C_FAULT_MASK2_POR_VAL, MC33771C_FAULT_MASK2_VALUE},
    {MC33771C_FAULT_MASK3_OFFSET, MC33771C_FAULT_MASK3_POR_VAL, MC33771C_FAULT_MASK3_VALUE},
    {MC33771C_WAKEUP_MASK1_OFFSET, MC33771C_WAKEUP_MASK1_POR_VAL, MC33771C_WAKEUP_MASK1_VALUE},
    {MC33771C_WAKEUP_MASK2_OFFSET, MC33771C_WAKEUP_MASK2_POR_VAL, MC33771C_WAKEUP_MASK2_VALUE},
    {MC33771C_WAKEUP_MASK3_OFFSET, MC33771C_WAKEUP_MASK3_POR_VAL, MC33771C_WAKEUP_MASK3_VALUE},
};

static const bcc_init_reg_t s_initRegsMc33772c[MC33772C_INIT_CONF_REG_CNT] = {
    {MC33772C_GPIO_CFG1_OFFSET, MC33772C_GPIO_CFG1_POR_VAL, MC33772C_GPIO_CFG1_VALUE},
    {MC33772C_GPIO_CFG2_OFFSET, MC33772C_GPIO_CFG2_POR_VAL, MC33771C_GPIO_CFG2_VALUE},
    {MC33772C_TH_ALL_CT_OFFSET, MC33772C_TH_ALL_CT_POR_VAL, MC33771C_TH_ALL_CT_VALUE},
    {MC33772C_TH_CT6_OFFSET, MC33772C_TH_CT6_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_CT5_OFFSET, MC33772C_TH_CT5_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_CT4_OFFSET, MC33772C_TH_CT4_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_CT3_OFFSET, MC33772C_TH_CT3_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_CT2_OFFSET, MC33772C_TH_CT2_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_CT1_OFFSET, MC33772C_TH_CT1_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33772C_TH_AN6_OT_OFFSET, MC33772C_TH_AN6_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN5_OT_OFFSET, MC33772C_TH_AN5_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN4_OT_OFFSET, MC33772C_TH_AN4_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN3_OT_OFFSET, MC33772C_TH_AN3_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN2_OT_OFFSET, MC33772C_TH_AN2_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN1_OT_OFFSET, MC33772C_TH_AN1_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN0_OT_OFFSET, MC33772C_TH_AN0_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN6_UT_OFFSET, MC33772C_TH_AN6_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN5_UT_OFFSET, MC33772C_TH_AN5_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN4_UT_OFFSET, MC33772C_TH_AN4_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN3_UT_OFFSET, MC33772C_TH_AN3_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN2_UT_OFFSET, MC33772C_TH_AN2_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN1_UT_OFFSET, MC33772C_TH_AN1_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN0_UT_OFFSET, MC33772C_TH_AN0_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33772C_TH_ISENSE_OC_OFFSET, MC33772C_TH_ISENSE_OC_POR_VAL, MC33771C_TH_ISENSE_OC_VALUE},
    {MC33772C_TH_COULOMB_CNT_MSB_OFFSET, MC33772C_TH_COULOMB_CNT_MSB_POR_VAL, MC33771C_TH_COULOMB_CNT_MSB_VALUE},
    {MC33772C_TH_COULOMB_CNT_LSB_OFFSET, MC33772C_TH_COULOMB_CNT_LSB_POR_VAL, MC33771C_TH_COULOMB_CNT_LSB_VALUE},
    {MC33772C_CB1_CFG_OFFSET, MC33772C_CB1_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_CB2_CFG_OFFSET, MC33772C_CB2_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_CB3_CFG_OFFSET, MC33772C_CB3_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_CB4_CFG_OFFSET, MC33772C_CB4_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_CB5_CFG_OFFSET, MC33772C_CB5_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_CB6_CFG_OFFSET, MC33772C_CB6_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33772C_OV_UV_EN_OFFSET, MC33772C_OV_UV_EN_POR_VAL, MC33772C_OV_UV_EN_VALUE},
    {MC33772C_SYS_CFG1_OFFSET, MC33772C_SYS_CFG1_POR_VAL, MC33772C_SYS_CFG1_VALUE},
    {MC33772C_SYS_CFG2_OFFSET, MC33772C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
    {MC33772C_ADC_CFG_OFFSET, MC33772C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
    {MC33772C_ADC2_OFFSET_COMP_OFFSET, MC33772C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
    {MC33772C_FAULT_MASK1_OFFSET, MC33772C_FAULT_MASK1_POR_VAL, MC33771C_FAULT_MASK1_VALUE},
    {MC33772C_FAULT_MASK2_OFFSET, MC33772C_FAULT_MASK2_POR_VAL, MC33771C_FAULT_MASK2_VALUE},
    {MC33772C_FAULT_MASK3_OFFSET, MC33772C_FAULT_MASK3_POR_VAL, MC33772C_FAULT_MASK3_VALUE},
    {MC33772C_WAKEUP_MASK1_OFFSET, MC33772C_WAKEUP_MASK1_POR_VAL, MC33771C_WAKEUP_MASK1_VALUE},
    {MC33772C_WAKEUP_MASK2_OFFSET, MC33772C_WAKEUP_MASK2_POR_VAL, MC33771C_WAKEUP_MASK2_VALUE},
    {MC33772C_WAKEUP_MASK3_OFFSET, MC33772C_WAKEUP_MASK3_POR_VAL, MC33772C_WAKEUP_MASK3_VALUE},
};

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

static bcc_status_t initRegisters();
static bcc_status_t clearFaultRegs();
static void initDemo(status_t *error, bcc_status_t *bccError);
static bcc_status_t startApp(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*!
 * @brief Initializes BCC device registers according to BCC_INIT_CONF.
 * Registers having the wanted content already after POR are not rewritten.
 */
static bcc_status_t initRegisters()
{
    uint8_t cid, i;
    bcc_status_t status;

    for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
        if (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C)
        {
            for (i = 0; i < MC33771C_INIT_CONF_REG_CNT; i++)
            {
                if (s_initRegsMc33771c[i].value != s_initRegsMc33771c[i].defaultVal)
                {
                    status = BCC_Reg_Write(&g_bccData.drvConfig, (bcc_cid_t)cid,
                            s_initRegsMc33771c[i].address, s_initRegsMc33771c[i].value);
                    if (status != BCC_STATUS_SUCCESS)
                    {
                        return status;
                    }
                }
            }
        }
        else
        {
            for (i = 0; i < MC33772C_INIT_CONF_REG_CNT; i++)
            {
                if (s_initRegsMc33772c[i].value != s_initRegsMc33772c[i].defaultVal)
                {
                    status = BCC_Reg_Write(&g_bccData.drvConfig, (bcc_cid_t)cid,
                            s_initRegsMc33772c[i].address, s_initRegsMc33772c[i].value);
                    if (status != BCC_STATUS_SUCCESS)
                    {
                        return status;
                    }
                }
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
    uint8_t cid;
    bcc_status_t status;

    for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_OV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_UV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_COMM);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT1);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT2);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT3);
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
    ntc_config_t ntcConfig;
#ifdef TPL
    uint8_t i;
#endif

    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
            g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* Pin-muxing + GPIO pin direction and initial value settings. */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Initialize LPUART instance. */
    *error = DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_TYPE,
            &lpuart1_State, &lpuart1_InitConfig0);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

#ifdef TPL
    /* Initialize LPSPI instances. */
    *error = LPSPI_DRV_MasterInit(LPSPITPLTX, &lpspiTplTxState, &lpspiTplTx_MasterConfig);
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

    *error = LPSPI_DRV_SlaveInit(LPSPISPI_TPL1RX, &lpspiSpi_Tpl1RxState, &lpspiTpl1Rx_SlaveConfig);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }
#else
    /* Initialize LPSPI instance. */
    *error = LPSPI_DRV_MasterInit(LPSPISPI_TPL1RX, &lpspiSpi_Tpl1RxState, &lpspiSpi_MasterConfig);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }
#endif

    /* Initialize LPIT. */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);

    /* Initialize LPIT channel 3 and configure it as a periodic counter
     * which is used to generate an interrupt. */
    *error = LPIT_DRV_InitChannel(INST_LPIT1, 3U, &lpit1_ChnConfig3);
    if (*error != STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize BCC driver configuration structure (g_bccData.drvConfig). */
#ifdef TPL
    g_bccData.drvConfig.drvInstance = 0U;
    g_bccData.drvConfig.commMode = BCC_MODE_TPL;
    g_bccData.drvConfig.devicesCnt = BCC_DEMO_BCC_CNT;
    for (i = 0; i < BCC_DEMO_BCC_CNT; i++)
    {
    #ifdef MC33771C
        g_bccData.drvConfig.device[i] = BCC_DEVICE_MC33771C;
        g_bccData.drvConfig.cellCnt[i] = 14U;
    #else
        g_bccData.drvConfig.device[i] = BCC_DEVICE_MC33772C;
        g_bccData.drvConfig.cellCnt[i] = 6U;
    #endif
    }
    g_bccData.drvConfig.loopBack = false;
#else
    g_bccData.drvConfig.drvInstance = 0U;
    g_bccData.drvConfig.commMode = BCC_MODE_SPI;
    g_bccData.drvConfig.devicesCnt = 1U;
    #ifdef MC33771C
    g_bccData.drvConfig.device[0] = BCC_DEVICE_MC33771C;
    g_bccData.drvConfig.cellCnt[0] = 14U;
    #else
    g_bccData.drvConfig.device[0] = BCC_DEVICE_MC33772C;
    g_bccData.drvConfig.cellCnt[0] = 6U;
    #endif
#endif

    /* Precalculate NTC look up table for fast temperature measurement. */
    ntcConfig.rntc = 6800U;               /* NTC pull-up 6.8kOhm */
    ntcConfig.refTemp = 25U;              /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.refRes = 10000U;            /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.beta = 3900U;
    fillNtcTable(&ntcConfig);

    /* Initialize BCC device. */
    *bccError = BCC_Init(&g_bccData.drvConfig);
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize BCC device registers. */
    *bccError = initRegisters();
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Clear fault registers. */
    *bccError = clearFaultRegs();
}

/*!
 * @brief This function executes all prepared test cases.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t startApp(void)
{
    uint8_t cid;
    bcc_status_t error;

    for (cid = BCC_CID_DEV1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Print values of the configurable registers. */
        if ((error = printInitialSettings(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Do a measurement and print the measured values. */
        if ((error = doMeasurements(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Print content of the fault registers. */
        if ((error = printFaultRegisters(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    /* Send NOP command to all nodes in order to prevent communication timeout. */
    sendNops();

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
        PRINTF("------------- BEGIN ---------------\r\n");

        if ((bccError = startApp()) != BCC_STATUS_SUCCESS)
        {
            PRINTF("An error occurred (0x%04x)\r\n)", bccError);
            PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
        }

        bccError = BCC_Sleep(&g_bccData.drvConfig);
        if (bccError != BCC_STATUS_SUCCESS)
        {
            PRINTF("SLEEP (0x%04x)\r\n)", bccError);
            PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
        }

        PRINTF("-------------- END ----------------\r\n");
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
