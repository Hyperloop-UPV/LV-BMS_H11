/*
 * Copyright (c) 2016, NXP B.V.
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
 * o Neither the name of  NXP B.V. nor the names of its
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
 * @file  nxp_console_adapter.h
 * @brief This module implements adapter functions needed to get running KSDK
 *        debug console code in the S32 SDK environment.
 */
#ifndef _NXP_CONSOLE_ADAPTER_H_
#define _NXP_CONSOLE_ADAPTER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>
#include <stdint.h>

#include "lpuart_driver.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* Debug console type definition. */
/* Note that this adapter implements only LPUART to be used for debug console !*/
#define DEBUG_CONSOLE_DEVICE_TYPE_NONE 0U   /*!< No debug console.             */
#define DEBUG_CONSOLE_DEVICE_TYPE_UART 1U   /*!< Debug console base on UART.   */
#define DEBUG_CONSOLE_DEVICE_TYPE_LPUART 2U /*!< Debug console base on LPUART. */
#define DEBUG_CONSOLE_DEVICE_TYPE_LPSCI 3U  /*!< Debug console base on LPSCI.  */
#define DEBUG_CONSOLE_DEVICE_TYPE_USBCDC 4U /*!< Debug console base on USBCDC. */

/* Number of LPUART instances on the S32K144. */
#define FSL_FEATURE_SOC_LPUART_COUNT LPUART_INSTANCE_COUNT

/* Timeout to complete blocking transfer. */
#define XFER_TIMEOUT 1000U

/* The LPUART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_LPUART
/* Based address of the LPUART instance used for virtual serial port. */
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) LPUART1

/* Definition of types to retain compatibility.  */
typedef long long int64_t;
typedef unsigned long long uint64_t;

/*!
 * @addtogroup fsl_console_adapter_enum_group
 * Module enumerations.
 * @{
 */

 /*! @} */
 
/*!
 * @addtogroup fsl_console_adapter_struct_group
 * Module structures.
 * @{
 */

/*! @} */
 
/*******************************************************************************
 * Global variables
 ******************************************************************************/
/*!
 * @addtogroup fsl_console_adapter_globals_group
 * Module global variables.
 * @{
 */

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup fsl_console_adapter_functions_group
 * Module global functions.
 * @{
 */

/*!
 * @brief Gets instance for LPUART module base pointer.
 *
 * @parma lpuartBase LPUART base pointer.
 *
 * @return Instance matching given base pointer.
 */
uint32_t LPUART_GetInstance(void *lpuartBase);

/*!
 * @brief Implements adapter function for writing data to LPUART.
 *
 * Implements adapter function which translates S32 SDK signature
 * format into KSDK 2.0 signature. It accesses LPUART low level
 * function used for sending data.
 *
 * @param base      LPUART peripheral base address.
 * @param data      Start address of the data to write.
 * @param length    Size of the data to write.
 *
 * @return              None.
 */
void LPUART_WriteBlocking(LPUART_Type *base, const uint8_t *buffer, size_t length);


/*!
 * @brief Implements adapter function for reading data from LPUART.
 *
 * Implements adapter function which translates S32 SDK signature
 * format into KSDK 2.0 signature. It accesses LPUART low level
 * function used for reading data.
 *
 * @param base      LPUART peripheral base address.
 * @param data      Start address of the buffer to store the received data.
 * @param length    Size of the buffer.
 *
 * @return          Whether the reading was successful or not.
 * @retval kStatus_Success          Execution successfully
 * @retval kStatus_Fail             Execution failure
 */
status_t LPUART_ReadBlocking(LPUART_Type *base, uint8_t *buffer, size_t length);

/*! @} */

#endif /* _NXP_CONSOLE_ADAPTER_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
