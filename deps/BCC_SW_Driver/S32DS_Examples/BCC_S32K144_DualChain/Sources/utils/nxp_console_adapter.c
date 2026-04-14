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
 * @file  nxp_console_adapter.c
 * @brief This module implements adapter functions needed to get running KSDK
 *        debug console code in the S32 SDK environment.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "nxp_console_adapter.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! Table of base addresses for LPUART instances. */
static LPUART_Type* const g_lpuartBases[] = LPUART_BASE_PTRS;

/*******************************************************************************
 * Code - internal functions
 ******************************************************************************/
 
/*******************************************************************************
 * Code - public functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_GetInstance
 * Description   : Gets instance for LPUART module base pointer.
 *
 *END**************************************************************************/
uint32_t LPUART_GetInstance(void *lpuartBase)
{
    DEV_ASSERT(lpuartBase != NULL);

    uint32_t instance;

    LPUART_Type *base = (LPUART_Type *)lpuartBase;

    /* Find the instance index from base address mappings. */
    for (instance = 0U; instance < LPUART_INSTANCE_COUNT; instance++)
    {
        if (g_lpuartBases[instance] == base)
        {
            break;
        }
    }

    DEV_ASSERT(instance < LPUART_INSTANCE_COUNT);

    return instance;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_WriteBlocking
 * Description   : Implements adapter function which translates S32 SDK signature
 *                 format into KSDK 2.0 signature. It accesses LPUART low level
 *                 function used for sending data.
 *
 *END**************************************************************************/
void LPUART_WriteBlocking(LPUART_Type *base, const uint8_t *buffer, size_t length)
{
    /* We don't care about return value. */
    LPUART_DRV_SendDataBlocking(LPUART_GetInstance(base), buffer, \
            (uint32_t)length, XFER_TIMEOUT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_ReadBlocking
 * Description   : Implements adapter function which translates S32 SDK signature
 *                 format into KSDK 2.0 signature. It accesses LPUART low level
 *                 function used for reading data.
 *
 *END**************************************************************************/
status_t LPUART_ReadBlocking(LPUART_Type *base, uint8_t *buffer, size_t length)
{
    status_t status;

    status = LPUART_DRV_ReceiveDataBlocking(LPUART_GetInstance(base), buffer, \
            (uint32_t)length, XFER_TIMEOUT);

    return (status != STATUS_SUCCESS) ? STATUS_ERROR : STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
