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
 * @file nxp_helpers.c
 *
 * This module implements helpers function to facilitate common tasks.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nxp_helpers.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * Returns string with binary representation of value.
 * 
 * @param result Destination string for storing binary representation of value.
 * @return Returns pointer to destination string.
 */
char* uintToBinary(char result[], uint32_t value, uint8_t valueLength)
{
    uint16_t i;
    char binStr;
    uint32_t start = 1 << (valueLength - 1);
    
    result[0] = '\0';
    
    for (i = start; i > 0; i >>= 1) {
        binStr = (value & i) ? '1' : '0';
        strncat(result, &binStr, 1);
        // Append space after 4 bits
        if (i == 0x10000000 || i == 0x1000000 || i == 0x100000 || i == 0x10000 || 
                i == 0x1000 || i == 0x0100 || i == 0x0010) {
            strncat(result, " ", 1);
        }
    }
    return result;
}

char* uint16ToBinary(char result[20], uint16_t value)
{
    return uintToBinary(result, value, 16);
}

char* uint8ToBinary(char result[10], uint16_t value)
{
    return uintToBinary(result, value, 8);
}
