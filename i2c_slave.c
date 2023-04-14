/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.

 */

/** @file
 *
 * WICED sample application for I2C slave
 *
 */
#ifdef SLAVE
#include "sparcommon.h"
#include "wiced_bt_dev.h"

#include "wiced_hal_gpio.h"
#include "wiced_timer.h"

#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#if !defined(CYW20706A2)
#include "cycfg_pins.h"
#endif
#ifdef BTSTACK_VER
#include "wiced_memory.h"
#include "bt_types.h"
#endif
#include "wiced_hal_i2c.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#ifdef BTSTACK_VER
#define BT_STACK_HEAP_SIZE          1024 * 6
wiced_bt_heap_t *p_default_heap = NULL;
#endif

#define I2C_REGISTER_WRITE_CMD                          0x0
#define I2C_REGISTER_READ_CMD                           0x1


/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
uint8_t i2c_registers[256];

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static void wiced_hal_i2c_slave_call_back(uint32_t evtFlag, uint8_t* rx_PktPointer, uint16_t rx_PktLength);

/* Entry point to the application. */

APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

#ifdef BTSTACK_VER
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
#endif

    WICED_BT_TRACE("**** I2C Slave Application **** \n\r");

    wiced_hal_i2c_slave_register_cb(&wiced_hal_i2c_slave_call_back);
#ifdef CYW989820M2EVB_01
    wiced_hal_i2c_slave_select_pads(WICED_P28, WICED_P29);
#else
#ifdef CYW20835B1
    wiced_hal_i2c_slave_select_pads(WICED_P34, WICED_P38);
#endif
#ifdef CYW20819A1
    wiced_hal_i2c_slave_select_pads(WICED_P26, WICED_P27);
#endif
#endif

    wiced_hal_i2c_slave_init();

}

void wiced_hal_i2c_slave_call_back(uint32_t evtFlag, uint8_t* rx_PktPointer, uint16_t rx_PktLength)
{
    uint16_t i = 0;

    switch(evtFlag)
    {
        case I2C_TRAN_EVENT_MASK_RX_DONE:
        case I2C_TRAN_EVENT_MASK_RX_STOP:

            WICED_BT_TRACE("RX event, received %d bytes ", rx_PktLength);
            for( i = 0; i < rx_PktLength; i++ )
            {
                WICED_BT_TRACE("0x%x ", rx_PktPointer[i]);
            }
            WICED_BT_TRACE("\n");

            if( (rx_PktPointer[0] == I2C_REGISTER_WRITE_CMD) && (rx_PktLength == 3) )
            {
                WICED_BT_TRACE("Write 0x%x to offset 0x%x\n", rx_PktPointer[2], rx_PktPointer[1]);
                i2c_registers[rx_PktPointer[1]] = rx_PktPointer[2];
            }
            else if( (rx_PktPointer[0] == I2C_REGISTER_READ_CMD) && (rx_PktLength == 2) )
            {
                WICED_BT_TRACE("Read 0x%x from 0x%x\n", i2c_registers[rx_PktPointer[1]], rx_PktPointer[1]);
                wiced_hal_i2c_slave_synchronous_write(&i2c_registers[rx_PktPointer[1]], 1);
            }
            else
            {
                WICED_BT_TRACE("Invalid command 0x%x and length 0x%x\n", rx_PktPointer[0], rx_PktPointer[1]);
            }
            break;
        case I2C_TRAN_EVENT_MASK_TX_DONE:
            break;
        default:
            break;
    }
}

#endif
