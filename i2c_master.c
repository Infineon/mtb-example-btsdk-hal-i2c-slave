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
 * WICED sample application for I2C master
 *
 */
#ifdef MASTER
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

#define I2C_MASTER_STATE_IDLE                           0
#define I2C_MASTER_STATE_WRITE_DONE                     1
#define I2C_MASTER_STATE_READ_COMMAND_DONE              2
#define I2C_MASTER_TX_INTERVAL_TIME                     500     // 500ms
#define I2C_SLAVE_ADDRESS                               0x77
#define I2C_REGISTER_WRITE_CMD                          0x0
#define I2C_REGISTER_READ_CMD                           0x1

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
extern void utilslib_delayUs(uint32_t delay);
static wiced_timer_t hal_i2c_app_timer;
static void hal_i2c_app_timer_cb(TIMER_PARAM_TYPE arg);

/* Entry point to the application. */

APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

#ifdef BTSTACK_VER
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
#endif
    WICED_BT_TRACE("**** I2C Master test app **** \n\r");

    /*Initialize I2C and set speed to 400kHz */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    // Initialize timer to control the pin toggle frequency
    wiced_init_timer(&hal_i2c_app_timer, &hal_i2c_app_timer_cb, 0, WICED_MILLI_SECONDS_TIMER);
    wiced_start_timer(&hal_i2c_app_timer, I2C_MASTER_TX_INTERVAL_TIME);
}

/*
 * The function invoked on timeout of app. seconds timer.
 */
void hal_i2c_app_timer_cb(TIMER_PARAM_TYPE arg)
{
    static uint32_t i2c_master_state = I2C_MASTER_STATE_IDLE;
    static uint8_t offset = 0;
    static uint8_t data = 0;

    uint8_t buffer[3];

    switch(i2c_master_state)
    {
        case I2C_MASTER_STATE_IDLE:
            buffer[0] = I2C_REGISTER_WRITE_CMD;   // command
            buffer[1] = offset;                   // register offset
            buffer[2] = data;                     // value
            wiced_hal_i2c_write(buffer, 3, I2C_SLAVE_ADDRESS);
            i2c_master_state = I2C_MASTER_STATE_WRITE_DONE;
            wiced_start_timer(&hal_i2c_app_timer, I2C_MASTER_TX_INTERVAL_TIME);
            break;
        case I2C_MASTER_STATE_WRITE_DONE:
            buffer[0] = I2C_REGISTER_READ_CMD;    // command
            buffer[1] = offset;                 // register offset
            wiced_hal_i2c_write(buffer, 2, I2C_SLAVE_ADDRESS);
            i2c_master_state = I2C_MASTER_STATE_READ_COMMAND_DONE;
            wiced_start_timer(&hal_i2c_app_timer, I2C_MASTER_TX_INTERVAL_TIME);
            break;
        case I2C_MASTER_STATE_READ_COMMAND_DONE:
            wiced_hal_i2c_read(buffer, 1, I2C_SLAVE_ADDRESS);
            i2c_master_state = I2C_MASTER_STATE_IDLE;
            if(buffer[0] != data)
            {
                WICED_BT_TRACE("Error!!!! offset 0x%x, data 0x%x expected, but received 0x%x\n", offset, data, buffer[0]);
            }
            else
            {
                WICED_BT_TRACE("Offset 0x%x, Data 0x%x is matched\n", offset, data);
                wiced_start_timer(&hal_i2c_app_timer, I2C_MASTER_TX_INTERVAL_TIME);
                offset++;
                data++;
            }
            break;
        default:
            WICED_BT_TRACE("Invalid state 0x%x\n", i2c_master_state);
            break;
    }
}

#endif
