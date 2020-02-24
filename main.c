/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_srv_common.h"
#include "ble_hci.h"

#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

// GAP INIT #DEFINES
#define DEVICE_NAME "MINIBEE"
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(20, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
// #define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(1000, UNIT_10_MS)

#define APP_BEACON_INFO_LENGTH 0x17   /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH 0x15      /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE 0x02          /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI 0xC3        /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER 0x0059 /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE 0x01, 0x02    /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE 0x03, 0x04    /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID 0x01, 0x12, 0x23, 0x34, \
                        0x45, 0x56, 0x67, 0x78, \
                        0x89, 0x9a, 0xab, 0xbc, \
                        0xcd, 0xde, 0xef, 0xf0 /**< Proprietary UUID for Beacon. */

//Radio transmit power in dBm 
//(accepted values are -40, -20, -16, -12, -8, -4, 0, and 4 dBm)
#define TX_POWER -40

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO 18 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS 0x10001080          /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

APP_TIMER_DEF(advertisingUpdateTimer);
static uint8_t advertisingChannel = 37;

static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
        .scan_rsp_data =
            {
                .p_data = NULL,
                .len = 0

            }};

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void advertising_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr = NULL; // Undirected advertisement.
    m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    m_adv_params.primary_phy = BLE_GAP_PHY_AUTO;
    m_adv_params.secondary_phy = BLE_GAP_PHY_AUTO;
    m_adv_params.channel_mask[4] = 0xC0; // advertise on 37 only
    // m_adv_params.channel_mask[4] = 0xA0; // advertise on 38 only
    // m_adv_params.channel_mask[4] = 0x60; // advertise on 39 only

    // Data
    m_adv_data.adv_data.p_data[0] = 0x1E; // length
    for (int i = 1; i < m_adv_data.adv_data.len; i++) { m_adv_data.adv_data.p_data[i] = 0x0; }

    // start advertising with m_adv_data and m_adv_params
    sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
}

void advertising_start(void)
{
    ret_code_t err_code;
    
    // set power level     
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, TX_POWER);
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

uint8_t reverseByte(uint8_t byte)
{
    uint8_t reversedBits = 0;

    for (uint8_t i = 0; i < 4; i++) {
        reversedBits |= (byte >> (7-(2*i))) & (0x01 << i);
        reversedBits |= (byte << (7-(2*i))) & (0x80 >> i);
    }

    return reversedBits;
}

static uint8_t whiten(uint8_t byte, bool reset)
{
    static uint8_t shiftRegister = 37 | (1 << 6);
    uint8_t whitenedByte = 0;
    uint8_t feedbackBit = 0;

    if (reset) { shiftRegister = advertisingChannel | (1 << 6); }

    for (int i = 0; i < 8; i++) {
        whitenedByte |= ((byte & 0x1) ^ (shiftRegister & 0x1)) << i;
        byte = byte >> 1;
        feedbackBit = shiftRegister & 0x1;
        shiftRegister = shiftRegister >> 1; // rotate right

        // clear 0th bit and set it to feedback bit
        shiftRegister = (shiftRegister & ~(1 << 6)) | (feedbackBit << 6);

        // clear 4th bit and xor with feedback bit
        shiftRegister = (shiftRegister & ~(1 << 2)) | ((((shiftRegister >> 2) & 0x1) ^ feedbackBit) << 2);
    }

    return whitenedByte;
}

void testWhitener()
{
    // test whitener
    whiten(0, true); // reset shift register
    for (int i = 0; i < 8; i++) { whiten(0, false); } // rotate shift register until at first data bit
    for (int i = 1; i < m_adv_data.adv_data.len; i++) {
    //    NRF_LOG_INFO("%02X ", whiten(0xFF, false)); // all ones
    }
}

static uint8_t stretchedData[31] = {0};

// This function stretches data and loads it into m_adv_data.adv_data.p_data
static void queueStretchedData(uint8_t* data, uint8_t dataLength, uint8_t stretch)
{
    // if ((dataLength * stretch) > 30) { NRF_LOG_INFO("Error, can't fit stretchedData into 30 bytes"); }

    uint8_t bitsInserted = 8;
    uint8_t bit = 0;
    for (int i = 0; i < dataLength; i++) {
        for (int b = 0; b < 8; b++) {
            bit = (data[i] & (0x80 >> b)) >> (7 - b);
            for (int j = 0; j < stretch; j++) {
                stretchedData[bitsInserted/8] |= bit << (7-(bitsInserted%8));
                bitsInserted++;
            }
        }
    }
}

#define FREQUENCY_DIVIDER 30

#define START_WAIT_TIME_MS 19 // 500uA average current
// #define STOP_WAIT_TIME_MS 1
#define STOP_WAIT_TIME_MS 275

static void advertisingUpdateTimerHandler(void * p_context)
{
    static bool restart = true;

    if (restart) {
        // rotate shift register until at first data bit
        // length:2 address:6 payloadLength:1
        /*advertisingChannel = 37 + (((advertisingChannel+1) % 37) % 3);

        if (advertisingChannel == 37) { m_adv_params.channel_mask[4] = 0xC0; }
        else if (advertisingChannel == 38) { m_adv_params.channel_mask[4] = 0xA0; }
        else if (advertisingChannel == 39) { m_adv_params.channel_mask[4] = 0x60; }
*/
        whiten(0, true); // reset shift register
        for (int i = 0; i < 8; i++) { whiten(0, false); }

        // payload
        m_adv_data.adv_data.p_data[0] = 0x1E; // length

        // load desired data at (1Mb / frequencyDivider) into data
        uint8_t data[30] = {0};
        uint8_t dataLength = 30 / FREQUENCY_DIVIDER;
        for (int i = 0; i < dataLength; i++) { data[i] = i+0xA1; }
        queueStretchedData(data, dataLength, 30/dataLength);

        // copy stretchedData into actual advertising packet
        for (int i = 1; i < m_adv_data.adv_data.len; i++) {
            m_adv_data.adv_data.p_data[i] = whiten(reverseByte(stretchedData[i]), false); // all ones
        }

        // start advertising with m_adv_data and m_adv_params
        sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        
        advertising_start();
    } else {
        sd_ble_gap_adv_stop(m_adv_handle);
    }

    app_timer_start(advertisingUpdateTimer, APP_TIMER_TICKS(restart ? START_WAIT_TIME_MS : STOP_WAIT_TIME_MS), NULL);
    restart = !restart;
}

int main(void)
{
    // log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    advertising_init();
    sd_power_dcdc_mode_set(true);

    // Timer
    ret_code_t err_code;
    err_code = app_timer_create(&advertisingUpdateTimer,
        APP_TIMER_MODE_SINGLE_SHOT,
        advertisingUpdateTimerHandler);
    APP_ERROR_CHECK(err_code);
    app_timer_start(advertisingUpdateTimer, APP_TIMER_TICKS(5000), NULL);

    for (;;) {
        idle_state_handle();
    }
}


// Scratchpad
// Start execution.
// NRF_LOG_INFO("MiniBee started");
// NRF_LOG_INFO("Last modified 12.17.2019");
// NRF_LOG_INFO("Expected:");
// uint8_t data[30] = {0};
// uint8_t dataLength = 30 / FREQUENCY_DIVIDER;
// for (int i = 0; i < dataLength; i++) { data[i] = i+0xA1; }
// queueStretchedData(data, dataLength, 30/dataLength);
// for (int i = 1; i < 31; i++) { NRF_LOG_INFO("%02X", reverseByte(stretchedData[i])); }

// Use this one to print out and generate packets in MATLAB
// for (int i = 1; i < 31; i++) {
    // NRF_LOG_INFO("%02X", (stretchedData[i]));
// }

