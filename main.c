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

// GAP INIT #DEFINES
#define DEVICE_NAME "MINIBEE"
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

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

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO 18 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS 0x10001080          /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

APP_TIMER_DEF(advertisingUpdateTimer);
#define ADVERTISING_CHANNEL 37
/**@brief Struct that contains pointers to the encoded advertising data. */
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

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // err_code = sd_ble_gap_device_name_set(&sec_mode,
    //                                       (const uint8_t *)DEVICE_NAME,
    //                                       strlen(DEVICE_NAME));
    // APP_ERROR_CHECK(err_code);

    // err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    // APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(void)
{
    uint32_t err_code;

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
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    NRF_LOG_INFO("error code: %d\n", err_code);
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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

/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
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
    static uint8_t shiftRegister = ADVERTISING_CHANNEL | (1 << 6);
    uint8_t whitenedByte = 0;
    uint8_t feedbackBit = 0;

    if (reset) { shiftRegister = ADVERTISING_CHANNEL | (1 << 6); }

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
       NRF_LOG_INFO("%02X ", whiten(0xFF, false)); // all ones
    }
}

// This function stretches data and loads it into m_adv_data.adv_data.p_data
static void queueStretchedData(uint8_t* data, uint8_t dataLength, uint8_t stretch)
{
    if ((dataLength * stretch) > 30) { NRF_LOG_INFO("Error, can't fit stretchedData into 30 bytes"); }

    // zero out advertising packet
    for (int i = 1; i < m_adv_data.adv_data.len; i++) { m_adv_data.adv_data.p_data[i] = 0; }

    uint8_t bitsInserted = 8;
    uint8_t bit = 0;
    for (int i = 0; i < dataLength; i++) {
        for (int b = 0; b < 8; b++) {
            bit = (data[i] & (0x80 >> b)) >> (7 - b);
            for (int j = 0; j < stretch; j++) {
                m_adv_data.adv_data.p_data[bitsInserted/8] |= bit << (7-(bitsInserted%8));
                bitsInserted++;
            }
        }
    }

    // debug
    for (int i = 0; i < dataLength; i++) { NRF_LOG_INFO("%02X", data[i]); }
    NRF_LOG_INFO("");
    for (int i = 0; i < m_adv_data.adv_data.len; i++) { NRF_LOG_INFO("%02X", m_adv_data.adv_data.p_data[i]); }
}

static void advertisingUpdateTimerHandler(void * p_context)
{
    // rotate shift register until at first data bit
    // length:2 address:6 payloadLength:1
    whiten(0, true); // reset shift register
    for (int i = 0; i < 8; i++) { whiten(0, false); }

    // payload
    m_adv_data.adv_data.p_data[0] = 0x1E; // length

    for (int i = 1; i < m_adv_data.adv_data.len; i++) {
        m_adv_data.adv_data.p_data[i] = whiten(0xFF, false); // all ones
    }

    // start advertising with m_adv_data and m_adv_params
    sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    advertising_init();

    // Timer
    ret_code_t err_code;
    err_code = app_timer_create(&advertisingUpdateTimer,
        APP_TIMER_MODE_REPEATED,
        advertisingUpdateTimerHandler);
    APP_ERROR_CHECK(err_code);

    app_timer_start(advertisingUpdateTimer, APP_TIMER_TICKS(200), NULL);

    // Start execution.
    NRF_LOG_INFO("MiniBee started.");

    // 30 / frequency divider
    // uint8_t dataLength = 15;
    // uint8_t data[15];
    // for (int i = 0; i < dataLength; i++) { data[i] = 0xAA; }

    queueStretchedData(data, dataLength, 30/dataLength);

    advertising_start();

    for (;;) {
        idle_state_handle();
    }
}
