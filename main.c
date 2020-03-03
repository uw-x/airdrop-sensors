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

#include "nrf_ble_scan.h"
#include "nrf_fstorage.h"
#include "nrf_sdh_soc.h"

// GAP INIT #DEFINES
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)              /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)              /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                                 /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)                /**< Connection supervisory timeout (4 seconds). */

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< SoC observer priority of the application. There is no need to modify this value. */

#define NON_CONNECTABLE_ADV_INTERVAL MSEC_TO_UNITS(20, UNIT_0_625_MS)   /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define TX_POWER 4                                                      /** (accepted values are -40, -20, -16, -12, -8, -4, 0, and 4 dBm) */

#define SCAN_INTERVAL               0x0320                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0320                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION           	0x0000

static ble_gap_adv_params_t m_adv_params;                               /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static bool    m_memory_access_in_progress;   /**< Flag to keep track of ongoing operations on persistent memory. */
static uint8_t advertisingChannel = 37;

APP_TIMER_DEF(advertisingUpdateTimer);
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static ble_gap_adv_data_t m_adv_data =
{
    .adv_data = {
        .p_data = m_enc_advdata,
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
    .scan_rsp_data = {
        .p_data = NULL,
        .len = 0 }
};

static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x00,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = 0,
    .channel_mask  = {0},
};

#define MY_TIMER            NRF_TIMER1
#define MY_TIMER_IRQn       TIMER1_IRQn
#define MY_TIMER_IRQHandler TIMER1_IRQHandler

static uint32_t my_timer_seconds;

static void my_timer_start(void)
{
    // Reset the second variable
    my_timer_seconds = 0;

    // Ensure the timer uses 24-bit bitmode or higher
    MY_TIMER->BITMODE = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos;

    // Set the prescaler to 4, for a timer interval of 1 us (16M / 2^4)
    MY_TIMER->PRESCALER = 4;

    // Set the CC[0] register to hit after 1 second
    MY_TIMER->CC[0] = 1000000;

    // Make sure the timer clears after reaching CC[0]
    MY_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    // Trigger the interrupt when reaching CC[0]
    MY_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

    // Set a low IRQ priority and enable interrupts for the timer module
    NVIC_SetPriority(MY_TIMER_IRQn, 7);
    NVIC_EnableIRQ(MY_TIMER_IRQn);

    // Clear and start the timer
    MY_TIMER->TASKS_CLEAR = 1;
    MY_TIMER->TASKS_START = 1;
}

uint32_t my_timer_get_ms(void)
{
    // Store the current value of the timer in the CC[1] register, by triggering the capture task
    MY_TIMER->TASKS_CAPTURE[1] = 1;

    // Combine the state of the second variable with the current timer state, and return the result
    return (my_timer_seconds * 1000) + (MY_TIMER->CC[1] / 1000);
}

uint64_t my_timer_get_us(void)
{
    // Store the current value of the timer in the CC[1] register, by triggering the capture task
    MY_TIMER->TASKS_CAPTURE[1] = 1;

    // Combine the state of the second variable with the current timer state, and return the result
    return (uint64_t)my_timer_seconds * 1000000 + MY_TIMER->CC[1];
}

// Timer interrupt handler
void MY_TIMER_IRQHandler(void)
{
    if(MY_TIMER->EVENTS_COMPARE[0])
    {
        MY_TIMER->EVENTS_COMPARE[0] = 0;

        // Increment the second variable
        my_timer_seconds++;
    }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
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

static uint8_t expectedBytes[31] = {
    0x1E,0xCE,0xEE,0xB7,0xA9,0x77,0xF8,0xE3,
    0xB6,0x16,0x54,0x2F,0x9D,0x53,0x33,0xD8,
    0xBA,0x98,0x08,0x24,0xCB,0x3B,0xFC,0x71,
    0xA3,0xF4,0x55,0x94,0x30,0x56,0xE6
};

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    static uint16_t packetsReceived = 0;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_ADV_REPORT: {
            ble_gap_evt_t gapEvent = p_ble_evt->evt.gap_evt;
            ble_gap_evt_adv_report_t advReport = gapEvent.params.adv_report;

            if ((advReport.ch_index == 37) && (advReport.data.len == 31)) {
                // NRF_LOG_RAW_INFO("channelIndex  %d\r\n", advReport.ch_index);
                // for (int i = 0; i < advReport.data.len; i++) {
                //     NRF_LOG_RAW_INFO("%02X ", advReport.data.p_data[i]);
                //     if (((i+1) % 8) == 0) { NRF_LOG_RAW_INFO("\r\n"); }
                // }   NRF_LOG_RAW_INFO("\r\n");

                uint8_t mismatchedBytes = 0;
                for (uint8_t i = 0; i < advReport.data.len; i++) {
                    if (advReport.data.p_data[i] != expectedBytes[i]) {
                        mismatchedBytes++;
                    }
                }

                if (!mismatchedBytes) {
                    NRF_LOG_RAW_INFO("%d | ", my_timer_get_ms());
                    NRF_LOG_RAW_INFO("rssi %d ", advReport.rssi);
                    NRF_LOG_RAW_INFO ("total %d", ++packetsReceived);
                    NRF_LOG_RAW_INFO("\r\n");
                }

                // NRF_LOG_RAW_INFO ("------------------------\r\n");
            }
            break;
        }

        default: {
            NRF_LOG_RAW_INFO ("Unsupported header event: %d\r\n", p_ble_evt->header.evt_id);
            break;
        }

    }
}

static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    switch(p_scan_evt->scan_evt_id) {
        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        default: {
            // NRF_LOG_RAW_INFO ("Unsupported scan event: %d\r\n", p_scan_evt->scan_evt_id);
            break;
        }
    }
}

static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param     = &m_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id) {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress) {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;
        default:
            break;
    }
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

    // Register event handling
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}

void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
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
        advertisingChannel = 37 + (((advertisingChannel+1) % 37) % 3);

        if (advertisingChannel == 37) { m_adv_params.channel_mask[4] = 0xC0; }
        else if (advertisingChannel == 38) { m_adv_params.channel_mask[4] = 0xA0; }
        else if (advertisingChannel == 39) { m_adv_params.channel_mask[4] = 0x60; }

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
    log_init();
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
    my_timer_start();

    // Scan initialization
    scan_init();
    scan_start();

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

