/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
/** 
 * Perhipheral: nRF52 SAADC
 * Compatibility: nRF52832 rev 1/nRF52840 Eng A, nRF5 SDK 13.0.0
 * Softdevice used: No softdevice
 *

 * This example enables the RTC timer to periodically trigger SAADC sampling. RTC is chosen here instead of 
 * TIMER because it is low power. The example samples on a single input pin, the AIN0, which maps to physical pin P0.02 on the nRF52832 IC.
 * This SAADC example shows the following features:
 * - Low Power -> Enabled with initializing SAADC when sampling and uninitializing when sampling is complete.
 *                Low power can only be obtained when UART_PRINTING_ENABLED is not defined and
 *                SAADC_SAMPLES_IN_BUFFER is 1
 * - Oversampling -> This reduces SAADC noise level, especially for higher SAADC resolutions, see
 *                   https://devzone.nordicsemi.com/question/83938/nrf52832-saadc-sampling/?comment=84340#comment-84340
 *                   Configured with the SAADC_OVERSAMPLE constant.
 * - BURST mode -> Burst mode can be combined with oversampling, which makes the SAADC sample all oversamples as fast
 *                 as it can with one SAMPLE task trigger. Set the SAADC_BURST_MODE constant to enable BURST mode.
 * - Offset Calibration -> SAADC needs to be occasionally calibrated. The desired calibration interval depends on the
 *                         expected temperature change rate, see the nRF52832 PS for more information. The
 *                         calibration interval can be adjusted with configuring the SAADC_CALIBRATION_INTERVAL
 *                         constant.
 * The SAADC sample result is printed on UART. To see the UART output, a UART terminal (e.g. Realterm) can be configured on 
 * your PC with the UART configuration set in the uart_config function, which is also described in the saadc example documentation -> 
 * http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v11.0.0/nrf_dev_saadc_example.html?cp=5_0_0_4_5_24
 *

 * Indicators on the nRF52-DK board:
 * LED1: SAADC Sampling triggered 
 * LED2: SAADC sampling buffer full and event received
 * LED3: SAADC Offset calibration complete
 */

#include <stdbool.h>
#include <stdint.h>
#include <nrfx_saadc.h>
#include "nrf_delay.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_ppi.h"
#include "nrfx_rtc.h"

#define SAADC_CHANNEL_COUNT   3
#define SAADC_SAMPLE_INTERVAL_MS 250

#define SAADC_SAMPLE_INTERVAL_TICKS \
    NRFX_RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS * 1000, 32)

static nrf_saadc_value_t samples[SAADC_CHANNEL_COUNT];
static nrfx_saadc_channel_t channels[SAADC_CHANNEL_COUNT] = {NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN7, 0), NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN6, 1), NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN5, 2)};

static const nrfx_rtc_t m_rtc = NRFX_RTC_INSTANCE(2);
static uint32_t next_sample_ticks = SAADC_SAMPLE_INTERVAL_TICKS;
 
static nrf_ppi_channel_t m_ppi_channel;

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        for(int i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("CH%d: %d", i, p_event->data.done.p_buffer[i]);
        }

        NRF_LOG_INFO("Enqueing sampler");
        next_sample_ticks += SAADC_SAMPLE_INTERVAL_TICKS;
        ret_code_t err_code = nrfx_rtc_cc_set(&m_rtc, 0, next_sample_ticks, false);
        APP_ERROR_CHECK(err_code);
    }
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void on_rtc_evt(nrfx_rtc_int_type_t int_type)
{
}

/**@brief Create timers.
 */
static void timers_init()
{
    lfclk_config();                                  //Configure low frequency 32kHz clock

    app_timer_init();   // Not used, but here to show that our RTC2 usage can co-exist.

    ret_code_t err_code;

    nrfx_rtc_config_t rtc_config = NRFX_RTC_DEFAULT_CONFIG;
    err_code = nrfx_rtc_init(&m_rtc, &rtc_config, on_rtc_evt);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_rtc_cc_set(&m_rtc, 0, next_sample_ticks, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_assign(m_ppi_channel,
                                       nrfx_rtc_event_address_get(&m_rtc, NRF_RTC_EVENT_COMPARE_0),
                                       nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrfx_rtc_enable(&m_rtc);
}
 
int main(void)
{
    NRF_POWER->DCDCEN = 1;
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);                       //Configure Logging. LOGGING is used to show the SAADC sampled result. Default is UART, but RTT can be configured in sdk_config.h
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("nrfx_saadc_api2 simple SAADC Low Power Example.");	

    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err_code);
 
    err_code = nrfx_saadc_channels_config(channels, SAADC_CHANNEL_COUNT);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_simple_mode_set((1<<0) | (1<<1) | (1<<2),
                                          NRF_SAADC_RESOLUTION_8BIT,
                                          NRF_SAADC_OVERSAMPLE_DISABLED,
                                          saadc_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_saadc_buffer_set(samples, SAADC_CHANNEL_COUNT);
    APP_ERROR_CHECK(err_code);

    timers_init();

    err_code = nrfx_saadc_mode_trigger();
    APP_ERROR_CHECK(err_code);

    while (1)
    {
        while(NRF_LOG_PROCESS() != NRF_SUCCESS);
        __WFE();
    }
    
}


/** @} */
