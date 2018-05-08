/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
 * This file contains the source code for a sample application using I2S.
 * Included as well is the driver and support for using the Teensy SGTL5000 Audio board.
 */

#include <stdio.h>
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/************************************************************
*   Include support for SGTL5000. 
************************************************************/
#include "drv_sgtl5000.h"
#define AUDIO_FRAME_WORDS                   320
#define I2S_BUFFER_SIZE_WORDS               AUDIO_FRAME_WORDS * 2   // Double buffered - I2S lib will switch between using first and second half
static uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
static uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];
    
/* Include car sample to demonstrate that sample can be played from application as well */
#define SAMPLE_LEN                  67200
extern const uint8_t                car_sample[SAMPLE_LEN];
static uint8_t * p_sample           = (uint8_t *)car_sample;
static uint32_t sample_idx          = 0;


static bool i2s_sgtl5000_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
    bool ret = false;
    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler %d", p_evt->evt);

    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler RX BUF RECEIVED");
                // TODO: Handle RX as desired - for this example, we dont use RX for anything
                //uint16_t * p_buffer  = (uint16_t *) p_evt->param.rx_buf_received.p_data_received;
            }
            break;
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler TX BUF REQ");
                
                /* Play sample! 16kHz sample played on 32kHz frequency! If frequency is changed, this approach needs to change. */
                /* Playback of this 16kHz sample depends on I2S MCK, RATIO, Alignment, format, and channels! Needs to be DIV8, RATIO 128X, alignment LEFT, format I2S, channels LEFT. */
                
                uint16_t * p_buffer  = (uint16_t *) p_evt->param.tx_buf_req.p_data_to_send;
                uint32_t i2s_buffer_size_words = p_evt->param.rx_buf_received.number_of_words;
                int16_t pcm_stream[i2s_buffer_size_words];  // int16_t - i2s_buffer_size_words size; means we only cover half of data_to_send_buffer, which is fine since we are only using LEFT channel
                
                /* Clear pcm buffer */
                memset(pcm_stream, 0, sizeof(pcm_stream));

                /* Check if playing the next part of the sample will exceed the sample size, if not, copy over part of sample to be played */
                if (sample_idx < SAMPLE_LEN)
                {
                    /* Copy sample bytes into pcm_stream (or remaining part of sample). This should fill up half the actual I2S transmit buffer. */
                    /* We only want half becuase the sample is a 16kHz sample, and we are running the SGTL500 at 32kHz; see DRV_SGTL5000_FS_31250HZ */
                    uint32_t bytes_to_copy = ((sample_idx + sizeof(pcm_stream)) < SAMPLE_LEN) ? sizeof(pcm_stream) : SAMPLE_LEN - sample_idx - 1;
                    memcpy(pcm_stream, &p_sample[sample_idx], bytes_to_copy);
                    sample_idx += bytes_to_copy;
                    ret = true;
                }
                else 
                {
                    /* End of buffer reached. */
                    sample_idx = 0;
                    ret = false;
                }
                
                /* Upsample the decompressed audio */
                /* i < i2s_buffer_size_words * 2 because we have a uint16_t buffer pointer */
                for (int i = 0, pcm_stream_idx = 0; i < i2s_buffer_size_words * 2; i += 2)
                {
                    for (int j = i; j < (i + 2); ++j)
                    {
                        p_buffer[j] = pcm_stream[pcm_stream_idx];
                    }
                    ++pcm_stream_idx;
                }
            }
            break;
    }

    return ret;
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_clear(LED_2);
    nrf_gpio_pin_clear(LED_3);
    app_error_save_and_stop(id, pc, info);
}


void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_cfg_output(LED_3);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_pin_set(LED_3);
}


int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Teensy Audio board example started.");

    // Enable audio
    drv_sgtl5000_init_t sgtl_drv_params;
    sgtl_drv_params.i2s_tx_buffer           = (void*)m_i2s_tx_buffer;
    sgtl_drv_params.i2s_rx_buffer           = (void*)m_i2s_rx_buffer;
    sgtl_drv_params.i2s_buffer_size_words   = I2S_BUFFER_SIZE_WORDS/2;
    sgtl_drv_params.i2s_evt_handler         = i2s_sgtl5000_driver_evt_handler;
    sgtl_drv_params.fs                      = DRV_SGTL5000_FS_31250HZ;
    
    m_i2s_tx_buffer[0] = 167;
    m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS/2] = 167;
    NRF_LOG_INFO("size of  m_i2s_tx_buffer %d, %d", sizeof(m_i2s_tx_buffer) / sizeof(uint32_t), I2S_BUFFER_SIZE_WORDS);
    NRF_LOG_INFO("i2s_initial_tx_buffer addr1: %d, addr2: %d", m_i2s_tx_buffer, m_i2s_tx_buffer + I2S_BUFFER_SIZE_WORDS/2);
    NRF_LOG_INFO("i2s_initial_Rx_buffer addr1: %d, addr2: %d", m_i2s_rx_buffer, m_i2s_rx_buffer + I2S_BUFFER_SIZE_WORDS/2);
    
    drv_sgtl5000_init(&sgtl_drv_params);
    drv_sgtl5000_stop();
    NRF_LOG_INFO("Audio initialization done.");
    
    /* Demonstrate playback */
    drv_sgtl5000_start_sample_playback();
    nrf_delay_ms(2000);
    drv_sgtl5000_stop();
    
    /* Demonstrate playback form application handler */
    drv_sgtl5000_start();
    nrf_delay_ms(2000);
    drv_sgtl5000_stop();
    
    /* Demonstrate Mic loopback */
    NRF_LOG_INFO("Loop in main and loopback MIC data.");
    drv_sgtl5000_start_mic_loopback();
    
    for (;;)
    {
        NRF_LOG_FLUSH();
    }
}

/** @} */
