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

#include "drv_sgtl5000.h"

#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_twi.h"
#include "string.h"

/**
 * @brief   Teensy SGTL5000 Audio board driver for nRF52 series
 */

/* Define SINE table for testing audio output if needed for testing */
//#define SGTL5000_SINE_TABLE_LEN 32
//const static int16_t m_sine_table[SGTL5000_SINE_TABLE_LEN] = 
//    {-16384, -13086, -9923,  -7024,  -4509,  -2480,  -1020,  -189, 
//     -21,    -523,   -1674,  -3428,  -5712,  -8433,  -11479, -14726, 
//     -18042, -21289, -24335, -27056, -29340, -31094, -32245, -32747, 
//     -32579, -31748, -30288, -28259, -25744, -22845, -19682, -16384};

/* Define TWI instance used to communicate/configure audio device */
static nrf_drv_twi_t              m_twi_instance   = NRF_DRV_TWI_INSTANCE(DRV_SGTL5000_TWI_INSTANCE);
/* Define i2s handler to handle audio data to and from audio device */
static drv_sgtl5000_handler_t     m_i2s_evt_handler;
/* Define i2s configuration for audio device */
static nrf_drv_i2s_config_t       m_i2s_config;
/* Define volume variable used to set volume of device - This is a untested feature! */
static float                      m_volume;
/* Define audio sample length (has to match included audio sample file!), and include the sample itself. */
#define SAMPLE_LEN                  67200
extern const uint8_t                car_sample[SAMPLE_LEN];
static uint8_t * p_smpl             = (uint8_t *)car_sample;
static uint32_t smpl_idx            = 0;

/* Structure holding i2s buffers used for transmissions */
static struct
{
    uint32_t * tx_buffer;
    uint32_t * rx_buffer;
    uint32_t   buffer_size_words; 
} m_external_i2s_buffer;
     
/* Definition of TWI states */
static volatile enum
{ 
    SGTL5000_TWI_TRANSFER_IDLE,
    SGTL5000_TWI_TRANSFER_PENDING,
    SGTL5000_TWI_TRANSFER_SUCCESS,
    SGTL5000_TWI_TRANSFER_FAILED
} m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;

/* Definiton of SGTL5000 driver states */
static enum
{
    SGTL5000_STATE_UNINITIALIZED,   /* Not initialized */
    SGTL5000_STATE_CONFIGURATION,   /* Providing MCLK and configuring via TWI, but not streaming */
    SGTL5000_STATE_IDLE,            /* Initialized, but not running */
    SGTL5000_STATE_RUNNING,         /* Actively streaming audio to/from application */
    SGTL5000_STATE_RUNNING_LOOPBACK,/* Actively running audio loopback (microphone -> speaker) */
    SGTL5000_STATE_RUNNING_SAMPLE,  /* Actively streaming sample */
} m_state = SGTL5000_STATE_UNINITIALIZED;

/* Static function definitions used in this file */
static void sgtl5000_mclk_enable(void);
static void sgtl5000_mclk_disable(void);
static uint32_t sgtl5000_register_read(uint16_t reg_addr, uint16_t * p_reg_data, bool blocking);
static uint32_t sgtl5000_register_write(uint16_t reg_addr, uint16_t reg_data, bool blocking);
static void sgtl5000_register_write_verify(uint16_t reg_addr, uint16_t reg_data, uint16_t ro_mask);


/* TWI event handler, will change TWI state based on TWI events */
static void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{    
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_SUCCESS;
            break;
        
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_FAILED;
            break;
        
        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_twi_transfer_state = SGTL5000_TWI_TRANSFER_FAILED;
            break;
    }
}

/* I2S event handler matching SDK v14.2 implementation - will be invoked from the SDK v15 I2S event handler in order to ensure compatibility */
static void i2s_data_handler_old(uint32_t const * p_data_received, uint32_t * p_data_to_send, uint16_t number_of_words)
{
    // Non-NULL value in 'p_data_received' indicates that a new portion of
    // data has been received and should be processed.
    if (p_data_received != NULL)
    {
        if (m_state != SGTL5000_STATE_RUNNING)
        {
            // I2S only running in order to provide clock
            return;
        }
        
        drv_sgtl5000_evt_t evt;
        
        evt.evt                                     = DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED;
        evt.param.rx_buf_received.number_of_words   = number_of_words;
        evt.param.rx_buf_received.p_data_received   = p_data_received;
        
        m_i2s_evt_handler(&evt);
    }
    
    // Non-NULL value in 'p_data_to_send' indicates that the driver needs
    // a new portion of data to send.
    if (p_data_to_send != NULL)
    {
        if (m_state != SGTL5000_STATE_RUNNING)
        {
            // I2S only running in order to provide clock
            return;
        }
        
        drv_sgtl5000_evt_t evt;
        bool continue_running;
        
        // Request for I2S data to transmit
        evt.evt                              = DRV_SGTL5000_EVT_I2S_TX_BUF_REQ;
        evt.param.tx_buf_req.number_of_words = number_of_words;
        evt.param.tx_buf_req.p_data_to_send  = p_data_to_send;
        
        continue_running = m_i2s_evt_handler(&evt);
        if (!continue_running)
        {
            DRV_SGTL5000_EGU_INSTANCE->TASKS_TRIGGER[SGTL5000_EGU_TASK_STREAMING_STOP] = 1;  
        }
    }
}


/* I2S event handler. Based on the module state, will play sample, playback microphone data, or forward events to the application */
static void i2s_data_handler(nrf_drv_i2s_buffers_t const * p_released,
                         uint32_t                      status)
{
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }
    
    if (p_released == NULL) 
    {
        // Regardless of module state, when p_released is NULL, we provide the next buffers (to keep implementation a little simpler)
        nrf_drv_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = &m_external_i2s_buffer.rx_buffer[m_external_i2s_buffer.buffer_size_words/2],
            .p_tx_buffer = &m_external_i2s_buffer.tx_buffer[m_external_i2s_buffer.buffer_size_words/2],
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
        
        i2s_data_handler_old(NULL, &m_external_i2s_buffer.tx_buffer[m_external_i2s_buffer.buffer_size_words/2], m_external_i2s_buffer.buffer_size_words/2);
    }
    else if (p_released->p_rx_buffer == NULL)
    {
        // If RX buffer is NULL, no data has been received, and we need to provide the next buffers. Nothing else done (to keep implementation a little simpler).
        nrf_drv_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = &m_external_i2s_buffer.rx_buffer[m_external_i2s_buffer.buffer_size_words/2],
            .p_tx_buffer = &m_external_i2s_buffer.tx_buffer[m_external_i2s_buffer.buffer_size_words/2],
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
        
        i2s_data_handler_old(NULL, &m_external_i2s_buffer.tx_buffer[m_external_i2s_buffer.buffer_size_words/2], m_external_i2s_buffer.buffer_size_words/2);
    }
    else
    {
        // This is the normal standard I2S running state. We check module states here and not before to keep implementation a little simpler.
        if (m_state == SGTL5000_STATE_RUNNING)
        {
            // If we are forwarding events to the application, we let the "old" event handler take care of that
            i2s_data_handler_old(p_released->p_rx_buffer, (uint32_t *)p_released->p_tx_buffer, m_external_i2s_buffer.buffer_size_words/2);
        }
        else if (m_state == SGTL5000_STATE_RUNNING_LOOPBACK)
        {
            // Forward MIC to LINEOUT
            nrf_drv_i2s_buffers_t const next_buffers = {
                .p_rx_buffer = (uint32_t *)p_released->p_tx_buffer,
                .p_tx_buffer = (uint32_t *)p_released->p_rx_buffer,
            };
            APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
        }
        else if (m_state == SGTL5000_STATE_RUNNING_SAMPLE)
        {
            /* Play sample! 16kHz sample played on 32kHz frequency! If frequency is changed, this approach needs to change. */
            APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));
            APP_ERROR_CHECK_BOOL(m_i2s_config.mck_setup == NRF_I2S_MCK_32MDIV8);     // If these change, please change this function as well
            APP_ERROR_CHECK_BOOL(m_i2s_config.ratio == NRF_I2S_RATIO_128X);          // If these change, please change this function as well
            // Also depends on alignement, format, and channels!
            
            /* Extract buffer pointer for TX I2S buffer - uint16_t to match up with sample */
            uint16_t * p_buffer  = (uint16_t *) p_released->p_tx_buffer;
            uint32_t * p_buffer32  = (uint32_t *) p_released->p_tx_buffer;
            /* Since the I2S buffer is double buffered, we are only looking at the requested half. And since channels is LEFT, we only look at half of this as well. */
            uint32_t i2s_buffer_size_words = m_external_i2s_buffer.buffer_size_words/2;
            /* For this requested half, we only need half the amount of sample values because the sample is 16kHz and we are running 32kHz. See NRF_I2S_MCK_32MDIV8 and RATIO. */
            uint32_t pcm_stream_buffer_size = i2s_buffer_size_words;  // Because buffer is int16_t, we need i2s_buffer_size_words values to get half - format LEFT means we need half the I2S buffer
            int16_t pcm_stream[pcm_stream_buffer_size];
            
            /* Clear pcm buffer */
            memset(pcm_stream, 0, pcm_stream_buffer_size);

            /* Check if playing the next part of the sample will exceed the sample size, if not, copy over part of sample to be played */
            if (smpl_idx < SAMPLE_LEN)
            {
                /* Copy i2s_buffer_size_words * 2 number of bytes into pcm_stream (or remaining part of sample. This should fill up half the actual I2S transmit buffer. */
                /* We only want half becuase the sample is a 16kHz sample, and we are running the SGTL500 at 32kHz; see DRV_SGTL5000_FS_31250HZ */
                uint32_t bytes_to_copy = ((smpl_idx + sizeof(pcm_stream)) < SAMPLE_LEN) ? sizeof(pcm_stream) : SAMPLE_LEN - smpl_idx;
                memcpy(pcm_stream, &p_smpl[smpl_idx], bytes_to_copy);
                smpl_idx += bytes_to_copy;
            }
            else 
            {
                /* End of buffer reached. */
                smpl_idx = 0;
                DRV_SGTL5000_EGU_INSTANCE->TASKS_TRIGGER[SGTL5000_EGU_TASK_STREAMING_STOP] = 1;
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
        else 
        {
            // We do not handle this scenario
        }
    }
}


/* EGU interrupt handler. This handler will take care of stopping the I2S peripheral when requested. */
void DRV_SGTL5000_EGU_IRQHandler(void)
{
    if (DRV_SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] != 0)
    {
        DRV_SGTL5000_EGU_INSTANCE->EVENTS_TRIGGERED[SGTL5000_EGU_TASK_STREAMING_STOP] = 0;
        nrf_drv_i2s_stop();
        m_state = SGTL5000_STATE_IDLE;
    }
}


/* Initialization function of this driver. Handles TWI and I2S initializations, and also sets up the Audio board to function properly. */
uint32_t drv_sgtl5000_init(drv_sgtl5000_init_t * p_params)
{
    uint32_t                err_code;
    uint16_t                chip_id;
    
    //NRF_LOG_INFO("drv_sgtl5000_init() ");
    
    if (current_int_priority_get() < DRV_SGTL5000_TWI_IRQPriority)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    
    if (p_params->i2s_tx_buffer             == 0 ||
        p_params->i2s_rx_buffer             == 0 ||
        p_params->i2s_buffer_size_words     == 0 ||
        p_params->i2s_evt_handler           == 0 ||
        p_params->fs                        != DRV_SGTL5000_FS_31250HZ)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    DRV_SGTL5000_EGU_INSTANCE->INTENCLR = 0xFFFFFFFF;
    DRV_SGTL5000_EGU_INSTANCE->INTENSET = 0x0000FFFF;
    
    NVIC_ClearPendingIRQ(DRV_SGTL5000_EGU_IRQn);
    NVIC_SetPriority(DRV_SGTL5000_EGU_IRQn, DRV_SGTL5000_EGU_IRQPriority);
    NVIC_EnableIRQ(DRV_SGTL5000_EGU_IRQn);
    
    // Update configuration
    m_i2s_evt_handler                           = p_params->i2s_evt_handler;
    m_external_i2s_buffer.tx_buffer             = p_params->i2s_tx_buffer;
    m_external_i2s_buffer.rx_buffer             = p_params->i2s_rx_buffer;
    m_external_i2s_buffer.buffer_size_words     = p_params->i2s_buffer_size_words;
    
    // Initialize TWI interface 
    nrf_drv_twi_config_t twi_config = {
        .frequency          = DRV_SGTL5000_TWI_FREQ,
        .interrupt_priority = DRV_SGTL5000_TWI_IRQPriority,
        .scl                = DRV_SGTL5000_TWI_PIN_SCL,
        .sda                = DRV_SGTL5000_TWI_PIN_SDA};
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_config, twi_event_handler, 0);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    nrf_drv_twi_enable(&m_twi_instance);
    
    // Disable pull-up resistors on SCL and SDA (already mounted on audio board)
    nrf_gpio_cfg(
        DRV_SGTL5000_TWI_PIN_SCL, 
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);
    
    nrf_gpio_cfg(
        DRV_SGTL5000_TWI_PIN_SDA, 
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);
    
    // Initialize I2S 
    m_i2s_config.sck_pin      = DRV_SGTL5000_I2S_PIN_BCLK;
    m_i2s_config.lrck_pin     = DRV_SGTL5000_I2S_PIN_LRCLK;
    m_i2s_config.mck_pin      = DRV_SGTL5000_I2S_PIN_MCLK;
    m_i2s_config.sdout_pin    = DRV_SGTL5000_I2S_PIN_TX;
    m_i2s_config.sdin_pin     = DRV_SGTL5000_I2S_PIN_RX;
    m_i2s_config.irq_priority = DRV_SGTL5000_I2S_IRQPriority;
    m_i2s_config.mode         = NRF_I2S_MODE_MASTER;
    m_i2s_config.format       = NRF_I2S_FORMAT_I2S;
    m_i2s_config.alignment    = NRF_I2S_ALIGN_LEFT;
    m_i2s_config.sample_width = NRF_I2S_SWIDTH_16BIT;
    m_i2s_config.channels     = NRF_I2S_CHANNELS_LEFT;

    switch (p_params->fs)
    {
        case DRV_SGTL5000_FS_31250HZ:
            /* Please note that SGTL5000_STATE_RUNNING_SAMPLE relies on this setting, so if this changes, sample playback has to be changed as well */
            m_i2s_config.mck_setup    = NRF_I2S_MCK_32MDIV8; // MCLK = NRF_I2S_MCK_32MDIV8 = 4 MHz.
            m_i2s_config.ratio        = NRF_I2S_RATIO_128X;  // BCLK = NRF_I2S_RATIO_128X = 4 MHz / 128 = 31250 Hz.
            break;
        
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    
    
    err_code = nrf_drv_i2s_init(&m_i2s_config, i2s_data_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_state  = SGTL5000_STATE_IDLE;
    m_volume = -25.f;
    
    sgtl5000_mclk_enable();
    
    // Read ID register
    for (int i = 0; i < 5; ++i)
    {
        err_code = sgtl5000_register_read(DRV_SGTL5000_REGISTER_ADDR_CHIP_ID, &chip_id, true);
        if (err_code == NRF_SUCCESS && (chip_id & 0xFF00) == 0xA000)
        {
            break;
        }
    }

    if (err_code != NRF_SUCCESS || (chip_id & 0xFF00) != 0xA000)
    {
        sgtl5000_mclk_disable();
        return NRF_ERROR_NOT_FOUND;
    }


    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER, 
                                        (CHIP_ANA_POWER_REFTOP_POWERUP_Powerup << CHIP_ANA_POWER_REFTOP_POWERUP_POS)
                                        , 0xFFFF);  // VDDD is externally driven with 1.8V (power up reference bias currents)
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_REF_CTRL, 
                                        (CHIP_REF_CTRL_VAG_VAL_1_575V << CHIP_REF_CTRL_VAG_VAL_POS) | 
                                        (CHIP_REF_CTRL_BIAS_CTRL_p12_5 << CHIP_REF_CTRL_BIAS_CTRL_POS)
                                        , 0xFFFF);  // VAG=1.575, normal ramp, +12.5% bias current
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_CTRL, 
                                        (CHIP_LINE_OUT_CTRL_OUT_CURRENT_0_54mA << CHIP_LINE_OUT_CTRL_OUT_CURRENT_POS) | 
                                        (CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_1_650V << CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_POS)
                                        , 0xFFFF);  // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_SHORT_CTRL, 
                                        (CHIP_SHORT_CTRL_LVLADJR_125mA << CHIP_SHORT_CTRL_LVLADJR_POS) |
                                        (CHIP_SHORT_CTRL_LVLADJL_125mA << CHIP_SHORT_CTRL_LVLADJL_POS) |
                                        (CHIP_SHORT_CTRL_LVLADJC_250mA << CHIP_SHORT_CTRL_LVLADJC_POS) |
                                        (CHIP_SHORT_CTRL_MODE_LR_ShortDetectResetLatch << CHIP_SHORT_CTRL_MODE_LR_POS) |
                                        (CHIP_SHORT_CTRL_MODE_CM_ShortDetectAutoReset << CHIP_SHORT_CTRL_MODE_CM_POS)
                                        , 0xFFFF);  // allow up to 125mA
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 
                                        (CHIP_ANA_CTRL_MUTE_LO_Mute << CHIP_ANA_CTRL_MUTE_LO_POS) |
                                        (CHIP_ANA_CTRL_EN_ZCD_HP_Enabled << CHIP_ANA_CTRL_EN_ZCD_HP_POS) |
                                        (CHIP_ANA_CTRL_MUTE_HP_Mute << CHIP_ANA_CTRL_MUTE_HP_POS) |
                                        (CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS) |
                                        (CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS) |
                                        (CHIP_ANA_CTRL_MUTE_ADC_Mute << CHIP_ANA_CTRL_MUTE_ADC_POS)
                                        , 0xFFFF);  // enable zero cross detectors
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_DIG_POWER, 
                                        (CHIP_DIG_POWER_ADC_POWERUP_Enable << CHIP_DIG_POWER_ADC_POWERUP_POS) |
                                        (CHIP_DIG_POWER_DAC_POWERUP_Enable << CHIP_DIG_POWER_DAC_POWERUP_POS) |
                                        (CHIP_DIG_POWER_DAP_POWERUP_Enable << CHIP_DIG_POWER_DAP_POWERUP_POS) |
                                        (CHIP_DIG_POWER_I2S_OUT_POWERUP_Enable << CHIP_DIG_POWER_I2S_OUT_POWERUP_POS) |
                                        (CHIP_DIG_POWER_I2S_IN_POWERUP_Enable << CHIP_DIG_POWER_I2S_IN_POWERUP_POS)
                                        , 0xFFFF);  // power up all digital stuff
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER, 
                                        (CHIP_ANA_POWER_DAC_MONO_Stereo << CHIP_ANA_POWER_DAC_MONO_POS) |
                                        (CHIP_ANA_POWER_VAG_POWERUP_Powerup << CHIP_ANA_POWER_VAG_POWERUP_POS) |
                                        (CHIP_ANA_POWER_ADC_MONO_Mono << CHIP_ANA_POWER_ADC_MONO_POS) |
                                        (CHIP_ANA_POWER_REFTOP_POWERUP_Powerup << CHIP_ANA_POWER_REFTOP_POWERUP_POS) |
                                        (CHIP_ANA_POWER_HEADPHONE_POWERUP_Powerup << CHIP_ANA_POWER_HEADPHONE_POWERUP_POS) |
                                        (CHIP_ANA_POWER_DAC_POWERUP_Powerup << CHIP_ANA_POWER_DAC_POWERUP_POS) |
                                        (CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_Powerup << CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_POS) |
                                        (CHIP_ANA_POWER_ADC_POWERUP_Powerup << CHIP_ANA_POWER_ADC_POWERUP_POS) |
                                        (CHIP_ANA_POWER_LINEOUT_POWERUP_Powerup << CHIP_ANA_POWER_LINEOUT_POWERUP_POS)
                                        , 0xFFFF);  // original 0x40FF -> power up: lineout, hp, adc, dac
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_VOL, 
                                        (CHIP_LINE_OUT_CTRL_OUT_CURRENT_0_54mA << CHIP_LINE_OUT_CTRL_OUT_CURRENT_POS) |
                                        (CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_1_175V << CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_POS)
                                        , 0xFFFF);  // default approx 1.3 volts peak-to-peak
    switch (p_params->fs)
    {
        case DRV_SGTL5000_FS_31250HZ:
            sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL, 0x0000, 0xFFFF);  // sys_fs = 32 kHz, rate_mode = sys_fs, mclk_freq = Use PLL
            break;
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_I2S_CTRL, 
                                        (CHIP_I2S_CTRL_SCLKFREQ_32Fs << CHIP_I2S_CTRL_SCLKFREQ_POS) |
                                        (CHIP_I2S_CTRL_DLEN_16bits << CHIP_I2S_CTRL_DLEN_POS)
                                        , 0xFFFF);  // SCLK=32*Fs, 16bit, I2S format, Slave mode
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_SSS_CTRL, 
                                        (CHIP_SSS_CTRL_DAP_SELECT_I2S_IN << CHIP_SSS_CTRL_DAP_SELECT_POS) |
                                        (CHIP_SSS_CTRL_DAC_SELECT_DAP << CHIP_SSS_CTRL_DAC_SELECT_POS) |
                                        (CHIP_SSS_CTRL_I2S_SELECT_ADC << CHIP_SSS_CTRL_I2S_SELECT_POS)
                                        , 0xFFFF);  // ADC - DAP, DAP - DAC, ADC - I2Sdout
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ADCDAC_CTRL, 0x0000, 0x030F);  // disable dac mute
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_DAC_VOL, 0x3C3C, 0xFFFF);  // digital gain, 0dB
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, 
                                        (AUDIO_ANA_HP_CTRL_HP_VOL << CHIP_ANA_HP_CTRL_HP_VOL_LEFT_POS) |
                                        (AUDIO_ANA_HP_CTRL_HP_VOL << CHIP_ANA_HP_CTRL_HP_VOL_RIGHT_POS) 
                                        , 0xFFFF);  // set analog gain 0x18 - 0dB. 0x0 +12dB.
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 
                                        (CHIP_ANA_CTRL_EN_ZCD_HP_Enabled << CHIP_ANA_CTRL_EN_ZCD_HP_POS) |
                                        (CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS) |
                                        (CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS)
                                        , 0xFFFF);  // enable zero cross detectors. Unmute HP
    
    // MIC input - uncommment this section, and comment out LINEIN section if MIC input is desired
    #if (AUDIO_INPUT_MIC == 1)
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_MIC_CTRL, 
                                            (CHIP_MIC_CTRL_GAIN_0dB << CHIP_MIC_CTRL_GAIN_POS) |
                                            (CHIP_MIC_CTRL_BIAS_VOLT_3_00v << CHIP_MIC_CTRL_BIAS_VOLT_POS) |
                                            (CHIP_MIC_CTRL_BIAS_RESISTOR_2k << CHIP_MIC_CTRL_BIAS_RESISTOR_POS)
                                            , 0xFFFF);
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_ADC_CTRL, 
                                            (CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange << CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS) |
                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_MIC << CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS) |
                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_MIC << CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS)
                                            , 0xFFFF);    // Volume control.
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 
                                        (CHIP_ANA_CTRL_SELECT_HP_DAC << CHIP_ANA_CTRL_SELECT_HP_POS) |
                                        (CHIP_ANA_CTRL_SELECT_ADC_Microphone << CHIP_ANA_CTRL_SELECT_ADC_POS)
                                        , 0xFFFF);
    #endif //(AUDIO_INPUT_MIC == 1)

    // LINEIN input - uncommment this section, and comment out MIC section if LINEIN input is desired
    // Please note that this section has not bee tested recently - might require some changes in dB settings, etc
    #if (AUDIO_INPUT_LINEIN == 1)
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_ADC_CTRL, 
                                            (CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange << CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS) |
                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS) |
                                            (AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN << CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS)
                                            , 0xFFFF);    // Volume control.
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL, 
                                        (CHIP_ANA_CTRL_EN_ZCD_HP_Enabled << CHIP_ANA_CTRL_EN_ZCD_HP_POS) |
                                        (CHIP_ANA_CTRL_SELECT_HP_DAC << CHIP_ANA_CTRL_SELECT_HP_POS) |
                                        (CHIP_ANA_CTRL_SELECT_ADC_LINEIN << CHIP_ANA_CTRL_SELECT_ADC_POS) |
                                        (CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled << CHIP_ANA_CTRL_EN_ZCD_ADC_POS)
                                        , 0xFFFF);
    #endif //(AUDIO_INPUT_LINEIN == 1)
    
    sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_DAP_CONTROL, (CHIP_DAP_CONTROL_DAP_EN_Enable << CHIP_DAP_CONTROL_DAP_EN_POS) , 0xFFFF);


    sgtl5000_mclk_disable();
    m_state = SGTL5000_STATE_IDLE;

    return NRF_SUCCESS;
}


/* Ensures that MCLK is high enough for TWI to function properly with audio board. */
static bool sgtl5000_mclk_high_enough_for_twi(void)
{
    if (m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4 ||
        m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV3 ||
        m_i2s_config.mck_setup == I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2)
    {
        return true;
    }
    
    return false;
}


/* Enables MCLK in order to do TWI configurations of audio board. */
static void sgtl5000_mclk_enable(void)
{
    uint32_t             err_code;
    nrf_drv_i2s_config_t i2s_config;
    
    // Initialize I2S interface with MCLK >= 8 MHz in order to provide MCLK for initial SGTL5000 register access
    memcpy(&i2s_config, &m_i2s_config, sizeof(m_i2s_config));
    
    i2s_config.sdout_pin = NRF_DRV_I2S_PIN_NOT_USED;
    i2s_config.sdin_pin  = NRF_DRV_I2S_PIN_NOT_USED;
    i2s_config.mck_setup = NRF_I2S_MCK_32MDIV2;  // 16 MHz
    
    err_code = nrf_drv_i2s_init(&i2s_config, i2s_data_handler);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        nrf_drv_i2s_uninit();
        err_code = nrf_drv_i2s_init(&i2s_config, i2s_data_handler);
    }
    
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = m_external_i2s_buffer.tx_buffer,
        .p_rx_buffer = m_external_i2s_buffer.rx_buffer,
    };
    err_code = nrf_drv_i2s_start(&initial_buffers, (m_external_i2s_buffer.buffer_size_words/2), 0);
    APP_ERROR_CHECK(err_code);
}


/* Disables MCLK. */
static void sgtl5000_mclk_disable(void)
{
    uint32_t err_code;
    // Initialize I2S interface with all pins again
    nrf_drv_i2s_stop();
    nrf_drv_i2s_uninit();
    err_code = nrf_drv_i2s_init(&m_i2s_config, i2s_data_handler);
    APP_ERROR_CHECK(err_code);
}


/* Reads register over TWI from audio board. */
static uint32_t sgtl5000_register_read(uint16_t reg_addr, uint16_t * p_reg_data, bool blocking)
{
    nrf_drv_twi_xfer_desc_t twi_xfer;
    uint32_t                twi_flags;
    uint32_t                err_code;
    uint8_t                 reg_addr_buf[2];
    uint8_t                 reg_data_buf[2];
    
    reg_addr_buf[0] = (reg_addr >> 8) & 0xFF;
    reg_addr_buf[1] = (reg_addr)      & 0xFF;
    twi_flags       = NRF_DRV_TWI_FLAG_REPEATED_XFER;
    
    twi_xfer.address          = DRV_SGTL5000_TWI_ADDR;
    twi_xfer.type             = NRF_DRV_TWI_XFER_TXRX;
    twi_xfer.primary_length   = sizeof(reg_addr_buf);
    twi_xfer.p_primary_buf    = reg_addr_buf;
    twi_xfer.secondary_length = sizeof(reg_data_buf);
    twi_xfer.p_secondary_buf  = reg_data_buf;
    
    memset(reg_data_buf, 0, sizeof(reg_data_buf));
    
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
    
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &twi_xfer, twi_flags);
    
    if (err_code != NRF_SUCCESS)
    {
        sgtl5000_mclk_disable();
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
        return err_code;
    }
    
    while (blocking && (m_twi_transfer_state == SGTL5000_TWI_TRANSFER_PENDING))
    {
        __WFE();
    }
    
    if (err_code != NRF_SUCCESS || m_twi_transfer_state != SGTL5000_TWI_TRANSFER_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
        return err_code;
    }
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
    
    *p_reg_data = (reg_data_buf[0] << 8 | reg_data_buf[1]);
    
    return NRF_SUCCESS;
}


/* Writes register over TWI of audio board. */
static uint32_t sgtl5000_register_write(uint16_t reg_addr, uint16_t reg_data, bool blocking)
{
    nrf_drv_twi_xfer_desc_t twi_xfer;
    uint32_t                twi_flags;
    uint32_t                err_code;
    uint8_t                 write_buf[4];
    
    write_buf[0] = (reg_addr >> 8) & 0xFF;
    write_buf[1] = (reg_addr)      & 0xFF;
    write_buf[2] = (reg_data >> 8) & 0xFF;
    write_buf[3] = (reg_data)      & 0xFF;
    twi_flags    = 0;
    
    twi_xfer.address          = DRV_SGTL5000_TWI_ADDR;
    twi_xfer.type             = NRF_DRV_TWI_XFER_TX;
    twi_xfer.primary_length   = sizeof(write_buf);
    twi_xfer.p_primary_buf    = write_buf;
    
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
    
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &twi_xfer, twi_flags);
    
    if (err_code != NRF_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_PENDING;
        return err_code;
    }
    
    while (blocking && (m_twi_transfer_state == SGTL5000_TWI_TRANSFER_PENDING))
    {
        __WFE();
    }
    
    if (err_code != NRF_SUCCESS || m_twi_transfer_state != SGTL5000_TWI_TRANSFER_SUCCESS)
    {
        m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
        return err_code;
    }
    m_twi_transfer_state = SGTL5000_TWI_TRANSFER_IDLE;
    
    return NRF_SUCCESS;
}


/* Writes and verifies register over TWI of audio board. */
static void sgtl5000_register_write_verify(uint16_t reg_addr, uint16_t reg_data, uint16_t ro_mask)
{
    uint16_t read_value;
    
    //NRF_LOG_INFO("Writing 0x%04x to register 0x%04x ", reg_data, reg_addr);
    do
    {
        nrf_delay_us(50);
        sgtl5000_register_write(reg_addr, reg_data, true);
        nrf_delay_us(50);
        sgtl5000_register_read(reg_addr, &read_value, true);
    } while ((read_value & ro_mask) != reg_data);
}


/* Starts the I2S peripheral - which will communicate with the audio board and forward I2S events to the application. */
uint32_t drv_sgtl5000_start(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING;
        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_external_i2s_buffer.tx_buffer,
            .p_rx_buffer = m_external_i2s_buffer.rx_buffer,
        };
        (void)nrf_drv_i2s_start(&initial_buffers, (m_external_i2s_buffer.buffer_size_words/2), 0);
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}


/* Starts the I2S peripheral, forwards MIC input to Speaker. No events are forwarded to the application. */
uint32_t drv_sgtl5000_start_mic_loopback(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING_LOOPBACK;
        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_external_i2s_buffer.tx_buffer,
            .p_rx_buffer = m_external_i2s_buffer.rx_buffer,
        };
        (void)nrf_drv_i2s_start(&initial_buffers, (m_external_i2s_buffer.buffer_size_words/2), 0);
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}


/* Starts the I2S peripheral, plays an audio sample, then stops the peripheral. No events are forwarded to the application. */
uint32_t drv_sgtl5000_start_sample_playback(void)
{
    if (m_state == SGTL5000_STATE_IDLE)
    {
        m_state = SGTL5000_STATE_RUNNING_SAMPLE;
        nrf_drv_i2s_buffers_t const initial_buffers = {
            .p_tx_buffer = m_external_i2s_buffer.tx_buffer,
            .p_rx_buffer = m_external_i2s_buffer.rx_buffer,
        };
        (void)nrf_drv_i2s_start(&initial_buffers, (m_external_i2s_buffer.buffer_size_words/2), 0);
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}


/* Stops the I2S peripheral */
uint32_t drv_sgtl5000_stop(void)
{
    if (m_state == SGTL5000_STATE_RUNNING ||
        m_state == SGTL5000_STATE_RUNNING_SAMPLE)
    {
        nrf_drv_i2s_stop();
        m_state = SGTL5000_STATE_IDLE;
        
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_INVALID_STATE;
}


/* drv_sgtl5000_volume_set has not been tested nor verified working */
uint32_t drv_sgtl5000_volume_set(float volume_db)
{
    //bool    start_mclk;
    float   volume_float;
    uint8_t volume_right;
    uint8_t volume_left;
    
    if (m_state != SGTL5000_STATE_IDLE && !sgtl5000_mclk_high_enough_for_twi())
    {
        // Need fast MCLK to read/write configuration registers.
        // Cannot do this while streaming audio with 2 MHz MCLK
        return NRF_ERROR_INVALID_STATE;
    }
    
    // Valid range for analog amplifier: -51.5 to +12 dB in .5 dB steps
    if (volume_db > 12.f ||
        volume_db < -51.5f)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_volume = volume_db;
    
    // Value 0x00 = 12 dB (max)
    // Value 0x7F = -51.5 dB (min)
    
    volume_float = volume_db + 51.5f;
    APP_ERROR_CHECK_BOOL(volume_float >= 0.f);
    
    volume_right = (uint8_t) 0x7F - ((volume_float / 63.5f) * 127.f);
    volume_left  = volume_right;
    
    APP_ERROR_CHECK_BOOL(volume_right <= 0x7F);
    APP_ERROR_CHECK_BOOL(volume_left <= 0x7F);

    char str[100];
    
    sprintf(str, "Setting volume to %f (0x%02x) ", volume_db, volume_right);
    //NRF_LOG_INFO(str);
    
    if (m_state == SGTL5000_STATE_IDLE)
    {
        sgtl5000_mclk_enable();
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, ((volume_right << 8) | volume_left), 0xFFFF);
        sgtl5000_mclk_disable();
    }
    else
    {
        sgtl5000_register_write_verify(DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL, ((volume_right << 8) | volume_left), 0xFFFF);
    }
    
    //NRF_LOG_INFO("Volume setting success");
    
    return NRF_SUCCESS;
}


/* drv_sgtl5000_volume_get has not been tested nor verified working */
uint32_t drv_sgtl5000_volume_get(float * p_volume_db)
{
    if (m_state == SGTL5000_STATE_UNINITIALIZED)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    *p_volume_db = m_volume;
    
    return NRF_SUCCESS;
}

