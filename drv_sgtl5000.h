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

#ifndef __DRV_SGTL5000_H__
#define __DRV_SGTL5000_H__

/**
 * @brief   Teensy SGTL5000 Audio board driver for nRF52 series
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_util_platform.h"
#include "nrf.h"
#include "nrf_error.h"
#include "boards.h"         /* In order to get Ardunio PIN mapping for DKs */

/* TWI Instance used by SGTL5000 driver */
#define DRV_SGTL5000_TWI_INSTANCE       1

/* Utility EGU interrupt - This interrupt is used to stop the I2S peripheral when the transmission is done */
#define DRV_SGTL5000_EGU_INSTANCE       NRF_EGU3
#define DRV_SGTL5000_EGU_IRQn           SWI3_EGU3_IRQn
#define DRV_SGTL5000_EGU_IRQHandler     SWI3_EGU3_IRQHandler
#define DRV_SGTL5000_EGU_IRQPriority    APP_IRQ_PRIORITY_LOWEST
#define SGTL5000_EGU_TASK_STREAMING_STOP    0

/* TWI settings */
#define DRV_SGTL5000_TWI_ADDR           0x0A /* Note: has been right-shifted such that R/W bit is not part of this value*/
#define DRV_SGTL5000_TWI_FREQ           NRF_DRV_TWI_FREQ_250K
#define DRV_SGTL5000_TWI_IRQPriority    APP_IRQ_PRIORITY_HIGH

/* TWI pin mapping */
#define DRV_SGTL5000_TWI_PIN_SCL        ARDUINO_SCL_PIN  /* 27 for both DKs */
#define DRV_SGTL5000_TWI_PIN_SDA        ARDUINO_SDA_PIN  /* 26 for both DKs */

/* I2S Settings */
#define DRV_SGTL5000_I2S_IRQPriority    DRV_SGTL5000_EGU_IRQPriority

/* I2S pin mapping */
#define DRV_SGTL5000_I2S_PIN_MCLK       ARDUINO_11_PIN  /* 23 for nRF52832 DK, 1,13 for 52840 */
#define DRV_SGTL5000_I2S_PIN_BCLK       ARDUINO_9_PIN   /* 20 for nRF52832 DK, 1,11 for 52840 */
#define DRV_SGTL5000_I2S_PIN_LRCLK      ARDUINO_10_PIN  /* 22 for nRF52832 DK, 1,12 for 52840 */
#define DRV_SGTL5000_I2S_PIN_RX         ARDUINO_13_PIN  /* 25 for nRF52832 DK, 1,15 for 52840 */
#define DRV_SGTL5000_I2S_PIN_TX         ARDUINO_12_PIN  /* 24 for nRF52832 DK, 1,14 for 52840 */

/* Options for input source - Please note that dB settings should be looked at and tested towards your input device! */
#define AUDIO_INPUT_LINEIN      0
#define AUDIO_INPUT_MIC         1
#define AUDIO_ANA_ADC_CTRL_ADC_VOL_LINEIN   0xF  // Matches p22.5dB - CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_p22_5dB. This might be loud!
#define AUDIO_ANA_ADC_CTRL_ADC_VOL_MIC      0x8  // See CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_0dB. You might want to look at DRV_SGTL5000_REGISTER_ADDR_CHIP_MIC_CTRL as well! This also contains level settings
#define AUDIO_ANA_HP_CTRL_HP_VOL            0x40 // Matches NEG20dB - CHIP_ANA_HP_CTRL_HP_VOL_LEFT_neg20dB and RIGHT.

/* Sample frequency configuration structure */
typedef enum
{
    DRV_SGTL5000_FS_31250HZ,
} drv_sgtl5000_sample_freq_t;

/* SGTL500 driver application event types */
typedef enum
{
    DRV_SGTL5000_EVT_I2S_TX_BUF_REQ,        /* Request for I2S TX buffer */
    DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED,   /* I2S RX buffer received */
} drv_sgtl5000_evt_type_t;

/* SGTL5000 driver application event */
typedef struct
{
    drv_sgtl5000_evt_type_t evt;
    union
    {
        struct
        {
            uint32_t * p_data_to_send;          /* Pointer to buffer that should be filled  */
            uint16_t   number_of_words;         /* Buffer size in number of Words (32 bits) */
        } tx_buf_req;
        struct
        {
            uint32_t const * p_data_received;   /* Pointer to buffer that should be filled  */
            uint16_t   number_of_words;         /* Buffer size in number of Words (32 bits) */
        } rx_buf_received;
    } param;
} drv_sgtl5000_evt_t;

/* SGTL5000 driver application event handler definition */
typedef bool (* drv_sgtl5000_handler_t)(drv_sgtl5000_evt_t * p_evt);

/* SGTL5000 driver initialization struct used in init function */
typedef struct
{
    drv_sgtl5000_handler_t     i2s_evt_handler;
    drv_sgtl5000_sample_freq_t fs;
    void *                     i2s_tx_buffer;           /* Pointer to I2S TX double-buffer (should be 2 x uncompressed frame size) */
    void *                     i2s_rx_buffer;           /* Pointer to I2S RX  */
    uint32_t                   i2s_buffer_size_words;   /* Size of buffer (number of 32-bit words) */ 
} drv_sgtl5000_init_t;


/****************************************/
/* SGTL5000 driver function definitions */
/****************************************/
uint32_t drv_sgtl5000_init(drv_sgtl5000_init_t * p_params);
uint32_t drv_sgtl5000_start(void);
uint32_t drv_sgtl5000_start_mic_loopback(void);
uint32_t drv_sgtl5000_start_sample_playback(void);
uint32_t drv_sgtl5000_stop(void);
uint32_t drv_sgtl5000_volume_set(float volume_db);
uint32_t drv_sgtl5000_volume_get(float * p_volume_db);




/* SGTL5000 Register definitions */
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ID            0x0000
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_DIG_POWER     0x0002
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_CLK_CTRL      0x0004
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_I2S_CTRL      0x0006
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_SSS_CTRL      0x000A
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ADCDAC_CTRL   0x000E
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_DAC_VOL       0x0010
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_ADC_CTRL  0x0020
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_HP_CTRL   0x0022
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_CTRL      0x0024
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINREG_CTRL   0x0026
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_REF_CTRL      0x0028
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_MIC_CTRL      0x002A
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_CTRL 0x002C
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_LINE_OUT_VOL  0x002E
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_ANA_POWER     0x0030
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_PLL_CTRL      0x0032
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_TOP_CTRL      0x0034
#define DRV_SGTL5000_REGISTER_ADDR_CHIP_SHORT_CTRL    0x003C
#define DRV_SGTL5000_REGISTER_ADDR_DAP_CONTROL        0x0100
#define DRV_SGTL5000_REGISTER_ADDR_DAP_PEQ            0x0102
#define DRV_SGTL5000_REGISTER_ADDR_DAP_AUDIO_EQ       0x0108
#define DRV_SGTL5000_REGISTER_ADDR_DAP_AVC_CTRL       0x0124
#define DRV_SGTL5000_REGISTER_ADDR_DAP_AVC_THRESHOLD  0x0126
#define DRV_SGTL5000_REGISTER_ADDR_DAP_AVC_ATTACK     0x0128
#define DRV_SGTL5000_REGISTER_ADDR_DAP_AVC_DECAY      0x012A

/* SGTL5000 Bitfields definitions in use for this example */
#define CHIP_ANA_POWER_DAC_MONO_POS                     14  // While DAC_POWERUP is set, allows DAC to be mono for power saving
#define CHIP_ANA_POWER_PLL_POWERUP_POS                  10
#define CHIP_ANA_POWER_VCOAMP_POWERUP                   8
#define CHIP_ANA_POWER_VAG_POWERUP_POS                  7   // Power up VAG reference buffer
#define CHIP_ANA_POWER_ADC_MONO_POS                     6
#define CHIP_ANA_POWER_REFTOP_POWERUP_POS               5
#define CHIP_ANA_POWER_HEADPHONE_POWERUP_POS            4
#define CHIP_ANA_POWER_DAC_POWERUP_POS                  3
#define CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_POS    2
#define CHIP_ANA_POWER_ADC_POWERUP_POS                  1
#define CHIP_ANA_POWER_LINEOUT_POWERUP_POS              0
#define CHIP_ANA_POWER_DAC_MONO_Mono                        0x0  // (Left only)
#define CHIP_ANA_POWER_DAC_MONO_Stereo                      0x1
#define CHIP_ANA_POWER_PLL_POWERUP_Powerdown                0x0
#define CHIP_ANA_POWER_PLL_POWERUP_Powerup                  0x1
#define CHIP_ANA_POWER_VCOAMP_POWERUP_Powerdown             0x0
#define CHIP_ANA_POWER_VCOAMP_POWERUP_Powerup               0x1
#define CHIP_ANA_POWER_VAG_POWERUP_Powerdown                0x0
#define CHIP_ANA_POWER_VAG_POWERUP_Powerup                  0x1
#define CHIP_ANA_POWER_ADC_MONO_Mono                        0x0  // (Left only)
#define CHIP_ANA_POWER_ADC_MONO_Stereo                      0x1
#define CHIP_ANA_POWER_REFTOP_POWERUP_Powerdown             0x0
#define CHIP_ANA_POWER_REFTOP_POWERUP_Powerup               0x1
#define CHIP_ANA_POWER_HEADPHONE_POWERUP_Powerdown          0x0
#define CHIP_ANA_POWER_HEADPHONE_POWERUP_Powerup            0x1
#define CHIP_ANA_POWER_DAC_POWERUP_Powerdown                0x0
#define CHIP_ANA_POWER_DAC_POWERUP_Powerup                  0x1
#define CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_Powerdown  0x0
#define CHIP_ANA_POWER_CAPLESS_HEADPHONE_POWERUP_Powerup    0x1
#define CHIP_ANA_POWER_ADC_POWERUP_Powerdown                0x0
#define CHIP_ANA_POWER_ADC_POWERUP_Powerup                  0x1
#define CHIP_ANA_POWER_LINEOUT_POWERUP_Powerdown            0x0
#define CHIP_ANA_POWER_LINEOUT_POWERUP_Powerup              0x1


#define CHIP_TOP_CTRL_INPUT_FREQ_DIV2_POS           3       // Passthrough MCLK PLL
#define CHIP_TOP_CTRL_ENABLE_INT_OSC_POS            11      // Internal osc used for zero cross detectors.
#define CHIP_TOP_CTRL_INPUT_FREQ_DIV2_Passthrough   0x0
#define CHIP_TOP_CTRL_INPUT_FREQ_DIV2_SYS_MCLKdiv2  0x1     // Only used when input clock is above 17MHz

#define CHIP_REF_CTRL_VAG_VAL_POS           4
#define CHIP_REF_CTRL_BIAS_CTRL_POS         1
#define CHIP_REF_CTRL_SMALL_POP_POS         0
#define CHIP_REF_CTRL_VAG_VAL_1_575V        0x1F
#define CHIP_REF_CTRL_BIAS_CTRL_Nominal     0x0
#define CHIP_REF_CTRL_BIAS_CTRL_p12_5       0x1
#define CHIP_REF_CTRL_SMALL_POP_Normal      0x0
#define CHIP_REF_CTRL_SMALL_POP_Slowdown    0x1

#define CHIP_LINE_OUT_CTRL_OUT_CURRENT_POS      8
#define CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_POS      0
#define CHIP_LINE_OUT_CTRL_OUT_CURRENT_0_54mA   0xF
#define CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_1_650V   0x22
#define CHIP_LINE_OUT_CTRL_LO_VAGCNTRL_1_175V   0xF

#define CHIP_SHORT_CTRL_LVLADJR_POS                     12
#define CHIP_SHORT_CTRL_LVLADJL_POS                     8
#define CHIP_SHORT_CTRL_LVLADJC_POS                     4
#define CHIP_SHORT_CTRL_MODE_LR_POS                     2
#define CHIP_SHORT_CTRL_MODE_CM_POS                     0
#define CHIP_SHORT_CTRL_LVLADJR_125mA                   0x4
#define CHIP_SHORT_CTRL_LVLADJL_125mA                   0x4
#define CHIP_SHORT_CTRL_LVLADJC_250mA                   0x4
#define CHIP_SHORT_CTRL_MODE_LR_ShortDetectResetLatch   0x1
#define CHIP_SHORT_CTRL_MODE_CM_ShortDetectAutoReset    0x2

#define CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_POS                  8
#define CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_POS                 4
#define CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_POS                  0
#define CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_NoChange             0x0
#define CHIP_ANA_ADC_CTRL_ADC_VOL_M6DB_ADCrangereduce6dB    0x1
#define CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_0dB                 0x0
#define CHIP_ANA_ADC_CTRL_ADC_VOL_RIGHT_p22_5dB             0xF
#define CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_0dB                  0x0
#define CHIP_ANA_ADC_CTRL_ADC_VOL_LEFT_p22_5dB              0xF

#define CHIP_ANA_HP_CTRL_HP_VOL_LEFT_POS        0       // Left headphone channel volume. 0.5dB steps. 0x18 is 0dB, maks +12, min -51.5.
#define CHIP_ANA_HP_CTRL_HP_VOL_RIGHT_POS       8       // Right headphone channel volume. 0.5dB steps. 0x18 is 0dB, maks +12, min -51.5.
#define CHIP_ANA_HP_CTRL_HP_VOL_LEFT_neg51_5dB  0x7F
#define CHIP_ANA_HP_CTRL_HP_VOL_LEFT_neg20dB    0x40
#define CHIP_ANA_HP_CTRL_HP_VOL_LEFT_0dB        0x18
#define CHIP_ANA_HP_CTRL_HP_VOL_LEFT_p12dB      0x00
#define CHIP_ANA_HP_CTRL_HP_VOL_RIGHT_neg20dB   0x40


#define CHIP_ANA_CTRL_MUTE_LO_POS           8   // LINEOUT Mute
#define CHIP_ANA_CTRL_SELECT_HP_POS         6   // Select headphone input
#define CHIP_ANA_CTRL_EN_ZCD_HP_POS         5   // Enable headphone zero cross detector
#define CHIP_ANA_CTRL_MUTE_HP_POS           4   // Mute headphone outputs
#define CHIP_ANA_CTRL_SELECT_ADC_POS        2   // Select ADC input
#define CHIP_ANA_CTRL_EN_ZCD_ADC_POS        1   // Enable ADC analog zero cross detector
#define CHIP_ANA_CTRL_MUTE_ADC_POS          0   // Mute ADC analog volume
#define CHIP_ANA_CTRL_MUTE_LO_Unmute        0x0
#define CHIP_ANA_CTRL_MUTE_LO_Mute          0x1
#define CHIP_ANA_CTRL_SELECT_HP_DAC         0x0
#define CHIP_ANA_CTRL_SELECT_HP_LINEIN      0x1
#define CHIP_ANA_CTRL_EN_ZCD_HP_Disabled    0x0
#define CHIP_ANA_CTRL_EN_ZCD_HP_Enabled     0x1
#define CHIP_ANA_CTRL_MUTE_HP_Unmute        0x0
#define CHIP_ANA_CTRL_MUTE_HP_Mute          0x1
#define CHIP_ANA_CTRL_SELECT_ADC_Microphone 0x0
#define CHIP_ANA_CTRL_SELECT_ADC_LINEIN     0x1
#define CHIP_ANA_CTRL_EN_ZCD_ADC_Disabled   0x0
#define CHIP_ANA_CTRL_EN_ZCD_ADC_Enabled    0x1
#define CHIP_ANA_CTRL_MUTE_ADC_Unmute       0x0
#define CHIP_ANA_CTRL_MUTE_ADC_Mute         0x1

#define CHIP_DIG_POWER_ADC_POWERUP_POS          6
#define CHIP_DIG_POWER_DAC_POWERUP_POS          5
#define CHIP_DIG_POWER_DAP_POWERUP_POS          4
#define CHIP_DIG_POWER_I2S_OUT_POWERUP_POS      1
#define CHIP_DIG_POWER_I2S_IN_POWERUP_POS       0
#define CHIP_DIG_POWER_ADC_POWERUP_Disable      0x0
#define CHIP_DIG_POWER_ADC_POWERUP_Enable       0x1
#define CHIP_DIG_POWER_DAC_POWERUP_Disable      0x0
#define CHIP_DIG_POWER_DAC_POWERUP_Enable       0x1
#define CHIP_DIG_POWER_DAP_POWERUP_Disable      0x0
#define CHIP_DIG_POWER_DAP_POWERUP_Enable       0x1
#define CHIP_DIG_POWER_I2S_OUT_POWERUP_Disable  0x0
#define CHIP_DIG_POWER_I2S_OUT_POWERUP_Enable   0x1
#define CHIP_DIG_POWER_I2S_IN_POWERUP_Disable   0x0
#define CHIP_DIG_POWER_I2S_IN_POWERUP_Enable    0x1

#define CHIP_I2S_CTRL_SCLKFREQ_POS      8
#define CHIP_I2S_CTRL_DLEN_POS          4
#define CHIP_I2S_CTRL_SCLKFREQ_32Fs     0x0
#define CHIP_I2S_CTRL_DLEN_16bits       0x3

#define CHIP_SSS_CTRL_DAP_SELECT_POS    6   // Select data source for DAP
#define CHIP_SSS_CTRL_DAC_SELECT_POS    4   // Select data source for DAC
#define CHIP_SSS_CTRL_I2S_SELECT_POS    0   // Select data source for I2S_DOUT
#define CHIP_SSS_CTRL_DAP_SELECT_ADC    0x0
#define CHIP_SSS_CTRL_DAP_SELECT_I2S_IN 0x1
#define CHIP_SSS_CTRL_DAC_SELECT_ADC    0x0
#define CHIP_SSS_CTRL_DAC_SELECT_I2S_IN 0x1
#define CHIP_SSS_CTRL_DAC_SELECT_DAP    0x3
#define CHIP_SSS_CTRL_I2S_SELECT_ADC    0x0
#define CHIP_SSS_CTRL_I2S_SELECT_I2S_IN 0x1
#define CHIP_SSS_CTRL_I2S_SELECT_DAP    0x3

#define CHIP_ADCDAC_CTRL_ADC_HPF_BYPASS_POS         0
#define CHIP_ADCDAC_CTRL_ADC_HPF_FREEZE_POS         1
#define CHIP_ADCDAC_CTRL_DAC_MUTE_LEFT_POS          2
#define CHIP_ADCDAC_CTRL_DAC_MUTE_RIGHT_POS         3
#define CHIP_ADCDAC_CTRL_VOL_EXPO_RAMP_POS          8
#define CHIP_ADCDAC_CTRL_VOL_RAM_EN_POS             9
#define CHIP_ADCDAC_CTRL_VOL_BUSY_DAC_LEFT_POS      12
#define CHIP_ADCDAC_CTRL_VOL_BUSY_DAC_RIGHT_POS     13
#define CHIP_ADCDAC_CTRL_ADC_HPF_BYPASS_Normal      0x0
#define CHIP_ADCDAC_CTRL_ADC_HPF_BYPASS_Bypassed    0x1  // Bypassed and offset not updated
#define CHIP_ADCDAC_CTRL_ADC_HPF_FREEZE_Normal      0x0
#define CHIP_ADCDAC_CTRL_ADC_HPF_FREEZE_Freeze      0x1  // Freeze the ADC high-pass filter offset register. Offset continues to be subtracted from the ADC data stream


#define CHIP_DAP_CONTROL_DAP_EN_POS     0   // Enable/Disable digital audio processing (DAP)
#define CHIP_DAP_CONTROL_DAP_EN_Disable 0x0 // No audio passes through
#define CHIP_DAP_CONTROL_DAP_EN_Enable  0x1 // audio can pass through even if none of DAP functions are enabled

#define DAP_PEQ_EN_POS                  0
#define DAP_PEQ_EN_Disabled             0x0
#define DAP_PEQ_EN_2Filters             0x2
#define DAP_PEQ_EN_7Filters             0x7

#define DAP_AUDIO_EQ_EN_POS             0
#define DAP_AUDIO_EQ_EN_Disabled        0x0
#define DAP_AUDIO_EQ_EN_EnabledPEQ      0x1

#define DAP_AVC_CTRL_EN_POS             0
#define DAP_AVC_CTRL_EN_Disable         0x0
#define DAP_AVC_CTRL_EN_Enable          0x1

#define DAP_AVC_THRESHOLD_THRESH_POS        0
#define DAP_AVC_THRESHOLD_THRESH_neg12dB    0x1473
#define DAP_AVC_THRESHOLD_THRESH_neg18dB    0x0A40

#define DAP_AVC_ATTACK_RATE_POS         0
#define DAP_AVC_ATTACK_RATE_16dB_s      0x0014  // Example value from datasheet, might be incorrect

#define DAP_AVC_DECAY_RATE_POS          0
#define DAP_AVC_DECAY_RATE_2dB_s        0x0028  // Example value from datasheet, might be incorrect


#define CHIP_MIC_CTRL_BIAS_RESISTOR_POS 8   // MIC Bias output impedance adjustment - zero means powered off
#define CHIP_MIC_CTRL_BIAS_RESISTOR_Off 0x0
#define CHIP_MIC_CTRL_BIAS_RESISTOR_2k  0x1
#define CHIP_MIC_CTRL_BIAS_RESISTOR_4k  0x2
#define CHIP_MIC_CTRL_BIAS_RESISTOR_8k  0x3
#define CHIP_MIC_CTRL_BIAS_VOLT_POS     4   // MIC Bias Voltage Adjustment
#define CHIP_MIC_CTRL_BIAS_VOLT_1_25v   0x0
#define CHIP_MIC_CTRL_BIAS_VOLT_1_50v   0x1
#define CHIP_MIC_CTRL_BIAS_VOLT_1_75v   0x2
#define CHIP_MIC_CTRL_BIAS_VOLT_2_00v   0x3
#define CHIP_MIC_CTRL_BIAS_VOLT_2_25v   0x4
#define CHIP_MIC_CTRL_BIAS_VOLT_2_50v   0x5
#define CHIP_MIC_CTRL_BIAS_VOLT_3_00v   0x7
#define CHIP_MIC_CTRL_GAIN_POS          0   // MIC Amplifier Gain
#define CHIP_MIC_CTRL_GAIN_0dB          0x0
#define CHIP_MIC_CTRL_GAIN_p20dB        0x1
#define CHIP_MIC_CTRL_GAIN_p30dB        0x2
#define CHIP_MIC_CTRL_GAIN_p40dB        0x3

#endif /* __DRV_SGTL5000_H__ */
