/*
 * MAX30102.h
 *
 *  Created on: Nov 10, 2024
 *      Author: bilal
 */

#ifndef SRC_MAX30102_H_
#define SRC_MAX30102_H_


#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#define MAX30102_I2C_ADDR 0x57
#define MAX30102_I2C_TIMEOUT 1000

#define MAX30102_I2C_WRITE_ADDR 0xAE
#define MAX30102_I2C_READ_ADDR  0xAF

#define MAX30102_BYTES_PER_SAMPLE 6
#define MAX30102_SAMPLE_LEN_MAX 32

#define MAX30102_I2C_TIMEOUT_MS 1000

#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

#define MAX30102_SPO2_CONFIG 0x0a
#define MAX30102_SPO2_ADC_RGE 5
#define MAX30102_SPO2_SR 2
#define MAX30102_SPO2_LEW_PW 0

#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06

#define MAX30102_FIFO_DATA 0x07

#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_FIFO_CONFIG_SMP_AVE 5
#define MAX30102_FIFO_CONFIG_ROLL_OVER_EN 4
#define MAX30102_FIFO_CONFIG_FIFO_A_FULL 0

#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_MODE_SHDN 7
#define MAX30102_MODE_RESET 6
#define MAX30102_MODE_MODE 0

#define MAX30102_SPO2_CONFIG 0x0a
#define MAX30102_SPO2_ADC_RGE 5
#define MAX30102_SPO2_SR 2
#define MAX30102_SPO2_LEW_PW 0

#define MAX30102_LED_IR_PA1 0x0c
#define MAX30102_LED_RED_PA2 0x0d

#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

#define MAX30102_DIE_TINT 0x1f
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1

typedef struct
{
	I2C_HandleTypeDef *_ui2c;
	uint32_t _ir_samples[MAX30102_SAMPLE_LEN_MAX];
	uint32_t _red_samples[MAX30102_SAMPLE_LEN_MAX];
	uint8_t _interrupt_flag;
} max30102_t;

typedef enum max30102_mode_t
{
    MAX30102_heartRate = 0x02,
    MAX30102_spo2 = 0x03,
    MAX30102_multiLed = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32,
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

void max30102_plot(uint32_t ir_sample, uint32_t red_sample);

void MAX30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c);

void MAX30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);

void MAX30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);

void MAX30102_reset(max30102_t *obj);

void MAX30102_setSettings(max30102_t *obj);

void MAX30102_setInterruptFlag(max30102_t *obj);

uint8_t MAX30102_hasInterrupt(max30102_t *obj);

void MAX30102_interruptHandler(max30102_t *obj);

void MAX30102_setFifoConfig(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);

void MAX30102_readFifo(max30102_t *obj);

void MAX30102_readTemp(max30102_t *obj, int8_t *temp_int, uint8_t *temp_frac);

void MAX30102_ReadMultipleSamples(max30102_t *obj, int num_samples);

HAL_StatusTypeDef MAX30102_CheckConnection(max30102_t *obj);

void MAX30102_clearFifo(max30102_t *obj);

#endif /* SRC_MAX30102_H_ */
