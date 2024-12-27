/*
 * MAX30102.c
 *
 *  Created on: Nov 10, 2024
 *      Author: bilal
 */

#include "MAX30102.h"
#include "algorithm.h"
#include <stdio.h>

__weak void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
    UNUSED(ir_sample);
    UNUSED(red_sample);
}

void MAX30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c)
{
	obj->_ui2c = hi2c;
	obj->_interrupt_flag = 0;
	memset(obj->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
	memset(obj->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
}

void MAX30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t *payload = (uint8_t *)malloc((buflen + 1) * sizeof(uint8_t));
    *payload = reg;
    if (buf != NULL && buflen != 0)
        memcpy(payload + 1, buf, buflen);
    HAL_I2C_Master_Transmit(obj->_ui2c, MAX30102_I2C_ADDR << 1, payload, buflen + 1, MAX30102_I2C_TIMEOUT);
    free(payload);
}

void MAX30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t reg_addr = reg;
    HAL_I2C_Master_Transmit(obj->_ui2c, MAX30102_I2C_ADDR << 1, &reg_addr, 1, MAX30102_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(obj->_ui2c, MAX30102_I2C_ADDR << 1, buf, buflen, MAX30102_I2C_TIMEOUT);
}

void MAX30102_reset(max30102_t *obj)
{
    uint8_t val = 0x40;
    MAX30102_write(obj, MAX30102_MODE_CONFIG, &val, 1);
}

void MAX30102_setSettings(max30102_t *obj)
{
    uint8_t reg = 0;

    // Enable DIE_TEMP_RDY (internal temp ready) interrupt
    reg = (1 & 0x01) << MAX30102_INTERRUPT_DIE_TEMP_RDY;
    MAX30102_write(obj, MAX30102_INTERRUPT_ENABLE_2, &reg, 1);

    //Enable DIE_TEMP_EN (temp measurement)
    reg = (1 & 0x01) << MAX30102_DIE_TEMP_EN;
    MAX30102_write(obj, MAX30102_DIE_TEMP_CONFIG, &reg, 1);

    MAX30102_read(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
    reg |= (1 << MAX30102_INTERRUPT_A_FULL); // Enable A_FULL interrupt
    reg |= (1 << MAX30102_INTERRUPT_PPG_RDY); // Enable PPG_RDY interrupt
    reg |= (1 << MAX30102_INTERRUPT_ALC_OVF); // Enable ALC_OVF interrupt
    MAX30102_write(obj, MAX30102_INTERRUPT_ENABLE_1, &reg, 1);

    // Set mode
    uint8_t config;
    MAX30102_read(obj, MAX30102_MODE_CONFIG, &config, 1);
    config = (config & 0xf8) | MAX30102_spo2;
    MAX30102_write(obj, MAX30102_MODE_CONFIG, &config, 1);
    MAX30102_clearFifo(obj);

    // Set sampling rate
    config = 0;
    MAX30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x63) | (max30102_sr_800 << MAX30102_SPO2_SR);
    MAX30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);

    // Set LED pulse width
    config = 0;
    MAX30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x7c) | (max30102_pw_16_bit << MAX30102_SPO2_LEW_PW);
    MAX30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);

    // Set ADC resolution
    config = 0;
    MAX30102_read(obj, MAX30102_SPO2_CONFIG, &config, 1);
    config = (config & 0x1f) | (max30102_adc_2048 << MAX30102_SPO2_ADC_RGE);
    MAX30102_write(obj, MAX30102_SPO2_CONFIG, &config, 1);

    // Set LED1 current
    uint8_t pa = 6.2 / 0.2;
    MAX30102_write(obj, MAX30102_LED_IR_PA1, &pa, 1);

    // Set LED2 current
    pa = 6.2 / 0.2;
    MAX30102_write(obj, MAX30102_LED_RED_PA2, &pa, 1);
}

void MAX30102_setInterruptFlag(max30102_t *obj)
{
    obj->_interrupt_flag = 1;
}

uint8_t MAX30102_hasInterrupt(max30102_t *obj)
{
    return obj->_interrupt_flag;
}

void MAX30102_interruptHandler(max30102_t *obj)
{
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    MAX30102_read(obj, MAX30102_INTERRUPT_STATUS_1, reg, 2);

    if ((reg[0] >> MAX30102_INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
    	MAX30102_readFifo(obj);
    }

    if ((reg[0] >> MAX30102_INTERRUPT_PPG_RDY) & 0x01)
    {
        // New FIFO data ready
    }

    if ((reg[0] >> MAX30102_INTERRUPT_ALC_OVF) & 0x01)
    {
        // Ambient light overflow
    }

    if ((reg[1] >> MAX30102_INTERRUPT_DIE_TEMP_RDY) & 0x01)
    {
        // Temperature data ready
        int8_t temp_int;
        uint8_t temp_frac;
        MAX30102_readTemp(obj, &temp_int, &temp_frac);
        // float temp = temp_int + 0.0625f * temp_frac;
    }

    // Reset interrupt flag
    obj->_interrupt_flag = 0;
}

void MAX30102_setFifoConfig(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full)
{
    uint8_t config = 0x00;
    config |= smp_ave << MAX30102_FIFO_CONFIG_SMP_AVE;
    config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIG_ROLL_OVER_EN);
    config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIG_FIFO_A_FULL);
    MAX30102_write(obj, MAX30102_FIFO_CONFIG, &config, 1);
}

void MAX30102_readFifo(max30102_t *obj)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wrPtr = 0, rdPtr = 0;
    MAX30102_read(obj, MAX30102_FIFO_WR_PTR, &wrPtr, 1);
    MAX30102_read(obj, MAX30102_FIFO_RD_PTR, &rdPtr, 1);

    int8_t numSamples;

    numSamples = (int8_t)wrPtr - (int8_t)rdPtr;
    if (numSamples < 1)
    {
        numSamples += 32;
    }

    // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
    for (int8_t i = 0; i < numSamples; i++)
    {
        uint8_t sample[6];
        MAX30102_read(obj, MAX30102_FIFO_DATA, sample, 6);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
        uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;
        obj->_ir_samples[i] = ir_sample;
        obj->_red_samples[i] = red_sample;
        max30102_plot(ir_sample, red_sample);
    }
}

void MAX30102_readTemp(max30102_t *obj, int8_t *temp_int, uint8_t *temp_frac)
{
    MAX30102_read(obj, MAX30102_DIE_TINT, (uint8_t *)temp_int, 1);
    MAX30102_read(obj, MAX30102_DIE_TFRAC, temp_frac, 1);
}

HAL_StatusTypeDef MAX30102_CheckConnection(max30102_t *obj)
{
    uint8_t partId;
    HAL_StatusTypeDef status;

    // Read the Part ID register (0xFF)
    status = HAL_I2C_Mem_Read(obj->_ui2c, 0xAE, 0xFF, I2C_MEMADD_SIZE_8BIT, &partId, 1, MAX30102_I2C_TIMEOUT_MS);

    if (status == HAL_OK && partId == 0x15) {
        // Successful communication and correct Part ID
        printf("MAX30102 is connected and functioning correctly.\r\n");
        return HAL_OK;
    } else {
        // Communication failed or incorrect Part ID
        printf("Failed to communicate with MAX30102 or incorrect Part ID.\r\n");
        return HAL_ERROR;
    }
}

void MAX30102_clearFifo(max30102_t *obj)
{
    uint8_t val = 0x00;
    MAX30102_write(obj, MAX30102_FIFO_WR_PTR, &val, 3);
    MAX30102_write(obj, MAX30102_FIFO_RD_PTR, &val, 3);
    MAX30102_write(obj, MAX30102_OVF_COUNTER, &val, 3);
}
