/*
 * sensor.c
 *
 *  Created on: Aug 28, 2025
 *      Author: Ogi
 */

/* BEGIN includes  */

#include "sensors_app.h"

#include <stdlib.h>
#include "stm32wlxx_hal.h"
#include "main.h"
#include "adc_if.h"
#include "usart.h"
#include <string.h>

/* END includes  */

extern UART_HandleTypeDef huart1;


/* BEGIN PV */

uint8_t requestMsg[] = { 0x31, 0x20, 0x70, 0x6F, 0x6C, 0x6C, 0x32, 0x0A, 0x0D };
uint8_t RS485_buffer[128] = {0};
char level[11] = {0};
char temperature[11] = {0};
char conductivity[11] = {0};

uint8_t uart_buf_rs485[128] = { 0 };
uint8_t uart_cnt_rs485 = 0;

uint8_t byte[1];

float levelf, tempf, condf;

int32_t level_d = 0;
int32_t temperature_d = 0;
int32_t ec_d = 0;
int32_t i = 0;
uint32_t measuredadc = 0;
float current_mA;
uint8_t send_index=0;

/* END PV */

/* BEGIN PF */


uint8_t level_b[4]={0};


void sensor_Read_Data(float* levelOut, float* tempOut, float* condOut)
{

	memset(RS485_buffer, 0x00, 128);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
	HAL_UART_Transmit(&huart1, requestMsg, sizeof(requestMsg), 100);
 	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);

	if (HAL_UART_Receive(&huart1,RS485_buffer, sizeof(RS485_buffer),100) == HAL_ERROR) {
		 *levelOut = *tempOut = *condOut = -1.0f; // UART hata
		 return;
		    }
        if (RS485_buffer[60] == 0x0a && RS485_buffer[61] == 0x0a)
        {

            for (int i = 0; i <= 10; i++) {
                level[i]        = RS485_buffer[i + 6];
                temperature[i]  = RS485_buffer[i + 21];
                conductivity[i] = RS485_buffer[i + 36];
            }

            *levelOut       = atof(level);
            *tempOut        = atof(temperature);
            *condOut        = atof(conductivity);
        }
        else {
            *levelOut = *tempOut = *condOut = -1.0f; // Geçersiz veri
        }
    }


/*  @brief  4–20 mA (150Ω) -> seviye [metre]
 * - avg_samples  : ADC ortalaması için kaç örnek alınacak (0 ise 1 yapılır)
 * - vdda_mV      : VDDA/Vref (mV) (örn. 3300). VREFINT ölçtüysen onu ver.
 * - level_min_m  : 4 mA’ye karşılık gelen seviye (metre)  (örn. 0.0f)
 * - level_max_m  : 20 mA’ye karşılık gelen seviye (metre) (örn. 1.0f)
 * Dönen: seviye [metre] (float). 4–20 dışına taşarsa uçlara klipler.
 */
float Read4_20_150R(uint16_t avg_samples,
                                  uint32_t vdda_mV,
                                  float    level_min_m,
                                  float    level_max_m)
{
    if (avg_samples == 0) avg_samples = 1;


    uint32_t sum = 0;
    for (uint16_t i = 0; i < avg_samples; i++) {
        int32_t raw = SYS_GetADC1Level();
        if (raw < 0) raw = 0;
        sum += (uint32_t)raw;
    }
    const uint32_t ADC_FS  = 4095U;
    const uint32_t R_SHUNT = 150U;

    uint32_t raw_avg = (sum + (avg_samples/2)) / avg_samples;


    uint32_t vmV = (raw_avg * vdda_mV + (ADC_FS/2)) / ADC_FS;


    uint32_t I_uA = (vmV * 1000U + (R_SHUNT/2)) / R_SHUNT;


//    const uint32_t Imin_uA = 2800U;   // 3550 mA
//    const uint32_t Imax_uA = 21700U;  // 21700 mA
//    if (I_uA <= Imin_uA) return Imin_uA;
//    if (I_uA >= Imax_uA) return Imax_uA;


//    float f = (float)(I_uA - Imin_uA) / (float)(Imax_uA - Imin_uA);  // 0..1
//    return level_min_m + f * (level_max_m - level_min_m);
    return I_uA;
}
/* END PF */
