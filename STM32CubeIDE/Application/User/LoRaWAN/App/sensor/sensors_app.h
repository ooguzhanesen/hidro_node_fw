/*
 * sensor1.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Ogi
 */

#ifndef APPLICATION_USER_LORAWAN_APP_SENSOR_SENSORS_APP_H_
#define APPLICATION_USER_LORAWAN_APP_SENSOR_SENSORS_APP_H_

#include <stdint.h>

/* ========= API ========= */

void sensor_Read_Data(float* levelOut, float* tempOut, float* condOut);

/* 4–20 mA (150 Ω şönt) → seviye [metre] dönüşümü */
//float Read4_20_150R(uint16_t avg_samples,
//                    uint32_t vdda_mV,
//                    float    level_min_m,
//                    float    level_max_m);
/* =========  globaller ========= */

extern uint8_t  requestMsg[9];
extern uint8_t  RS485_buffer[128];
extern char     level[11];
extern char     temperature[11];
extern char     conductivity[11];

extern uint8_t  uart_buf_rs485[128];
extern uint8_t  uart_cnt_rs485;
extern uint8_t  byte[1];

extern float    levelf, tempf, condf;

extern int32_t  level_d, temperature_d, ec_d, i;
extern uint32_t measuredadc;
extern float    current_mA;
extern uint8_t  send_index;

float Read4_20_150R(uint16_t avg_samples,
                    uint32_t vdda_mV,
                    float level_min_m,
                    float level_max_m);

extern uint8_t level_b[4];

#endif /* APPLICATION_USER_LORAWAN_APP_SENSOR_SENSORS_APP_H_ */
