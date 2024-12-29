/*
 * NEO-6M.h
 *
 *  Created on: Dec 25, 2024
 *      Author: bilal
 */

#ifndef SRC_NEO_6M_H_
#define SRC_NEO_6M_H_


#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define GPS_BUFFER_SIZE 1024

void parse_gps_data(char *nmea);

void getGPSValue();

void process_gps_dma_buffer(UART_HandleTypeDef *uart);

#endif /* SRC_NEO_6M_H_ */
