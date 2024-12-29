/*
 * NEO-6M.c
 *
 *  Created on: Dec 25, 2024
 *      Author: bilal
 */

#include "NEO-6M.h"
#include <stdio.h>

char gps_data[500]; // Buffer to store GPS data
char latitude[15], longitude[15];

bool isGPSValid = 0;

// Extract Latitude and Longitude
void parse_gps_data(char *nmea)
{
    char *token = strtok(nmea, ",");
    if (strcmp(token, "$GPRMC") == 0)
    {
        strtok(NULL, ","); // UTC time
        token = strtok(NULL, ","); // Status

        if (strcmp(token, "A") == 0)
        { // Ensure the status is "A" (active)
        	isGPSValid = true;
            strcpy(latitude, strtok(NULL, ",")); // Latitude value
            token = strtok(NULL, ","); // N/S Indicator

            if (strcmp(token, "S") == 0)
            {
                latitude[0] = '-'; // Convert to negative for southern hemisphere
            }

            strcpy(longitude, strtok(NULL, ",")); // Longitude value
            token = strtok(NULL, ","); // E/W Indicator

            if (strcmp(token, "W") == 0)
            {
                longitude[0] = '-'; // Convert to negative for western hemisphere
            }
        }
        else
        {
            // Invalid GPS data
        	isGPSValid = false;
            strcpy(latitude, "");
            strcpy(longitude, "");
        }
    }
}

void getGPSValue(UART_HandleTypeDef *uart)
{
    memset(gps_data, 0, sizeof(gps_data));
    memset(latitude, 0, sizeof(latitude));
    memset(longitude, 0, sizeof(longitude));

    // Receive the raw GPS data
    HAL_UART_Receive(uart, (uint8_t *)gps_data, sizeof(gps_data), HAL_MAX_DELAY);

    char *start = gps_data;
    char *end;

    while ((start = strchr(start, '$')) != NULL) // Find start of NMEA sentence
    {
        end = strchr(start, '\r'); // Find end of NMEA sentence

        if (end != NULL)
        {
            *end = '\0';
//            printf("LINE: %s\r\n", start);
            parse_gps_data(start);
            start = end + 1;
        }
        else
        {
            break;
        }
    }

    if (isGPSValid)
    {
        printf("Latitude: %s, Longitude: %s\r\n", latitude, longitude);
        printf("GPS VALID? %d\r\n", isGPSValid);
    }
    else
    {
        printf("NO GPS SIGNAL\r\n");
    }
}

