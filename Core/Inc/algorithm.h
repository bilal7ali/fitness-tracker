/*
 * algorithm.h
 *
 *  Created on: Dec 7, 2024
 *      Author: bilal
 */

#ifndef SRC_ALGORITHM_H_
#define SRC_ALGORITHM_H_


#include "main.h"
#include "stm32f4xx_hal.h"


#define MINIMUM_ADC_VALUE 180000
#define MEAN_FILTER_SIZE 5

#define PEAK_THRESHOLD 300
#define MIN_PEAK_DISTANCE_MS 300

typedef struct
{
  int32_t values[MEAN_FILTER_SIZE];
  uint8_t index;
  int32_t sum;
  uint8_t count;
} meanDiffFilter_t;

typedef struct
{
	uint32_t lastPeakTime;
	uint8_t peakDetected;
} bpmState_t;

int32_t dcFilter(uint32_t sample);

int32_t meanDiff(int32_t newSample, meanDiffFilter_t *sampleValues);

#endif /* SRC_ALGORITHM_H_ */
