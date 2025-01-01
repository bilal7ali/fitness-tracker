/*
 * algorithm.c
 *
 *  Created on: Dec 7, 2024
 *      Author: bilal
 */

#include "MAX30102.h"
#include "algorithm.h"
#include <stdio.h>


extern max30102_t max30102;

static bpmState_t bpmState = {0, 0};

void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
	uint16_t bpm = 0;
	uint16_t timeDelta = 0;
	static meanDiffFilter_t meanDiffFilter = {0};
	int32_t dcFilteredRed = dcFilter(red_sample);
	int32_t meanDiffRed = meanDiff(dcFilteredRed, &meanDiffFilter);
	uint32_t currentTime = HAL_GetTick();

	if (meanDiffRed > PEAK_THRESHOLD && bpmState.peakDetected == 0)
	{
		bpmState.peakDetected = 1;
		timeDelta = currentTime - bpmState.lastPeakTime;

		if (timeDelta > MIN_PEAK_DISTANCE_MS)
		{
			bpm = 60000 / timeDelta;
			printf("BPM: %u\r\n", bpm);
			printf("Red: %ld\r\n", meanDiffRed);
			printf("TIME DELTA: %u\r\n", timeDelta);
			bpmState.lastPeakTime = currentTime;
		}

	}
	else if (meanDiffRed < PEAK_THRESHOLD)
	{
		bpmState.peakDetected = 0;
	}

	printf("Red:%ld\r\n", meanDiffRed);
}

int32_t dcFilter(uint32_t sample)
{
	uint16_t alpha = 95;
	static int32_t prev_w = 0;

	int32_t w = sample + (alpha * prev_w) / 100;
	int32_t result = w - prev_w;

	prev_w = w;

	return result;
}


int32_t meanDiff(int32_t newSample, meanDiffFilter_t *sampleValues)
{
	int32_t avg = 0;

	sampleValues->sum -= sampleValues->values[sampleValues->index];
	sampleValues->values[sampleValues->index] = newSample;
	sampleValues->sum += sampleValues->values[sampleValues->index];

	sampleValues->index++;
	sampleValues->index %= MEAN_FILTER_SIZE;

	if (sampleValues->count < MEAN_FILTER_SIZE)
	{
		sampleValues->count++;
	}

	avg = sampleValues->sum / sampleValues->count;

	return avg - newSample;
}

