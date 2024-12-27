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

void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
	static meanDiffFilter_t meanDiffFilter = {0};
//	printf("Unfiltered Red:%lu\r\n", red_sample);
	int32_t dcFilteredRed = dcFilter(red_sample);
	int32_t meanDiffRed = meanDiff(dcFilteredRed, &meanDiffFilter);

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

