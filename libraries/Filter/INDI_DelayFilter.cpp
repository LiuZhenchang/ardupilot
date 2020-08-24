/*
This program realizes a delay filter to ensure the time synchronization of control states  in INDI method
*/
#pragma once
#include <inttypes.h>
#include "INDI_DelayFilter.h"

template <class T, uint8_t FILTER_SIZE>
void INDI_DelayFilter<T, FILTER_SIZE>::update(T sample, uint32_t timestamp)
{
	uint8_t i = FilterWithBuffer<T, FILTER_SIZE>::sample_index;
	uint8_t i1;
	if (i == 0) {
		i1 = FILTER_SIZE - 1;
	}
	else {
		i1 = i - 1;
	}
	if (_timestamps[i1] == timestamp) {
		// this is not a new timestamp - ignore
		return;
	}

	// add timestamp before we apply to FilterWithBuffer
	_timestamps[i] = timestamp;

	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T, FILTER_SIZE>::apply(sample);

	_new_data = true;
}

// output delay results according to Filter_Size
template <class T, uint8_t FILTER_SIZE>
float INDI_DelayFilter<T, FILTER_SIZE>::delay_output(void)
{
	if (!_new_data) {
		return _last_output;
	}

	float result = 0;
	uint8_t i = FilterWithBuffer<T, FILTER_SIZE>::sample_index;

	if (i - (FILTER_SIZE - 1) / 2 >= 0) {
		result = FilterWithBuffer<T, FILTER_SIZE>::samples[i - (FILTER_SIZE - 1) / 2];
	}
	else {
		result = FilterWithBuffer<T, FILTER_SIZE>::samples[i - (FILTER_SIZE - 1) / 2 + FILTER_SIZE];
	}
	_new_data = false;
	_last_output = result;

	return result;
}

// reset - clear all samples
template <class T, uint8_t FILTER_SIZE>
void INDI_DelayFilter<T, FILTER_SIZE>::reset(void)
{
	// call parent's apply function to get the sample into the array
	FilterWithBuffer<T, FILTER_SIZE>::reset();
}

// add new instances as needed here
template void INDI_DelayFilter<float, 3>::update(float sample, uint32_t timestamp);
template float INDI_DelayFilter<float, 3>::delay_output(void);
template void INDI_DelayFilter<float, 3>::reset(void);

template void INDI_DelayFilter<float, 5>::update(float sample, uint32_t timestamp);
template float INDI_DelayFilter<float, 5>::delay_output(void);
template void INDI_DelayFilter<float, 5>::reset(void);

template void INDI_DelayFilter<float, 7>::update(float sample, uint32_t timestamp);
template float INDI_DelayFilter<float, 7>::delay_output(void);
template void INDI_DelayFilter<float, 7>::reset(void);

template void INDI_DelayFilter<float, 9>::update(float sample, uint32_t timestamp);
template float INDI_DelayFilter<float, 9>::delay_output(void);
template void INDI_DelayFilter<float, 9>::reset(void);

template void INDI_DelayFilter<float, 11>::update(float sample, uint32_t timestamp);
template float INDI_DelayFilter<float, 11>::delay_output(void);
template void INDI_DelayFilter<float, 11>::reset(void);