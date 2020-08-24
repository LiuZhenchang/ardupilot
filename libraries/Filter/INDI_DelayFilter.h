/*
This program realizes a delay filter to ensure the time synchronization of control states  in INDI method
*/
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, uint8_t FILTER_SIZE>
class INDI_DelayFilter : public FilterWithBuffer<T, FILTER_SIZE>
{
public:
	// constructor
	INDI_DelayFilter() : FilterWithBuffer<T, FILTER_SIZE>() {
	};

	// update - Add a new raw value to the filter, but don't recalculate
	void update(T sample, uint32_t timestamp);

	// return the derivative value
	float delay_output(void);

	// reset - clear the filter
	virtual void        reset();

private:
	bool            _new_data;
	float           _last_output;

	// microsecond timestamps for samples. This is needed
	// to cope with non-uniform time spacing of the data
	uint32_t        _timestamps[FILTER_SIZE];
};

typedef INDI_DelayFilter<float, 3> INDI_DelayFilterFloat_Size3;
typedef INDI_DelayFilter<float, 5> INDI_DelayFilterFloat_Size5;
typedef INDI_DelayFilter<float, 7> INDI_DelayFilterFloat_Size7;
typedef INDI_DelayFilter<float, 9> INDI_DelayFilterFloat_Size9;
typedef INDI_DelayFilter<float, 11> INDI_DelayFilterFloat_Size11;