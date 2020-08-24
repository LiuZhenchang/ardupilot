/*
This program realizes a none delay derivative filter.The derivation of flight states is calculated by
the method designed by holoborodko.
See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
The linear programming method is used to forcast the current derivatives. 
*/
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"
#include "DerivativeFilter.h"
// The type of data being filtered is float.
// The number of elements in the filter is 9.

class INDI_NoneDelayDerivativeFilter
{
public:
	// constructor
	INDI_NoneDelayDerivativeFilter() :
		a(0), b(0)
	{
		reset();
	};

	// update - Add a new raw value to the filter, and calculate the derivative
	// states by Derivative Filter
	void update(float sample, uint32_t timestamp);

	// calculate a and b in linear programming method u=a*t+b
	void update_linear_programming();

	// return the current none delay derivative value
	float forcast();

	// reset - clear the filter
	virtual void        reset();

private:
	bool            _new_data;
	float           _last_forcast;
	float			d_samples[8];
	float			a;	// Linear programming u=a*t+b
	float			b;	// Linear programming u=a*t+b
	// microsecond timestamps for samples. This is needed
	// to cope with non-uniform time spacing of the data
	uint32_t        _timestamps[9];
	float        timestamps_new[9];

	DerivativeFilterFloat_Size9		dstate9;
	DerivativeFilterFloat_Size7		dstate7;
	DerivativeFilterFloat_Size5		dstate5;
	DerivativeFilterFloat_Size3		dstate3;
};


