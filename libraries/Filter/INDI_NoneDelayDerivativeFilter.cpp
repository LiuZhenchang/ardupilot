/*
This program realizes a none delay derivative filter.The derivation of flight states is calculated by
the method designed by holoborodko.
See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
The linear programming method is used to forcast the current derivatives.
*/

#include <inttypes.h>
#include "../AP_Math/AP_Math.h"
#include "INDI_NoneDelayDerivativeFilter.h"

void INDI_NoneDelayDerivativeFilter::update(float sample, uint32_t timestamp)
{
	uint8_t FILTER_SIZE = 9;
	int count;

	if (_timestamps[FILTER_SIZE - 1] == timestamp) {
		// this is not a new timestamp - ignore
		return;
	}

	// add timestamp
	for (count = 0; count < (FILTER_SIZE - 1); count++) {
		_timestamps[count] = _timestamps[count + 1];
	}
	_timestamps[FILTER_SIZE - 1] = timestamp;

	//update every derrivative filter
	dstate9.update(sample, timestamp);
	dstate7.update(sample, timestamp);
	dstate5.update(sample, timestamp);
	dstate3.update(sample, timestamp);

	//update d_samples
	for (count = 0; count < (FILTER_SIZE - 1) / 2; count++) {
		d_samples[count] = d_samples[count + 1];
	}
	d_samples[4] = dstate9.slope() * 1.0e3;
	d_samples[5] = dstate7.slope() * 1.0e3;
	d_samples[6] = dstate5.slope() * 1.0e3;
	d_samples[7] = dstate3.slope() * 1.0e3;

	update_linear_programming();

	_new_data = true;
}

void INDI_NoneDelayDerivativeFilter::update_linear_programming()
{
	uint8_t FILTER_SIZE = 9;
	int count;

	// define timestamps_new[0]=0, let timestamps_new[0] starts from zero; 	
	for (count = 0; count <= (FILTER_SIZE - 1); count++) {
		timestamps_new[count] = float((_timestamps[count] - _timestamps[0]) / 1.0e3);
	}
	// Linear programming u=a*t+b
	// u repersents d_samples, b repersents timestamps_new
	// calculate sum element in a 	
	float sigma1 = 0;	// Sigma(t*du)
	float sigma2 = 0;	// Sigma(t)*Sigma(u)
	float sigma3 = 0;	// Sigma(t^2)
	float sigma4 = 0;	// Sigma(t)
	for (count = 0; count < (FILTER_SIZE - 1); count++) {
		sigma1 = sigma1 + timestamps_new[count] * d_samples[count];				// Sigma(t*du)
		float temp = 0;
		for (int i = 0; i < (FILTER_SIZE - 1); i++) {
			temp = temp + d_samples[i];
		}
		sigma2 = sigma2 + timestamps_new[count] * temp;						// Sigma(t)*Sigma(u)
		sigma3 = sigma3 + timestamps_new[count] * timestamps_new[count];	// Sigma(t^2)
		sigma4 = sigma4 + timestamps_new[count];							// Sigma(t)
	}
	// calculate a
	if (sigma3 == 0 && sigma4 == 0) { a = 0; }
	else { a = (sigma1 - sigma2 / (FILTER_SIZE - 1)) / (sigma3 - sigma4 * sigma4 / (FILTER_SIZE - 1)); }

	// calculate sum element in b
	float sigma5 = 0;	// Sigma(u-a*t)
	for (count = 0; count < (FILTER_SIZE - 1); count++) {
		sigma5 = sigma5 + d_samples[count] - a * timestamps_new[count];
	}
	// calculate b
	b = sigma5 / (FILTER_SIZE - 1);
}
float INDI_NoneDelayDerivativeFilter::forcast()
{
	uint8_t FILTER_SIZE = 9;
	float results;
	return results = a * timestamps_new[FILTER_SIZE - 1] + b;
}


void INDI_NoneDelayDerivativeFilter::reset(void)
{
	// clear samples buffer
	uint8_t FILTER_SIZE = 9;
	for (int8_t i = 0; i < FILTER_SIZE; i++) {
		d_samples[i] = 0;
		_timestamps[i] = 0;
	}
}