#include "sampler.h"

void sampler2D::GenerateRndStratifiedSmps()
{
	float _dx = 1.0f/x_spp;
	float _dy = 1.0f/y_spp;
	//place uniform in a grid
	for(int j=0;j<y_spp;j++)
	{
		for(int i=0;i<x_spp;i++)
		{
			float xspos = 0.5*_dx + i*_dx;
			float yspos = 0.5*_dy + j*_dy;
			//rnd jitter
			float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
			float randy = ((float) rand()) / (float) RAND_MAX;
			xspos = xspos + (randx - 0.5f)*_dx;				//in the range [-0.5dx, 0.5dx]
			yspos = yspos + (randy - 0.5f)*_dy;
			Vector2f samp_pos;
			samp_pos(0) = xspos;
			samp_pos(1) = yspos;
			samples[i + j*x_spp] = samp_pos;
		}
	}
}