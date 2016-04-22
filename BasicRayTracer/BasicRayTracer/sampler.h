#ifndef SAMPLER_H
#define SAMPLER_H

#include <stdlib.h>
#include <cassert>
#include <iostream>

#include "Eigen/Eigen"
using namespace Eigen;


//a sampler for one pixel, sample offset range from [0-1]^2
class sampler2D{

public:

	//x sample in x axis, y sample in y axis
	inline Vector2f& operator()(int x, int y) { 
		assert(x<x_spp&&x>=0&&y<y_spp&&y>=0);
		return samples[x + y*x_spp]; 
	};
	inline Vector2f operator()(int x) const
	{
		assert(x<Total);
		return samples[x];
	}
	int get_xspp() const {return x_spp;}
	int get_yspp() const {return y_spp;}
	int totalSamples() {return x_spp*y_spp;}
	sampler2D(int numx_spp, int numy_spp):x_spp(numx_spp), y_spp(numy_spp)
	{
		samples = (Vector2f*)malloc(sizeof(Vector2f)*x_spp*y_spp);
		memset(samples,0x00, sizeof(Vector2f)*x_spp*y_spp);
		Total = x_spp*y_spp;
	}

	void GenerateRndStratifiedSmps();

	void Debug()
	{
		std::cout<<"sampler debug"<<std::endl;
		for(int i=0;i<x_spp*y_spp;i++)
			std::cout<<samples[i]<<std::endl;
	}
private:
	int Total;
	int x_spp;
	int y_spp;
	Vector2f* samples;
};
#endif