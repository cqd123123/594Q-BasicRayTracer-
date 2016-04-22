#ifndef RAY_H
#define RAY_H

#include <Eigen/Eigen>
#include "util.h"

using namespace Eigen;

class Ray
{
public:
	Ray(Vector3f ori, Vector3f Dir, float _t1, float _t2):Origin(ori), Direction(Dir), tmin(_t1), tmax(_t2)
	{
		//Specturm.setZero();
	}
	Ray(){
		Origin.setZero();
		Direction.setZero();
		//Specturm.setZero();
		tmin = RAY_EPSILON;
		tmax = FLT_MAX;
		Depth = 0;
		refindex = 1.f;
	}
	~Ray(){;}

	inline Vector3f operator()(float t) const
	{ 
		return Origin + t*Direction;
	};

	Vector3f position(float t) const
	{
		//direction is normalized
	//	t = clamp(t,tmin, tmax);
		return Origin + t*Direction;
	}

//private:
	Vector3f Origin;
	Vector3f Direction;
	float tmin;	// two param define a line segement
	float tmax;
	float refindex;
	int Depth;
};


#endif