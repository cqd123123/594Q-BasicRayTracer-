#include "util.h"


bool is_vec3_nan(const Vector3f& in)
{
	return !(in(0) == in(0) && in(1)==in(1) && in(2) == in(2));
}

bool is_vec3_minus(const Vector3f & in)
{
	if( in(0) < 0.f || in(1) < 0.f || in(2) < 0.f)
		return true;
	else return false;
}