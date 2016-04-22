#include "BoundingBox.h"

bool BBox::Intersect(const Ray& ray, float* tHit0,float* tHit1) const
{
	float t0 = ray.tmin;
	float t1 = ray.tmax;
	for(int i=0;i<3;i++)
	{

		float invRayDir = 1.f / ray.Direction[i];
		float tNear = (Boxmin[i] - ray.Origin[i]) * invRayDir;
		float tFar  = (Boxmax[i] - ray.Origin[i]) * invRayDir;
		// Update parametric interval from slab intersection $t$s
		if (tNear > tFar) swap(tNear, tFar);
		t0 = tNear > t0 ? tNear : t0;
		t1 = tFar  < t1 ? tFar  : t1;
		if (t0 > t1) return false;
	}
	if (tHit0) *tHit0 = t0;
	if (tHit1) *tHit1 = t1;
	return true;
}
BBox Union(const BBox &A, const BBox &B) {
	BBox ret;
	ret.Boxmin(0) = min(A.Boxmin(0), B.Boxmin(0));
	ret.Boxmin(1) = min(A.Boxmin(1), B.Boxmin(1));
	ret.Boxmin(2) = min(A.Boxmin(2), B.Boxmin(2));
	ret.Boxmax(0) = max(A.Boxmax(0), B.Boxmax(0));
	ret.Boxmax(1) = max(A.Boxmax(1), B.Boxmax(1));
	ret.Boxmax(2) = max(A.Boxmax(2), B.Boxmax(2));
	return ret;
}
BBox Union(const BBox &B, const Vector3f &p){
	BBox ret;
	ret.Boxmin(0) = min(B.Boxmin(0), p(0));
	ret.Boxmin(1) = min(B.Boxmin(1), p(1));
	ret.Boxmin(2) = min(B.Boxmin(2), p(2));
	ret.Boxmax(0) = max(B.Boxmax(0), p(0));
	ret.Boxmax(1) = max(B.Boxmax(1), p(1));
	ret.Boxmax(2) = max(B.Boxmax(2), p(2));
	return ret;
}
float BBox::SurfaceArea() const
{
	Vector3f d = Boxmax - Boxmin;
	return  2.f*(d(0)*d(1) + d(1)*d(2) + d(0)*d(2)) ;
}
int BBox::MaxExtent() const
{
	Vector3f d = Boxmax - Boxmin;
	if(d(0) > d(1) && d(0) > d(2))
		return 0;
	else if(d(1) > d(2))
		return 1;
	else
		return 2;
}