#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "Eigen/Eigen"
#include "Ray.h"

using namespace Eigen;

//bounding box classs


class BBox{
public:
	BBox(){
		Boxmin = Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
		Boxmax = Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	}
	BBox(Vector3f minc, Vector3f maxc):Boxmax(maxc), Boxmin(minc)
	{
		;
	}
	bool Intersect(const Ray& ray, float* tHit0,float* tHit1) const;
	bool Inside(Vector3f p) const{
		return (p(0) >= Boxmin(0) && p(1)>= Boxmin(1) && p(2) >= Boxmin(2) 
			&& p(0) <= Boxmax(0) && p(1) <= Boxmax(1) && p(2) <= Boxmax(2));
	}
	Vector3f getCentroid() const {return (Boxmax+Boxmin)*0.5f;}
	float SurfaceArea() const;
	//union with this bounding box with input in
	//inline void Union(BBox& in);
	int MaxExtent() const;
	Vector3f Offset(const Vector3f &p) const {
		return Vector3f((p(0) - Boxmin(0)) / (Boxmax(0) - Boxmin(0)),
			(p(1) - Boxmin(1)) / (Boxmax(1) - Boxmin(1)),
			(p(2) - Boxmin(2)) / (Boxmax(2) - Boxmin(2)));
	}
	Vector3f Boxmin;
	Vector3f Boxmax;
};
BBox Union(const BBox &b, const BBox &b2);
BBox Union(const BBox &b, const Vector3f& p);
#endif
