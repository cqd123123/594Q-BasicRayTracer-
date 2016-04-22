#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Eigen>
#include <cassert>
#include <math.h>
#include "scene_io.h"

using namespace Eigen;

#define RAY_EPSILON 1e-3f
#define RAY_EPSILON_2 1e-4f
#define INV_PI     0.31830988618379067154f
#define INV_TWOPI  0.15915494309189533577f
#define INV_FOURPI 0.07957747154594766788f
#ifndef TRILININTERPOLATE
#define TRILININTERPOLATE(in1, in2, in3, w1, w2, w3) in1*w1 + in2*w2 + in3*w3
#endif

//#define INTERSECTION_SHADER
//#define CHECKBORD_SHADER

//left hand rule to rotate

//bool is_nan(float x) { return isnan(x);}

inline Matrix4f Mat4FromQuaterion( Vector3f axis, float angle_in_radian)
{
	Quaternion<float> q;
	q = AngleAxis<float>(angle_in_radian, axis);
	Matrix3f r3 = q.toRotationMatrix ();
	Matrix4f rotataion;
	rotataion.setZero();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
			rotataion(i,j) = r3(i,j);
	}
	rotataion(3,3) = 1.0;	//!! !!
	return rotataion;
}

inline Matrix3f Mat3FromQuaterion(Vector3f axis, float angle_in_radian)
{
	Quaternion<float> q;
	q = AngleAxis<float>(angle_in_radian, axis);
	return q.toRotationMatrix ();
}

inline bool component_smaller(Vector3f arg1, Vector3f arg2)
{
	return arg1(0)<=arg2(0)&&arg1(1)<=arg2(1)&&arg1(2)<=arg2(2);
}
inline bool component_larger(Vector3f arg1, Vector3f arg2)
{
	return arg1(0)>=arg2(0)&&arg1(1)>=arg2(1)&&arg1(2)>=arg2(2);
}
//return componentwise smaller one
inline Vector3f min_comp(Vector3f a, Vector3f b)
{
	return component_smaller(a,b)?a:b;
}
//return componentwise larger one
inline Vector3f max_comp(Vector3f a, Vector3f b)
{
	return component_smaller(a,b)?b:a;
}
inline Vector3f comp_mult(Vector3f a, Vector3f b)
{
	return Vector3f(a(0)*b(0), a(1)*b(1), a(2)*b(2));
}
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
// template<typename Ta>
// inline Ta  max(Ta b, Ta a)
// {
// 	return b>a ? b:a;
// }
// template<typename T> T inline  min(T b, T a)
// {
// 	return b<a ? b:a;
// }
template <typename T> void inline swap(T &b, T &a)
{
	T temp;
	temp = a;
	a = b;
	b = temp;
}
inline void SetMatToIdentity(Matrix3f & mat)
{
	mat.setZero();
	mat(0,0) = 1.f;
	mat(1,1) = 1.f;
	mat(2,2) = 1.f;
}
template<typename T> T inline clamp(T in, T _min, T _max)
{
	if(in<_min)
		return _min;
	if(in>_max)
		return _max;
	return in;
}

inline float sign2D (Vector2f p1, Vector2f p2, Vector2f p3)
{
	return (p1(0) - p3(0)) * (p2(1) - p3(1)) - (p2(0) - p3(0)) * (p1(1) - p3(1));
}

inline bool PointInTriangle2D (Vector2f pt, Vector2f v1, Vector2f v2, Vector2f v3)
{
	bool b1, b2, b3;

	b1 = sign2D(pt, v1, v2) < 0.0f;
	b2 = sign2D(pt, v2, v3) < 0.0f;
	b3 = sign2D(pt, v3, v1) < 0.0f;

	return ((b1 == b2) && (b2 == b3));
}
inline float AreaTrangle(Vector2f A, Vector2f B, Vector2f C)
{
	return fabs(A(0)*(B(1)-C(1)) + B(0)*(C(1)-A(1)) + C(0)*(A(1)-B(1)))*0.5f;
}
inline Vector3f BayercentricInterpolation (Vector2f pos, Vector2f A1, Vector2f A2, Vector2f A3)
{
	if(!PointInTriangle2D(pos,A1,A2,A3))
		return Vector3f(0,0,0);
	else{

		float Area = AreaTrangle(A1,A2,A3);
		if(Area == 0) return Vector3f(0,0,0);
		float w1 = AreaTrangle(pos,A2,A3)/Area;
		float w2 = AreaTrangle(pos,A1,A3)/Area;
		float w3 = 1.f - w1 - w2;
		return Vector3f(w1,w2,w3);
	}
}
//pos : the position of interpolated point, val1 cord1... is value and coordinate associated with two points
template<typename T>
inline T LinearInterpolate( float pos, float cord1, float cord2, T val1, T val2){
	//assert( pos>=cord1&&pos<=cord2 );

	if(pos>=cord1&&pos<=cord2)
	{
		float dist = fabs(cord2 -cord1);
		float w1   = fabs(cord2 - pos)/dist;
		return val1*w1 + val2*(1.f-w1);
	}
	else if(pos<cord1)
		return val1;
	else return val2;
}
//calculate matrix rotate from original vec to to vec
inline void Mat3RotationFromVec(const Vector3f& OriVec, const Vector3f& ToVec, Matrix3f& Rot)
{
	Vector3f a = OriVec;
	Vector3f b = ToVec;
	a.normalize(); b.normalize();
	if(a == b)
	{
		SetMatToIdentity(Rot);
		return;
	}
	if(a == -b)
	{
		Rot.setZero();
		Rot(0,0) = -1.f;
		Rot(1,1) = -1.f;
		Rot(2,2) = -1.f;
		return;
	}
	Vector3f axis = a.cross(b);
	float s = axis.norm();
	float c = a.dot(b);
	float coef = (1 - c) / (s*s);
	float v1 = axis(0);
	float v2 = axis(1);
	float v3 = axis(2);

	Rot(0,0) = 1.f + coef*(-v3*v3 - v2*v2);
	Rot(0,1) = -v3 + coef*v2*v1;
	Rot(0,2) = v2 + coef*v3*v1;
	Rot(1,0) = v3 + coef*v1*v2;
	Rot(1,1) = 1.f + coef*(-v3*v3 - v1*v1);
	Rot(1,2) = -v1 + coef*v3*v2;
	Rot(2,0) = -v2 + coef*v3*v1;
	Rot(2,1) = v1 + coef*v2*v3;
	Rot(2,2) = 1.f + coef*(-v2*v2 - v1*v1);

}
inline Vector3f GetTrangleNormal(Vector3f A, Vector3f B, Vector3f C)
{
	Vector3f _result;
	_result(0) = (B(1) - A(1))*(C(2) - A(2)) - (C(1) - A(1))*(B(2) - A(2));
	_result(1) = (B(2) - A(2))*(C(0) - A(0)) - (C(2) - A(2))*(B(0) - A(0));
	_result(2) = (B(0) - A(0))*(C(1) - A(1)) - (C(0) - A(0))*(B(1) - A(1));
	_result.normalize();
	return _result;
}
//a, b, c ax^2 + bx + c =0; t0 and t1 are roots
inline bool SovleQuadric(float A, float B, float C, float* t0, float * t1){
	float val = B*B - 4.f*A*C;
	if(val < 0) return false;
	float rootDis = sqrtf(val);
	float q;
	if(B < 0) q = -0.5f*(B - rootDis);
	else q = -0.5f*(B+ rootDis);
	*t0 = q/A;
	*t1 = C / q;
	//
	if(*t0 > *t1)
		swap(*t0, *t1);
	return true;
}
//assign left to right
inline void PointAssign(Point& left,const Point& right){
	left[0] = right[0];
	left[1] = right[1];
	left[2] = right[2];
}
inline void PointToVector3f(const Point& in, Vector3f& out){
	out(0) = in[0];
	out(1) = in[1];
	out(2) = in[2];
}
inline void getMaterialColor(const MaterialIO& material, Vector3f& ambcol, Vector3f& difcol, Vector3f& speccol, Vector3f& emiscol){
	PointToVector3f(material.ambColor,ambcol);
	PointToVector3f(material.diffColor,difcol);
	PointToVector3f(material.specColor,speccol);
	PointToVector3f(material.emissColor,emiscol);
}

inline void getTriVetpos(const VertexIO& vert, Point& pt)
{
	pt[0] = vert.pos[0];
	pt[1] = vert.pos[1];
	pt[2] = vert.pos[2];
}
inline void getTriVetNormal(const VertexIO& vert, Vector3f& n)
{
	n(0) = vert.norm[0];
	n(1) = vert.norm[1];
	n(2) = vert.norm[2];
}
inline void getTriVetPos(const VertexIO& vert, Vector3f& pt)
{
	pt(0) = vert.pos[0];
	pt(1) = vert.pos[1];
	pt(2) = vert.pos[2];
}

inline void MaterialAssign(const MaterialIO* incol, MaterialIO& out)
{
	for(int i=0;i<3;i++)
	{
		out.ambColor[i] = incol->ambColor[i];
		out.diffColor[i] = incol->diffColor[i];
		out.emissColor[i] = incol->emissColor[i];
		out.specColor[i] = incol->specColor[i];
	}
	out.ktran = incol->ktran;
	out.shininess = incol->shininess;
}

inline void MaterialInterpolate(const MaterialIO* in1, const MaterialIO* in2, const MaterialIO* in3,
								const float w1,const float w2, const float w3, MaterialIO& out)
{
	for(int i=0;i<3;i++)
	{
		out.ambColor[i] = TRILININTERPOLATE(in1->ambColor[i], in2->ambColor[i], in3->ambColor[i], w1,w2,w3);
		out.diffColor[i] = TRILININTERPOLATE(in1->diffColor[i], in2->diffColor[i], in3->diffColor[i], w1, w2, w3);
		out.emissColor[i] = TRILININTERPOLATE(in1->emissColor[i], in2->emissColor[i], in3->emissColor[i], w1, w2, w3);
		out.specColor[i] = TRILININTERPOLATE(in1->specColor[i], in2->specColor[i], in3->specColor[i], w1, w2, w3);
	}
	out.ktran = TRILININTERPOLATE(in1->ktran, in2->ktran, in3->ktran, w1, w2, w3);
	out.shininess = TRILININTERPOLATE(in1->shininess, in2->shininess, in3->shininess, w1, w2, w3);
}
inline void ColorAssign(Point& color , const Vector3f& col_in)
{
	color[0] = col_in(0);
	color[1] = col_in(1);
	color[2] = col_in(2);
}
//u, v range from 0-1, distributed evenly, out is the output vector
inline void UniformSampleHemiSphere(Vector3f& out, const float u, const float v)
{
	out(0) = cos(2*M_PI*v) * sqrtf(1.f - u*u);
	out(1) = sin(2*M_PI*v) * sqrtf(1.f - u*u);
	out(2) =  u;
}

inline void ConcentricSampleDisk(float* x, float* y, const float u, const float v)
{
	float sx = 2.f*u - 1.f;  // [-1 1];
	float sy = 2.f*v - 1.f;  // [-1 1];
	float r, theta;

	//map to squares
	if (sx == 0.0 && sy == 0.0) {
		*x = 0.0;
		*y = 0.0;
		return;
	}
	if (sx >= -sy) {
		if (sx > sy) {
			// Handle first region of disk
			r = sx;
			if (sy > 0.0) theta = sy/r;
			else          theta = 8.0f + sy/r;
		}
		else {
			// Handle second region of disk
			r = sy;
			theta = 2.0f - sx/r;
		}
	}
	else {
		if (sx <= sy) {
			// Handle third region of disk
			r = -sx;
			theta = 4.0f - sy/r;
		}
		else {
			// Handle fourth region of disk
			r = -sy;
			theta = 6.0f + sx/r;
		}
	}
	theta *= M_PI / 4.f;
	*x = r * cosf(theta);
	*y = r * sinf(theta);
}
//out is the output vector, u, v uniform distributed in [0,1]
inline void CosineSampleSphere(Vector3f& out, const float u, const float v, float& pdf)
{
	//always keep in mind that sqrtf in this might be minus
	ConcentricSampleDisk(&out(0), &out(1), u,v);
	out(2) = sqrtf(max(0.f,1.f - out(0)*out(0) - out(1)*out(1)));
	pdf = out(2);
}
// divd / divs
inline Vector3f comp_divide(const Vector3f& divd, const Vector3f& divs)
{
	return Vector3f(divd(0) / divs(0), divd(1) / divs(1), divd(2) / divs(2));
}
bool is_vec3_nan(const Vector3f& in);
bool is_vec3_minus(const Vector3f & in);

inline Vector3f SphericalDirection(float sintheta,
								 float costheta, float phi) {
									 return Vector3f(sintheta * cosf(phi),
										 sintheta * sinf(phi),
										 costheta);
}


inline bool SameHemisphere(const Vector3f &w, const Vector3f &wp) {
	return w(2) * wp(2) > 0.f;
}

inline Vector3f Vec3fExp(const Vector3f& in)
{
	Vector3f result;
	result(0) = exp(in(0));
	result(1) = exp(in(1));
	result(2) = exp(in(2));
	return result;
}

inline float Lerp(float t, float v1, float v2) {
	return (1.f - t) * v1 + t * v2;
}

#endif