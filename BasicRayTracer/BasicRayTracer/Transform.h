#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "Eigen/Eigen"
#include "util.h"
#include <math.h>
#include "Ray.h"
#include <iostream>
#include <cassert>

using namespace Eigen;

class TransformV {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		TransformV();

	void SetViewProjectMatrix(float angleOfView, float nearClip,  float farClip);

	//add transform vector directly to camera transform
	void TransformCamera(Vector3f dir);

	//left hand rule to rotate world vertecies
	void RotateCamera(Vector3f axis, float angle_rad)
	{
		RotationMatrix = Mat4FromQuaterion(axis, angle_rad);
		World2CameraMatrix = RotationMatrix * World2CameraMatrix;
		TotalTransMatrix = ViewPointMatrix * World2CameraMatrix;
	}
	//position of the camera, point the camera is looking at, and up vector
	void LookAt(Vector3f pos,Vector3f look,Vector3f up);

	inline void TransVertex(Vector3f in, Vector3f& out);

	inline void TransVertexNew(Vector3f& in, Vector3f& out);

	inline bool OutofImagePlane(Vector3f in)
	{
		if(in(0)<-1 || in(0)>1 || in(1)<-1 || in(1)>1 || in(2) < 0 || in(2) > 1)
			return true;
		else return false;

	}
	//whether the depth of a quad lies in [nearclip, farclip]
	inline bool QuadOutofFrustum(Vector3f* _in);

	inline Vector2f Translate_To_screen(Vector3f in, int _w, int _h)
	{
		;
	}
	Matrix4f getViewProjectMartix() {return ViewPointMatrix;}
	Matrix4f getWorld2CameraMatrix() {return World2CameraMatrix;}
	Vector3f getCameraPos(){return CameraPos;}
	void printMatrix()
	{
		std::cout<<TotalTransMatrix<<std::endl;
	}
	//imgx, coordinate for sample position on image cord, scaled to [0,1]
	float GenerateRay(const float& imgx,const float &imgy, Ray* ray, const float & c, const int _w, const int _h) const;

	//transform ray to world coordinates
	void Camera2World(const Ray& _in, Ray& _out) const;
	void EnableDOF(float FocuesLength)
	{
		DOF_Enableed = true;
		Focus_D = FocuesLength;
	}

private:
	Matrix4f ViewPointMatrix;
	Matrix4f World2CameraMatrix;
	Matrix4f RotationMatrix;
	Matrix4f TotalTransMatrix;
	Vector3f CameraPos;
	float RaDFov;
	bool DOF_Enableed;
	float Focus_D;	//foucus distance
};
#endif