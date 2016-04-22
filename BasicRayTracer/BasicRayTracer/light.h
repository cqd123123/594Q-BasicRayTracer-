#ifndef LIGHT_H
#define LIGHT_H

#include <Eigen/Eigen>
#include <vector>
#include <math.h>
#include "scene_io.h"
#include "Gemetry.h"
#include <cassert>
#include "CImg.h"
#include "sampler.h"

using namespace Eigen;

using namespace cimg_library;

#define Attc1 0.25f
#define Attc2 0.10f
#define Attc3 0.025f



struct Lights{
	std::vector<LightIO*> _lights;
};
//delta lights : point light, directional light, spot light.
//non delta lights : env map
class Light{
public:
	virtual Vector3f LI( const Intersect& isec) const{
		std::cout<<"unimplemented light LI called "<<std::endl;
		exit(0);
		return Vector3f(0.f,0.f,0.f);
	}
	virtual Vector3f LI(const Vector3f& dir) const{
		std::cout<<"unimplemented light LI called "<<std::endl;
		exit(0);
		return Vector3f(0.f,0.f,0.f);
	}
	virtual void Generate_shadowRay(Ray& ray, const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const{;}
	bool is_Delta;
};
//Point position;	/* Position of a point light source		
//Vec direction;	/* Direction of a directional light source	*/
//Color color;	/* Color and intensity				*/

//Flt dropOffRate;	/* For spot lights, 0 <= x <= 1.  See man pages */
//Flt cutOffAngle;	/* Angle at which spot light intensity is zero  */

class PointLight :public Light{
public:
	PointLight(Vector3f pos, Vector3f col, float _droprate): position(pos), Color(col), dropOffRate(_droprate)
	{
		is_Delta = true;
	}
	void Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const;
	//output radiance
	Vector3f LI(const Intersect& iset) const;
	Vector3f LI(const Vector3f& dir) const {return Vector3f(0.f,0.f,0.f);}
private:
	Vector3f position;
	Vector3f Color;
	float dropOffRate;
};
class DirecionLight : public Light{
public:
	DirecionLight(Vector3f _direct, Vector3f col, float _droprate) : direction(_direct), Color(col), dropOffRate(_droprate)
	{
		is_Delta = true;
	}
	void Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf,  const sampler2D* smp, const int sampleOffset) const;

	//output radiance
	Vector3f LI(const Intersect& iset) const{
		return Color;
	}
	Vector3f LI(const Vector3f& dir) const {return Vector3f(0.f,0.f,0.f);}
private:
	Vector3f direction;
	Vector3f Color;
	float dropOffRate;
};
class SpotLight : public Light{
public:
	SpotLight(Vector3f pos,Vector3f col, Vector3f _dir, float _droprate, float _cutangle): position(pos), Color(col), Direction(_dir),
		dropOffRate(_droprate), cutOffAngle(_cutangle)
	{
		cosCutang = cosf(cutOffAngle);
		Direction.normalize();
		is_Delta = true;
	}
	void Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const;
	//output radiance
	Vector3f LI(const Intersect& iset) const;
	Vector3f LI(const Vector3f& dir) const {return Vector3f(0.f,0.f,0.f);}

private:
	Vector3f position;
	Vector3f Color;
	Vector3f Direction;
	float dropOffRate;
	float cutOffAngle;
	float cosCutang;
};

class Env_map: public Light {
public:
	Env_map(std::string name){
		
		image =  CImg<float>(name.c_str());
		xRes = image.width();
		yRes = image.height();
		is_Delta = false;

		for(int j=0;j<yRes;j++)
		{
			for(int i=0;i<xRes;i++)
			{
				for(int k=0;k<3;k++)
					image(i,j,k) = clamp(image(i,j,k), 0.f,1.f);
			}
		}
		//image.display();
	};
	//output radiance
	Vector3f LI(const Intersect& iset) const;
	Vector3f LI(const Vector3f& dir) const;
	void Generate_shadowRay(Ray& ray, const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const;
private:
	CImg<float> image;
	int xRes;
	int yRes;
};
/*
     ex
V2 ------------ V4
|               |
| ey            |
|               |
V1--------------V3
*/
class QuadLight: public Light
{
public:
	QuadLight(const Vector3f& vet1,const Vector3f& vet2, const Vector3f& vet3,const Vector3f& color):Color(color),
		v1(vet1), v2(vet2), v3(vet3)
	{
		v4 = v3 + v2 - v1;
		Centro = (v3 + v2)*0.5f;
		ex = v4 - v2;
		ey = v1 - v2;
		is_Delta = false;
		is_spot = false;
		cutAngle = 0.f;
	}
	Vector3f LI(const Intersect& iset) const;
	Vector3f LI(const Vector3f& dir) const;
	void Generate_shadowRay(Ray& ray, const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const;
	bool is_spot;
	float cutAngle;
private:
	Vector3f ex;
	Vector3f ey;
	Vector3f Centro;
	Vector3f v1;
	Vector3f v2;
	Vector3f v3;
	Vector3f v4;
	Vector3f Color;
};

#endif