#include "light.h"

void PointLight::Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const
{
	Vector3f rayori = iset.HitPos + RAY_EPSILON* iset.SufNormal;
	Vector3f raydir = (position - rayori);
	ray.tmax = raydir.norm();
	raydir.normalize();
	ray.tmin = 0.f;
	ray.Direction = raydir;
	ray.Origin = rayori;
	*pdf = 1.f;
}

Vector3f PointLight::LI(const Intersect& iset) const
{
	Vector3f result = Color;
	float dist = (position - iset.HitPos).norm();
	float fatt = min(1.f,1.f/(Attc1 + Attc2 * dist + Attc3 * dist * dist));
	return result*fatt;
}

void DirecionLight::Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const
{
	Vector3f raypos = iset.HitPos + RAY_EPSILON*iset.SufNormal;
	Vector3f raydir = direction;
	ray.tmax = FLT_MAX;
	ray.tmin = 0.f;
	ray.Origin = raypos;
	ray.Direction = raydir;
	*pdf = 1.f;
}

void SpotLight::Generate_shadowRay(Ray& ray,const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const
{
	Vector3f rayori = iset.HitPos + RAY_EPSILON*iset.SufNormal;
	Vector3f raydir = (position - rayori);
	ray.tmax = raydir.norm();
	raydir.normalize();
	ray.Origin = rayori;
	ray.Direction = raydir;
	ray.tmin = 0.f;
	*pdf = 1.f;
}
Vector3f SpotLight::LI(const Intersect& iset) const
{
	Vector3f pos = iset.HitPos;
	Vector3f _d  = pos - position;
	_d.normalize();
	float val = fabs(_d.dot(Direction));
	if(val < cosCutang)
	{
		return Vector3f(0.f,0.f,0.f);
	}
	else return Color;
}

Vector3f Env_map::LI(const Vector3f& dir) const
{

	float p_x = dir(0)*0.5f + 0.5f; 
	float p_y = dir(1)*0.5f + 0.5f;
	p_x = clamp(p_x, 0.f,0.9f);
	p_y = clamp(p_y, 0.f,0.9f);
	if((dir(0)*dir(0) + dir(1)*dir(1)) > 1.f)
	{
		std::cout<<dir.norm()<<std::endl;
	}
	int xpos    = (int)( p_x * xRes);
	int ypos    = (int) (p_y * yRes);

	Vector3f result;
	result(0) = image(xpos, ypos,0);
	result(1) = image(xpos, ypos,1);
	result(2) = image(xpos, ypos,2);


	return result;//Vector3f(1.f,1.f,1.f);
}

Vector3f Env_map::LI(const Intersect& iset ) const
{
	Vector3f s_dir = iset.shadow_Dir;
	Vector3f _normal = iset.SufNormal;
	Vector3f O_dir = s_dir;//I_dir - 2.f*(I_dir.dot(_normal))*_normal;
	//get theta, phi;
// 	float Theta = acosf(O_dir(2));		//[0 - PI]
// 	float Phi   = atan2f(O_dir(1), O_dir(0)); //[0 - 2*PI]
// 	float p_x   = Phi*0.5f / M_PI + 0.5;	//[0-1]
// 	
// 	float p_y   = Theta / M_PI;  //[0 -1]
	O_dir = O_dir*0.9;

	float p_x = O_dir(0)*0.5f + 0.5f;
	float p_y = O_dir(1)*0.5f + 0.5f;
	if((O_dir(0)*O_dir(0) + O_dir(1)*O_dir(1)) > 1.f)
	{
		std::cout<<O_dir.norm()<<std::endl;
	}
// 	p_x = clamp(p_x, 0.f,0.9999f);
// 	p_y = clamp(p_y, 0.f,0.9999f);
	int xpos    = (int)( p_x * xRes);
	int ypos    = (int) (p_y * yRes);
	
	Vector3f result;
	result(0) = image(xpos, ypos,0);
	result(1) = image(xpos, ypos,1);
	result(2) = image(xpos, ypos,2);

	
	return result;//Vector3f(1.f,1.f,1.f);
}
//need to rotate the cosine sample to world coordinates, using matrix product, faster way??
void Env_map::Generate_shadowRay(Ray& ray, const Intersect& iset, float* pdf, const  sampler2D* smp, const int sampleOffset) const
{
	Vector3f I_dir = iset.Incident_Dir;
	Vector3f _normal = iset.SufNormal;
	float shinness = 0.f;
	if(_normal.norm() != 0.f)
		shinness = iset.materl.shininess;
	Vector3f Direction;
// 
	if(shinness < 1.f && _normal.norm() != 0.f) //not specular
	{
		Vector2f samplePos = (*smp)(sampleOffset);
//		float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
//		float randy = ((float) rand()) / (float) RAND_MAX;
		Matrix3f Rot;
		Mat3RotationFromVec(Vector3f(0.f,0.f,1.f), _normal, Rot);
		CosineSampleSphere(Direction, samplePos(0), samplePos(1), *pdf);
		Direction = Rot * Direction;
		*pdf = Direction.dot(_normal);
		//std::cout<<*pdf<<std::endl;
		//Direction = iset.SufNormal;
	}
	else
	{
		Direction = I_dir - 2.f*(I_dir.dot(_normal))*_normal;
	}
	ray.Origin = iset.HitPos + RAY_EPSILON*iset.SufNormal;
	ray.Direction = Direction;
	ray.Direction.normalize();
	ray.tmin = 0.f;
	ray.tmax = FLT_MAX;
}

Vector3f QuadLight::LI(const Vector3f& dir) const
{
	return Vector3f(0.f,0.f,0.f);
}

Vector3f QuadLight::LI(const Intersect& iset) const
{
	Vector3f result = Color;
	float dist = (Centro - iset.HitPos).norm();
	float fatt = min(1.f,1.f/(Attc1 + Attc2 * dist + Attc3 * dist * dist));
	if(is_spot)
	{
		Vector3f pos = iset.HitPos;
		Vector3f Direction = ex.cross(ey);
		//std::cout<<Direction.dot(Vector3f(0.f,1.f,0.f))<<std::endl;
		Direction.normalize();
		Vector3f _d  = pos - Centro;
		_d.normalize();
		float val = fabs(_d.dot(Direction));
		//std::cout<<val<<std::endl;
		if( val < cutAngle)
		{
			result.setZero();
		}
		return result;
	}
	else
		return result*fatt;
}

void QuadLight::Generate_shadowRay(Ray& ray, const Intersect& iset, float* pdf, const sampler2D* smp, const int sampleOffset) const
{
	Vector3f I_dir = iset.Incident_Dir;
	Vector3f _normal = iset.SufNormal;
	float shinness = 0.f;
	if(_normal.norm()!=0.f)
		shinness = iset.materl.shininess;
	Vector3f Direction;
	Vector3f rayori = iset.HitPos + RAY_EPSILON* iset.SufNormal;
	// 
	if(shinness < 1.f) //not specular
	{
		Vector2f samplePos = (*smp)(sampleOffset);
		float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
		float randy = ((float) rand()) / (float) RAND_MAX;
		//sample on light sources//
		Vector3f LiPos = randx * ex + randy * ey + v2;
		Direction = LiPos - rayori;
		//Direction.normalize();
		*pdf = 1.f;
	}
	else
	{
		Direction = I_dir - 2.f*(I_dir.dot(_normal))*_normal;
	}
	ray.Origin = rayori;
	ray.tmax = Direction.norm();
	Direction.normalize();
	ray.Direction = Direction;
	ray.tmin = 0.f;
}