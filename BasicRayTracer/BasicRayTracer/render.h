#ifndef RENDER_H
#define RENDER_H

#include <Eigen/Eigen>
#include <math.h>
#include "Ray.h"
#include "Gemetry.h"
#include "sampler.h"
#include "Transform.h"
#include "light.h"
#include "BVH.h"
#include "volumeIntegrator.h"

#define AMBIENT_ATTENTUATION_FACTOR 1.0
#define DIFFUSE_ATTEN_FACTOR 1.0
#define RAY_MAX_DEPTH 5
#define REFLECTION_ATTEN_FACTOR 0.4
#define SPEC_POW_FACTOR 64
#define REFRACTION_INDEX 1.5
#define SPEC_ATTENUATION_FACTOR 0.00


using namespace Eigen;
class VolumeIntegrator;

class Renderer {
public:
	// Renderer Interface
	virtual ~Renderer(){ volumeInt = NULL;};
	virtual Vector3f Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const = 0;
	virtual Vector3f AreaLi(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const = 0;
	static Vector3f Uniform_Sample_All_light( const Scenes* scenes,const std::vector<Light*>& _ligs, const Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render);
	static Vector3f Uniform_Sample_One_light(const Scenes* scenes,const Light* lig, const Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f& Shadow_dir, const int sample_offset);
	//get incident radiance at a specific position.
	static Vector3f Light_Li_Position(const Scenes* scenes, const Light* lig, const Vector3f& position, const Vector3f& Direction,
		const sampler2D *sample, const int sample_offset, Ray& shray);

	
	VolumeIntegrator* volumeInt;
};
#endif