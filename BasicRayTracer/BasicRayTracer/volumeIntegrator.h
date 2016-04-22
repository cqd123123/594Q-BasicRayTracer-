#ifndef VOLUMEINTEGRATOR_H
#define VOLUMEINTEGRATOR_H

#include "Eigen/Eigen"
#include "Ray.h"
#include "Gemetry.h"
#include "sampler.h"
#include "Transform.h"
#include "light.h"
#include "BVH.h"
#include "render.h"


using namespace Eigen;
class Renderer;
class VolumeIntegrator
{
public:
	// VolumeIntegrator Interface
	virtual Vector3f Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f* T) const = 0;
	virtual Vector3f Transmittance(const Scenes* scenes,const std::vector<Light*>& _ligs, const Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const = 0;
};

class Emission : public VolumeIntegrator{
public:
	Emission(float ss):stepSize(ss){;}

	Vector3f Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f* T) const;
	Vector3f Transmittance(const Scenes* scenes,const std::vector<Light*>& _ligs, const Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const;
private:
	float stepSize;
	int tauSampleOffset, scatterSampleOffset;
};


class SingleScatter : public VolumeIntegrator
{
public:
	SingleScatter(float ss): stepSize(ss){;}

	Vector3f Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f* T) const;
	Vector3f Transmittance(const Scenes* scenes,const std::vector<Light*>& _ligs,const Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const;
private:
	float stepSize;
	int tauSampleOffset, scatterSampleOffset;
};
#endif