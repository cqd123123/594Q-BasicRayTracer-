#include "volumeIntegrator.h"
#include "util.h"


Vector3f Emission::Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
					  const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f* T) const
{
	Vector3f result(0.f,0.f,0.f);
	//stuff

	VolumeRegion *vr = scenes->volume;
	float t0=0.f, t1=0.f;
	if (!vr || !vr->IntersectP(ray, &t0, &t1) || (t1-t0) == 0.f) {
		*T = Vector3f(1.f, 1.f,1.f);
		return result;
	}
	
	int nSamples = (int)ceilf(((t1-t0) / stepSize));
	float step = (t1 - t0) / nSamples;
	Vector3f Tr(1.f,1.f,1.f);
	Vector3f p = ray(t0);
	Vector3f pPrev;
	Vector3f w = -ray.Direction;
	float offset = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
	while(offset == 0.f)
		offset = ((float) rand () ) / (float) RAND_MAX;

	t0 += offset;

	Vector3f Lv(0.f,0.f,0.f);

	for (int i = 0; i < nSamples; ++i, t0 += step) {
		// Advance to sample at _t0_ and update _T_
		pPrev = p;
		p = ray(t0);
		Vector3f direction = p - pPrev;
		//if(direction.norm() == 0.f) std::cout<<"norm zero"<<offset<<" " <<t0<<std::endl;
		direction.normalize();
		Ray tauRay(pPrev, direction, 0.f, 1.f);
		float rndOffset = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
		Vector3f stepTau = vr->tau(tauRay, .5f * stepSize, rndOffset);
		
		Vector3f exptau = Vec3fExp( - stepTau);
		//Tr *= Vec3fExp(-stepTau);
		Tr = comp_mult(Tr, exptau);

		// Possibly terminate ray marching if transmittance is small
		if (Tr(1) < 1e-3) {
			const float continueProb = .5f;
			if ( ((float) rand()) / (float) RAND_MAX > continueProb) {
				Tr.setZero();
				break;
			}
			Tr /= continueProb;
		}

		// Compute emission-only source term at _p_
		Lv += comp_mult(Tr, vr->Lve(p));
	}

	*T = Tr;
	return Lv * step;
}

Vector3f Emission::Transmittance(const Scenes* scenes,const std::vector<Light*>& _ligs,const Ray &ray, Intersect& isec, 
								 const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
	Vector3f result(0.f,0.f,0.f);
	if(!scenes->volume) return Vector3f(1.f,1.f,1.f);
	//stuff
	float step = stepSize;
	float offset = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
	Vector3f tau = scenes->volume->tau(ray, step, offset);
	result(0) = exp( - tau(0));
	result(1) = exp( - tau(1));
	result(2) = exp( - tau(2));
	return result;
}

Vector3f SingleScatter::Transmittance(const Scenes* scenes,const std::vector<Light*>& _ligs, const Ray &ray, Intersect& isec, 
									  const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
	Vector3f result(1.f,1.f,1.f);
	if(!scenes->volume) return result;

	float step = stepSize;
	float offset = ((float) rand()) / (float) RAND_MAX; //[0-1]
	Vector3f tau = scenes->volume->tau(ray, step, offset);
	result(0) = exp( - tau(0));
	result(1) = exp( - tau(1));
	result(2) = exp( - tau(2));

	return result;
}

Vector3f SingleScatter::Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
						   const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f* T) const
{
	Vector3f result(0.f,0.f,0.f);
	//stuff
	VolumeRegion *vr = scenes->volume;
	float t0=0.f, t1=0.f;
	if (!vr || !vr->IntersectP(ray, &t0, &t1) || (t1-t0) == 0.f) {
		*T = Vector3f(1.f, 1.f,1.f);
		return result;
	}
	int nSamples = (int)ceilf(((t1-t0) / stepSize));
	float step = (t1 - t0) / nSamples;
	Vector3f Tr(1.f,1.f,1.f);
	Vector3f p = ray(t0);
	Vector3f pPrev;
	Vector3f w = -ray.Direction;
	float offset = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
	while(offset == 0.f)
		offset = ((float) rand () ) / (float) RAND_MAX;

	t0 += offset;

	Vector3f Lv(0.f,0.f,0.f);

	for (int i = 0; i < nSamples; ++i, t0 += step) {
		// Advance to sample at _t0_ and update _T_
		pPrev = p;
		p = ray(t0);
		Vector3f direction = p - pPrev;
		//if(direction.norm() == 0.f) std::cout<<"norm zero"<<offset<<" " <<t0<<std::endl;
		direction.normalize();
		Ray tauRay(pPrev, direction, 0.f, 1.f);
		float rndOffset = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
		Vector3f stepTau = vr->tau(tauRay, .5f * stepSize, rndOffset);

		Vector3f exptau = Vec3fExp( - stepTau);
		//Tr *= Vec3fExp(-stepTau);
		Tr = comp_mult(Tr, exptau);

		// Possibly terminate ray marching if transmittance is small
		if (Tr(1) < 1e-3) {
			const float continueProb = .5f;
			if ( ((float) rand()) / (float) RAND_MAX > continueProb) {
				Tr.setZero();
				break;
			}
			Tr /= continueProb;
		}

		// Compute emission term at _p_
		Lv += comp_mult(Tr, vr->Lve(p));
		// add single scattering term at p
		Vector3f ss = vr->sigma_s(p);
		if(ss.norm() != 0.f && _ligs.size() != 0){
			Intersect isec;
			isec.HitPos = p;
			Vector3f Lig(0.f,0.f,0.f);
			Ray Nray;
			for(int i=0;i<_ligs.size();i++)
			{
				Lig += Renderer::Light_Li_Position(scenes, _ligs[i], p, direction, sample, 0, Nray);
				Lig = comp_mult(Lig, Transmittance(scenes, _ligs, Nray , isec, sample, trans, render));
				Lv += comp_mult(Lig, ss);
			}
		}
	}

	*T = Tr;
	return Lv * step;
}