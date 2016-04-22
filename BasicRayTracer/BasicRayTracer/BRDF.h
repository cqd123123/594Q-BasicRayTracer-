#ifndef BRDF_H
#define BRDF_H


#include <Eigen/Eigen>
#include "util.h"

using namespace Eigen;


class BRDF {
public:
	// BxDF Interface
	virtual ~BRDF() { }
	BRDF() { }
	virtual Vector3f f(const Vector3f &wo, const Vector3f &wi) const = 0;
	virtual Vector3f Sample_f(const Vector3f &wo, Vector3f *wi,
		float u1, float u2, float *pdf) const;
	virtual Vector3f rho(const Vector3f &wo, int nSamples,
		const float *samples) const;
	virtual Vector3f rho(int nSamples, const float *samples1,
		const float *samples2) const;
	virtual float Pdf(const Vector3f &wi, const Vector3f &wo) const;

	// BxDF Public Data
	//const BxDFType type;
};

class Fresnel {
public:
	// Fresnel Interface
	virtual ~Fresnel();
	virtual Vector3f Evaluate(float cosi) const = 0;
};

class FresnelConductor : public Fresnel {
public:
	// FresnelConductor Public Methods
	Vector3f Evaluate(float cosi) const;
	FresnelConductor(const Vector3f &e, const Vector3f &kk)
		: eta(e), k(kk) {
	}

private:
	Vector3f eta, k;
};

class Blinn {
public:
	Blinn(float e) { if (e > 10000.f || e!= e) e = 10000.f;
	exponent = e; }
	// Blinn Public Methods
	float D(const Vector3f &wh) const {
		float costhetah = fabsf(wh(2));
		return (exponent+2) * INV_TWOPI * powf(costhetah, exponent);
	}
	virtual void Sample_f(const Vector3f &wi, Vector3f *sampled_f, float u1, float u2, float *pdf) const;
	virtual float Pdf(const Vector3f &wi, const Vector3f &wo) const;
private:
	float exponent;
};

class Microfacet : public BRDF{
public:
	// Microfacet Public Methods
	Microfacet(const Vector3f &reflectance, Fresnel *f,
		Blinn *d);
	Vector3f f(const Vector3f &wo, const Vector3f &wi) const;
	float G(const Vector3f &wo, const Vector3f &wi, const Vector3f &wh) const {
		float NdotWh = fabsf(wh(2));
		float NdotWo = fabsf(wo(2));
		float NdotWi = fabsf(wi(2));
		float WOdotWh = fabs(wo.dot(wh));
		return min(1.f, min((2.f * NdotWh * NdotWo / WOdotWh),
			(2.f * NdotWh * NdotWi / WOdotWh)));
	}
	Vector3f Sample_f(const Vector3f &wo, Vector3f *wi,
		float u1, float u2, float *pdf) const;
	float Pdf(const Vector3f &wo, const Vector3f &wi) const;
private:
	// Microfacet Private Data
	Vector3f R;
	Blinn *distribution;
	Fresnel *fresnel;
};

#endif