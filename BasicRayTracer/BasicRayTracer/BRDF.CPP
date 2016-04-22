#include "BRDF.h"
#include "util.h"

 Vector3f FrCond(float cosi, const Vector3f &eta, const Vector3f &k) {
	Vector3f tmp = (eta.cwiseProduct(eta) + k.cwiseProduct(k)) * cosi*cosi;
	Vector3f tem1 = (tmp + (2.f * eta * cosi) + Vector3f(1.f,1.f,1.f));
	Vector3f Rparl2 = (tmp - (2.f * eta * cosi) + Vector3f(1.f,1.f,1.f));
	Rparl2 = comp_divide(Rparl2, tem1);
	  
	Vector3f tmp_f = eta.cwiseProduct(eta) + k.cwiseProduct(k);
	float val = cosi*cosi;
	Vector3f tem2 = (tmp_f + (2.f * eta * cosi) + Vector3f(val,val,val));
	Vector3f Rperp2 = (tmp_f - (2.f * eta * cosi) + Vector3f(val,val,val));
	Rperp2 = comp_divide(Rperp2,tem2 );	
		
	return (Rparl2 + Rperp2) / 2.f;
}

 Fresnel::~Fresnel() { }

 Vector3f FresnelConductor::Evaluate(float cosi) const {
	 return FrCond(fabsf(cosi), eta, k);
 }

 Vector3f BRDF::Sample_f(const Vector3f &wo, Vector3f *wi,
	 float u1, float u2, float *pdf) const {
		 // Cosine-sample the hemisphere, flipping the direction if necessary
		 float temp;
		 CosineSampleSphere(*wi,u1, u2,temp);
		 if (wo((2)) < 0.) (*wi)(2) *= -1.f;
		 *pdf = Pdf(wo, *wi);
		 return f(wo, *wi);
 }


 float BRDF::Pdf(const Vector3f &wo, const Vector3f &wi) const {
	 return SameHemisphere(wo, wi) ? fabsf(wi(2)) * INV_PI : 0.f;
 }
 Vector3f BRDF::rho(const Vector3f &w, int nSamples,
	 const float *samples) const {
		return Vector3f(0.f,0.f,0.f);
 }


 Vector3f BRDF::rho(int nSamples, const float *samples1,
	 const float *samples2) const {
		return Vector3f(0.f,0.f,0.f);
 }

 void Blinn::Sample_f(const Vector3f &wo, Vector3f *wi, float u1, float u2,
	 float *pdf) const {
		 // Compute sampled half-angle vector $\wh$ for Blinn distribution
		 float costheta = powf(u1, 1.f / (exponent+1));
		 float sintheta = sqrtf(max(0.f, 1.f - costheta*costheta));
		 float phi = u2 * 2.f * M_PI;
		 Vector3f wh = SphericalDirection(sintheta, costheta, phi);
		 if (!SameHemisphere(wo, wh)) wh = -wh;

		 // Compute incident direction by reflecting about $\wh$
		 *wi = -wo + 2.f * wo.dot(wh) * wh;

		 // Compute PDF for $\wi$ from Blinn distribution
		 float blinn_pdf = ((exponent + 1.f) * powf(costheta, exponent)) /
			 (2.f * M_PI * 4.f * wo.dot(wh));
		 if (wo.dot(wh) <= 0.f) blinn_pdf = 0.f;
		 *pdf = blinn_pdf;
 }


 float Blinn::Pdf(const Vector3f &wo, const Vector3f &wi) const {
	 Vector3f wh = (wo + wi);
	 wh.normalize();
	 float costheta = fabsf(wh(2));
	 // Compute PDF for $\wi$ from Blinn distribution
	 float blinn_pdf = ((exponent + 1.f) * powf(costheta, exponent)) /
		 (2.f * M_PI * 4.f * wo.dot(wh));
	 if (wo.dot(wh) <= 0.f) blinn_pdf = 0.f;
	 return blinn_pdf;
 }
 Vector3f Microfacet::f(const Vector3f &wo, const Vector3f &wi) const {
	 float cosThetaO = fabsf(wo(2));
	 float cosThetaI = fabsf(wi(2));
	 if (cosThetaI == 0.f || cosThetaO == 0.f) return Vector3f(0.f,0.f,0.f);
	 Vector3f wh = wi + wo;
	 if (wh(0) == 0. && wh(1) == 0. && wh(2) == 0.) return Vector3f(0.f,0.f,0.f);
	 wh.normalize();
	 float cosThetaH = wi.dot(wh);
	 Vector3f F = fresnel->Evaluate(cosThetaH);
	 Vector3f tmp1 = R * distribution->D(wh);
	 Vector3f tmp2 = F * G(wo, wi, wh);
	 return tmp1.cwiseProduct(tmp2) / (4.f * cosThetaI * cosThetaO);
 }

 Vector3f Microfacet::Sample_f(const Vector3f &wo, Vector3f *wi,
	 float u1, float u2, float *pdf) const {
		 distribution->Sample_f(wo, wi, u1, u2, pdf);
		 if (!SameHemisphere(wo, *wi)) return Vector3f(0.f,0.f,0.f);
		 return f(wo, *wi);
 }


 float Microfacet::Pdf(const Vector3f &wo, const Vector3f &wi) const {
	 if (!SameHemisphere(wo, wi)) return 0.f;
	 return distribution->Pdf(wo, wi);
 }

 Microfacet::Microfacet(const Vector3f &reflectance, Fresnel *f,
	 Blinn *d):
	 R(reflectance), distribution(d), fresnel(f) {
 }

