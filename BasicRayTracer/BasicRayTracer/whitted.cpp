#include "whitted.h"

Vector3f WhittedRender::Li(const Scenes* scenes, const std::vector<Light*>& _lings, Ray &ray, Intersect& isec,
						   const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
	//if(!scenes->Intersect1(ray, isec))
		//return Vector3f(0.f,0.f,0.f);

	Vector3f result(0,0,0);
	Gemometry* hit_geom = isec.geoms;
	float ras[3];
	//get hit gemoetry material
	MaterialIO material = isec.materl; //hit_geom->GetMaterial();
	//if(material == NULL) return result;
	Ray ShadowRay;

	Vector3f ambcol, difcol, speccol, emiscol;
	float tpancy = material.ktran;
	getMaterialColor(material,ambcol,difcol,speccol,emiscol);
	//get ambient term
	result += (1.f - tpancy)*(AMBIENT_ATTENTUATION_FACTOR)*ambcol;

	//get light term
	result += Renderer::Uniform_Sample_All_light(scenes,_lings,ray, isec, sample, trans, render) * DIFFUSE_ATTEN_FACTOR;

	//spawn new ray
	Vector3f RefRadiance(0.f,0.f,0.f);
	if(ray.Depth < MaxDepth)
	{
		Vector3f _dir = ray.Direction;
		Vector3f _normal = isec.SufNormal;
		float shines = material.shininess;
		float ktran  = material.ktran;
		if(shines != 0.f)	//spawn reflective ray
		{
			Ray RefRay;
			RefRay.Origin = isec.HitPos + RAY_EPSILON * _normal;
			RefRay.Direction = _dir - 2.f*(_dir.dot(_normal))*_normal;
			RefRay.tmin = 0.f;
			RefRay.tmax = FLT_MAX;
			RefRay.Depth = ray.Depth + 1;
			Intersect newIsec;
			if(scenes->Intersect1(RefRay,newIsec))
			{
				Vector3f Refcomp = render->Li(scenes, _lings, RefRay,newIsec, sample, trans, render);
				float RefFactor = REFLECTION_ATTEN_FACTOR;
				if(shines > 0.9f) RefFactor = shines;
				result += RefFactor*shines * Refcomp;
			}
		}
		//spawn transpancy ray
		if(ktran != 0.f)
		{
			Ray RefRay;
			float val = _normal.dot(_dir);
			float sgn = val < 0.f ? -1.f: 1.f;
			float n1  = ray.refindex;
			RefRay.Origin = isec.HitPos + RAY_EPSILON * sgn * _normal;
			RefRay.tmin = 0.f;
			RefRay.tmax = FLT_MAX;
			RefRay.Depth = ray.Depth + 1;
			bool specref = false;
			float cosin = fabs(val);
			float sinin = sqrtf(1.f - cosin * cosin);
			float sinout, cosout;
			if(sgn < 0.f)
			{
				sinout = sinin * n1 * (1.f / REFRACTION_INDEX);
				cosout = sqrtf(1.f - sinout*sinout);
				RefRay.Direction = n1 * (1.f / REFRACTION_INDEX) *( cosin*_normal + _dir) - cosout * _normal;
				//if(RefRay.Direction.norm() > 1.f + RAY_EPSILON) std::cout<<"e1 "<<RefRay.Direction.norm()<<std::endl;
				RefRay.refindex = REFRACTION_INDEX;
			}
			else{
				float sinout = sinin * n1;
				if(sinout < 1.f )
				{
					cosout = sqrtf(1.f - sinout*sinout);
					RefRay.Direction = n1 * ( - cosin*_normal + _dir) + cosout * _normal;
					//if(RefRay.Direction.norm() > 1.f + RAY_EPSILON) std::cout<<"e2 "<<RefRay.Direction.norm()<<std::endl;
					RefRay.refindex = 1.f;
				}
				else{
					RefRay.Origin = isec.HitPos - RAY_EPSILON * _normal;
					RefRay.Direction = _dir - 2.f * (_dir.dot(_normal)) * _normal;
					//if(RefRay.Direction.norm() > 1.f + RAY_EPSILON) std::cout<<"e3 "<<RefRay.Direction.norm()<<std::endl;
					RefRay.refindex = n1;
					specref = true;
				}
			}

			Intersect newIsec;
			if(scenes->Intersect1(RefRay,newIsec))
			{
				Vector3f Refcomp = render->Li(scenes, _lings, RefRay,newIsec, sample, trans, render);
			
				if(!specref)
					result += ktran * Refcomp;
				else result += Refcomp;
			}
		}
	}
	if( scenes->volume!= NULL && volumeInt != NULL)
	{
		Vector3f Tr = volumeInt->Transmittance(scenes, _lings, ray, isec, sample, trans, render );
		Vector3f TTR;
		Vector3f LLi = volumeInt->Li(scenes, _lings, ray, isec, sample, trans, render, &TTR);
		if(is_vec3_nan(LLi)) std::cout<<"sasa"<<std::endl;
		result = comp_mult(result, Tr);
		result += LLi;
	}
	return result;
}