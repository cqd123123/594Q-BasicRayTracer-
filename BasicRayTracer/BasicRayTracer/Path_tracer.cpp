#include "Path_tracer.h"





Vector3f PathTracer::Li(const Scenes* scenes,const std::vector<Light*>& _lings,  Ray &ray, 
						Intersect& isec, const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
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
	result += emiscol;
	//get ambient term
	result += (1.f - tpancy)*(AMBIENT_ATTENTUATION_FACTOR)*ambcol;

	result += Renderer::Uniform_Sample_All_light(scenes,_lings,ray, isec, sample, trans, render) * DIFFUSE_ATTEN_FACTOR;


	//spawn new ray
	Vector3f RefRadiance(0.f,0.f,0.f);
	if(ray.Depth < MaxDepth)
	{
		Vector3f _dir = ray.Direction;
		Vector3f _normal = isec.SufNormal;
		float shines = material.shininess;
		float ktran  = material.ktran;
		float pdf;
		Vector3f Direction;
		if(shines != 0.f)	//spawn reflective ray
		{
			Ray RefRay;
			RefRay.Origin = isec.HitPos + RAY_EPSILON * _normal;
			//RefRay.Direction = _dir - 2.f*(_dir.dot(_normal))*_normal;
			if(shines < 1.f)
			{
				float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
				float randy = ((float) rand()) / (float) RAND_MAX;
				Matrix3f Rot;
				Mat3RotationFromVec(Vector3f(0.f,0.f,1.f), _normal, Rot);
				CosineSampleSphere(Direction, randx, randy, pdf);
				Direction = Rot * Direction;
			}
			else
			{
				Direction = _dir - 2.f*(_dir.dot(_normal))*_normal;
				pdf = 1.f;
			}
//			pdf = Direction.dot(_normal);
			if(pdf != pdf || pdf < 0.f)
			{
				std::cout<<"err "<<std::endl;
			}
			RefRay.Direction = Direction;
			RefRay.tmin = 0.f;
			RefRay.tmax = FLT_MAX;
			RefRay.Depth = ray.Depth + 1;
			Intersect newIsec;

			float cosFactor = max((_normal.dot(Direction)), 0.f);
			//get dif lighting componnet
			Vector3f Difcol =  difcol * (cosFactor);
			
			if(scenes->Intersect1(RefRay,newIsec) && pdf!= 0.f )
			{
				Vector3f Refcomp = render->Li(scenes, _lings, RefRay,newIsec, sample, trans, render);
				if( scenes->volume!= NULL && render->volumeInt != NULL)
				{
					Vector3f Tr = render->volumeInt->Transmittance(scenes, _lings, RefRay, newIsec, sample, trans, render );
					Refcomp = comp_mult(Refcomp, Tr);
				}

				if(isec.geoms->brdf != NULL)
				{
					//!!TODO Rotate the axis to normal, z (0,0,1) !!//
					float ligpdf;
					Vector3f GLOS;
					Matrix3f Rot;
					Vector3f wi = - _dir;
					Vector3f wo = RefRay.Direction;
					//Mat3RotationFromVec(Vector3f(0.f,0.f,1.f), Normal, Rot);
					Mat3RotationFromVec(  _normal, Vector3f(0.f,0.f,1.f), Rot);
					wi = Rot*wi;
					wo = Rot*wo;

					GLOS = isec.geoms->brdf->f( wo, wi);
					ligpdf = isec.geoms->brdf->Pdf(wo, wi);
					if(ligpdf > 0.f)
					{
						//std::cout<<"pdf greated than zro"<<std::endl;
						result += comp_mult( GLOS / ligpdf * cosFactor ,Refcomp / pdf) ;//Vector3f(1.f,0.f,0.f);
					}
				}
				else
				{
					if(shines < 1.f)
						result += comp_mult(Refcomp / pdf, Difcol);
					else
						result += Refcomp;
				}
			}
		}
	}
	if( scenes->volume!= NULL && volumeInt != NULL)
	{
		Vector3f Tr = volumeInt->Transmittance(scenes, _lings, ray, isec, sample, trans, render );
		Vector3f TTR;
		Vector3f LLi = volumeInt->Li(scenes, _lings, ray, isec, sample, trans, render, &TTR);
		
		//if(is_vec3_nan(LLi)) std::cout<<"sasa"<<std::endl;
		result = comp_mult(result, Tr);
		result += LLi;
	}
	return result;
}

Vector3f PathTracer::AreaLi(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray,
							Intersect& isec, const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
	Vector3f result(0.f,0.f,0.f);
	for(int i=0;i<_ligs.size();i++)
	{
		result += _ligs[i]->LI(ray.Direction);
	}
	if( scenes->volume!= NULL && volumeInt != NULL)
	{
		Vector3f Tr = volumeInt->Transmittance(scenes, _ligs, ray, isec, sample, trans, render );
		Vector3f TTR;
		Vector3f LLi = volumeInt->Li(scenes, _ligs, ray, isec, sample, trans, render, &TTR);
		result = comp_mult(result, Tr);
		result += LLi;
	}
	return result;
}