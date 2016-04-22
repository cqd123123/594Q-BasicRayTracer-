#include "render.h"


Vector3f Renderer::Uniform_Sample_All_light(const Scenes* scenes,const std::vector<Light*>& _ligs, const Ray &ray,
											Intersect& isec, const sampler2D *sample, const TransformV* trans, const Renderer* render)
{
	//direct illumination
	Vector3f Result(0.f,0.f,0.f);
	Ray ShadowRay;
	Vector3f AverageDir(0.f,0.f,0.f);
	Vector3f Shadow_dir;

	if(_ligs.size() == 0) return Result;

	for(int i=0;i<_ligs.size();i++)
	{

		if(_ligs[i]->is_Delta)
			Result += Uniform_Sample_One_light(scenes, _ligs[i], ray, isec, sample, trans, render, Shadow_dir,0);
		else
		{
			Vector3f tempRes(0.f,0.f,0.f);
			int TotalS = sample->get_xspp() * sample->get_yspp();
			
			for(int j=0;j<TotalS;j++)
			{
				tempRes += Uniform_Sample_One_light(scenes, _ligs[i], ray, isec, sample, trans, render, Shadow_dir,j);
				AverageDir += Shadow_dir;
			}
			tempRes = tempRes / TotalS;
			Result += tempRes;
		}

	}
	return Result;

}

Vector3f Renderer::Uniform_Sample_One_light(const Scenes* scenes,const Light* lig,  const Ray &ray, Intersect& isec,
											const sampler2D *sample, const TransformV* trans, const Renderer* render, Vector3f& shaow_dir, const int sampleOffset)
{
	Vector3f Result(0.f,0.f,0.f);
	Ray ShadowRay;
	if(!lig) return Result;
	float pdf=1.f;

	lig->Generate_shadowRay(ShadowRay,isec, &pdf, sample, sampleOffset);
	isec.shadow_Dir = ShadowRay.Direction;
	shaow_dir = ShadowRay.Direction;

	Vector3f ShadowFactor(1.f,1.f,1.f);
	float thit;
	//Intersect shadow_intersect;
	Vector3f hitnorm;
	//MaterialIO material;
	//		Vector3f Lig_LI(0.f,0.f,0.f);
	std::vector<Light*> ll;
	scenes->ComputeShadowRay(ShadowRay,ShadowFactor,thit,hitnorm);
	
	if( scenes->volume!= NULL && render->volumeInt != NULL)
	{
		Vector3f Tr = render->volumeInt->Transmittance(scenes, ll, ShadowRay, isec, sample, trans, render );
		ShadowFactor = comp_mult(ShadowFactor, Tr);
	}

	if(ShadowFactor.norm() != 0.f)	//acc
	{
		MaterialIO material;
		MaterialAssign(&isec.materl, material);//= isec.materl;//isec.geoms->GetMaterial();
		//get incident radiance
		Vector3f RadIn = lig->LI(isec);
		//calculate outgoint radiance
		Vector3f DifCol, SpecCol;
		float kt = material.ktran;
		float ks = material.shininess;
		Vector3f SDir = ShadowRay.Direction;
		Vector3f Normal = isec.SufNormal;
		//if(Normal.dot(SDir) < 0.f ) Normal = -Normal;
		PointToVector3f(material.diffColor, DifCol);
		PointToVector3f(material.specColor, SpecCol);
		float cosFactor = max((Normal.dot(SDir)), 0.f);
		//get dif lighting componnet
		DifCol = (1.f - kt) * DifCol * (cosFactor);

		//get spec component
		Vector3f Sdirref = 2.f * (Normal.dot(SDir)) * Normal - SDir;
		float sCosfator = -(ray.Direction.dot(Sdirref));
		sCosfator = max(sCosfator, 0.f);
		// 			if(ray.Direction .norm() > 1.f + RAY_EPSILON)
		// 			{
		// 				std::cout<<"err" <<ray.Direction.norm()<<std::endl;
		// 			}
		SpecCol = ks * (powf(sCosfator, SPEC_POW_FACTOR)) * SPEC_POW_FACTOR * SpecCol * SPEC_ATTENUATION_FACTOR;
		//accumulate color
		Vector3f Totalmat_col = DifCol + SpecCol;

		Vector3f Totallig_col = comp_mult(RadIn, ShadowFactor);
		Result += comp_mult(Totallig_col, Totalmat_col);

		if(isec.geoms->brdf != NULL)
		{
			//!!TODO Rotate the axis to normal, z (0,0,1) !!//
			float ligpdf;
			Vector3f GLOS;
			Matrix3f Rot;
			Vector3f wi = - ray.Direction;
			Vector3f wo = -SDir;
			//Mat3RotationFromVec(Vector3f(0.f,0.f,1.f), Normal, Rot);
			Mat3RotationFromVec(  Normal, Vector3f(0.f,0.f,1.f), Rot);
			wi = Rot*wi;
			wo = Rot*wo;

			GLOS = isec.geoms->brdf->f( -wo, wi);
			ligpdf = isec.geoms->brdf->Pdf(-wo, wi);
			if(ligpdf > 0.f)
			{
				//std::cout<<"pdf greated than zro"<<std::endl;
				Result += comp_mult( GLOS / ligpdf * cosFactor ,RadIn) ;//Vector3f(1.f,0.f,0.f);
			}
		}

		if(pdf > 0.f)
		{
			return Result /pdf ;
		}
		else
		{
			return Vector3f(0.f,0.f,0.f);
		}
	}
	return Result;
}

//get incident radiance at specific position, no transmissions
Vector3f Renderer::Light_Li_Position(const Scenes* scenes, const Light* lig, const Vector3f& position, const Vector3f& Direction,
									 const sampler2D *sample, const int sample_offset, Ray& shray)
{
	Vector3f Result(0.f,0.f,0.f);
	Ray ShadowRay;
	if(!lig) return Result;
	float pdf=1.f; 
	Intersect isec;
	isec.geoms = NULL;
	isec.HitPos = position;
	//isec.materls;
	isec.Incident_Dir = Direction;
	isec.SufNormal = Vector3f(0.f,0.f,0.f);
	lig->Generate_shadowRay(ShadowRay,isec, &pdf, sample, sample_offset);
	isec.shadow_Dir = ShadowRay.Direction;
//	shaow_dir = ShadowRay.Direction;
	shray = ShadowRay;

	Vector3f ShadowFactor(1.f,1.f,1.f);
	float thit;
	//Intersect shadow_intersect;
	Vector3f hitnorm;
	//MaterialIO material;
	//		Vector3f Lig_LI(0.f,0.f,0.f);

	scenes->ComputeShadowRay(ShadowRay,ShadowFactor,thit,hitnorm);
	Vector3f RadIn = lig->LI(isec);
	return comp_mult(RadIn, ShadowFactor);
}