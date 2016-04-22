#ifndef DIRECTLIGHTING_H
#define DIRECTLIGHTING_H

#include <Eigen/Eigen>
#include <vector>
#include "Gemetry.h"
#include "render.h"
#include "sampler.h"
#include "light.h"

using namespace Eigen;

Vector3f Evaluate_Direct_Illumination( Scenes* scenes, Lights* _ligs, const Ray &ray, Intersect& isec,
									  const sampler2D *sample, const TransformV* trans, const Renderer* render) const
{
	Vector3f result(0.f,0.f,0.f);
	for(int i=0;i<_ligs->_lights.size();i++)
	{
		LightIO* _li = _ligs->_lights[i];
		
		//get shadow rays
		Ray shadowRay;
		
	}
}

#endif