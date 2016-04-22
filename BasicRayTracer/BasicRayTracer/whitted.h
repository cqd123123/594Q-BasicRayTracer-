#ifndef WHITTED_H
#define WHITTED_H

#include "render.h"
#include "scene_io.h"
#include "Gemetry.h"

class WhittedRender:public Renderer{

public:

	Vector3f Li(const Scenes* scenes, const std::vector<Light*>& _lings, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const;

	Vector3f AreaLi(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const
	{return Vector3f(0.f,0.f,0.f);}

	WhittedRender(int md):MaxDepth(md)
	{
		;
	}
	~WhittedRender() {;}
private:
	int MaxDepth;
};

#endif