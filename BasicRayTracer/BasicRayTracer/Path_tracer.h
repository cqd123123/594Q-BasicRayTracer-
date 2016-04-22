#ifndef PATH_TRACER
#define PATH_TRACER

#include "render.h"

class PathTracer : public Renderer
{
public:
	PathTracer(int md):MaxDepth(md){;}
	~PathTracer(){;}
	Vector3f Li(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray,
		Intersect& isec, const sampler2D *sample, const TransformV* trans, const Renderer* render) const;

	Vector3f AreaLi(const Scenes* scenes,const std::vector<Light*>& _ligs, Ray &ray, Intersect& isec,
		const sampler2D *sample, const TransformV* trans, const Renderer* render) const;

private:
	int MaxDepth;

};


#endif