#ifndef BVH_H
#define BVH_H

#include "Gemetry.h"
#include "BoundingBox.h"
 //#include "Ray.h"
// #include <vector>
#include "volume.h"

struct IngInters{
	MaterialIO material;
	float      dist;
};

class BVH_Node{
public:
	BVH_Node(){
		box = BBox();
		left = NULL;
		right = NULL;
		bailed = false;	//debug purpose
	}
	BVH_Node(BVH_Node* _l, BVH_Node* _r):left(_l), right(_r)
	{
		has_child = true;
	}
	bool Intersect1(Ray& ray, Intersect& isec) const;
	void PushObjects(Gemometry* geo)
	{
		geoms.push_back(geo);
		box = Union(box, geo->box);
	}
	void clear(){geoms.clear();}
	void ComputeShadowRayIntersection(Ray& ray, std::vector<IngInters>& integes, bool& is_zero) const;
	void BVH_Normal_Check() const;
	std::vector<Gemometry*> geoms;
	bool has_child;
	bool bailed;
//protected:
	BBox box;
	BVH_Node* left;
	BVH_Node* right;
};

struct BVH_Prim{
	BVH_Prim(Gemometry* _g)
	{
		geom = _g;
		centro = _g->box.getCentroid();
	}
	Gemometry* geom;
	Vector3f centro;
};

struct BVH_bucket{
	BVH_bucket(){count =0 ;}
	int count;
	BBox box;
};
class Scenes{
public:
	Scenes(){
		bvh = new BVH_Node();
		bvh->has_child = false;
		bvh->box = BBox();
		volume = NULL;
	}
	~Scenes(){
		bvh->clear();
	}
	void push_geometry(Gemometry* G_in)
	{
		//	geoms.push_back(G_in);
		bvh->PushObjects(G_in);
	}
	bool Intersect1(Ray& ray, Intersect& iset) const;
	void ComputeShadowRay(Ray& ray, Vector3f& shadowFactor, float& thit, Vector3f& hitnorm) const;
	BVH_Node* BuildBVH();

	void get_all_geoms(std::vector<Gemometry*> & geom){
		int totalSize = bvh->geoms.size();
		for(int i=0;i<totalSize;i++)
			geom.push_back(bvh->geoms[i]);
	}
	///std::vector<Gemometry*> geoms;
	BVH_Node* bvh;
	VolumeRegion * volume;
};

BVH_Node* RecusiveBuild(const std::vector<BVH_Prim*>& buildarr, const int start, const int end, int* totalNodes);

#endif