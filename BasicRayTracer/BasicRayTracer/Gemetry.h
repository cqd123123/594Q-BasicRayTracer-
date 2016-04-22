#ifndef GEMETRY_H
#define GEMETRY_H

#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include "Ray.h"
#include "util.h"
#include "scene_io.h"
#include <cassert>
#include <vector>
#include "BoundingBox.h"
#include "BRDF.h"

using namespace Eigen;

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif



class Gemometry
{
public:
	Gemometry(ObjIO* _obj):GObject(_obj)
	{
		brdf = NULL;
	}
	virtual bool Intersect(const Ray& ray, float* tHit, Vector3f& HitNormal, MaterialIO& material) const =0;

	//inline MaterialIO* GetMaterial(){return material;}
	virtual void CheckNormal () const = 0;
	//MaterialIO* material;
	char *name;
	long numMaterials;
	BBox box;
	BRDF* brdf;
	protected:
	ObjIO* GObject;

};



class Sphere : public Gemometry{
public:
	Sphere(Matrix3f _rot, Vector3f _trans, float _r, float _zmin, float _zmax, float _pm, ObjIO* _obj);

	bool Intersect(const Ray& ray, float* Hit,Vector3f& HitNormal, MaterialIO& material) const;
    inline void WorldToObject(const Ray &Rin, Ray &Rout) const;
	void CheckNormal() const {;}
private:
	float Radius;
	float Zmin;
	float Zmax;
	float thetaMin;
	float thetaMax;
	float PhiMax;


	Matrix3f Model_Rotate;
	Vector3f Model_trans;
	Matrix3f Model_Rotate_invT;
	Matrix3f Model_Rotate_inv;
};

struct Intersect{
	Intersect(){
		geoms = NULL;
	}
	Vector3f shadow_Dir;
	Vector3f Incident_Dir;
	Vector3f SufNormal;
	Vector3f HitPos;
	Gemometry* geoms;
	MaterialIO materl;
};
struct Vertex{
	Vector3f pos;			/* Vertex position */
	Vector3f norm;			/* Normal to vertex (if PER_VERTEX_NORMAL) */
	long materialIndex;		/* Index into materials list (see ObjIO), */
	/* if material is PER_VERTEX_MATERIAL */
	float s, t;			/* Texture coordinates (if hasTextureCoords) */
};
struct Triangle{
	Vertex vert[3];
};

class TrangleMeshNew :public Gemometry
{
public:
	//the plset vertex must be in world coordinates
	TrangleMeshNew(PolySetIO* plset, ObjIO* _obj);

	inline Triangle* Get_trangle(int i){
		assert(i< tris.size());
		return tris[i];
	}
	bool Intersect(const Ray& ray, float* tHit, Vector3f& HitNormal, MaterialIO& material) const;
	//void ConvertToTrangleSoup(std::vector<Gemometry*>& geoms);
	static void ConvetplsetToTrangleSoup(PolySetIO* plset, ObjIO* _obj,  std::vector<Gemometry*>& geoms);
	static inline void TransPoly2Tri(const PolygonIO* poly, Triangle* triout) ;
	void CheckNormal() const {return;}
private:


	std::vector<Triangle*> tris;
	int TotalTri;
};
class TriangleNew : public Gemometry
{
public:
	//the plset vertex must be in world coordinates
	TriangleNew(Triangle* t, ObjIO* _obj, bool Ntype, bool MType);
	bool Intersect(const Ray& ray, float* tHit, Vector3f& HitNormal, MaterialIO& material) const;
	void CheckNormal() const{
		for(int i=0;i<3;i++)
		{
			Vertex vert = tri->vert[i];
			if(vert.norm.norm() == 0.f)
			{
				std::cout<<"norm is zero"<<std::endl;
			}
		}
	}
private:
	bool PerVertexNormal;
	bool PerVertexMaterial;
	Triangle* tri;
};
#endif