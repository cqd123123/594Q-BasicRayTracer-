#include "Gemetry.h"
#include "util.h"


//ray trangle intersect method, ray and trangle cords MUST in WORLDCORD

bool IntersectTri(const Ray &ray, float* tHit,float* w1, float* w2, const Vector3f& P1,const Vector3f& P2,const Vector3f& P3)
{
	//transform into local coordinates
	//Ray objRay;
	//WorldToObject(ray, objRay);
	Vector3f _Dir = ray.Direction;
	Vector3f _Ori = ray.Origin;

	//compute denominator(s1)

	//
	Vector3f e1 = P2 - P1;
	Vector3f e2 = P3 - P1;
	Vector3f s1 =  _Dir.cross(e2);
	float denominator = s1.dot(e1);
	if(denominator == 0.f)
		return false;
	float invDeno = 1.f/denominator;
	//first barycentric coord
	Vector3f d = _Ori - P1;
	float b1 = d.dot(s1)*invDeno;
	if(b1 < 0.f || b1 > 1.f)
		return false;
	//second barycentric coord
	Vector3f s2 =  d.cross(e1);
	float b2 = _Dir.dot(s2)*invDeno;
	if(b2 < 0.f || (b2 + b1) > 1.f)
		return false;
	//get intersection point
	float t = e2.dot(s2)*invDeno;
	if(t < ray.tmin || t > ray.tmax)
		return false;
	//fill in differencial geometry
	//.....
	//
	*tHit = t;
	*w1   = b1;
	*w2   = b2;

	return true;
}
inline void Sphere::WorldToObject(const Ray &Rin, Ray &Rout) const
{
	Vector3f origin = Rin.Origin;
	Vector3f _D = Rin.Direction;
	Rout.Origin = origin - Model_trans;
	Rout.Direction = Model_Rotate_inv*_D;
	Rout.tmin = Rin.tmin;
	Rout.tmax = Rin.tmax;
}

Sphere::Sphere(Matrix3f _rot, Vector3f _trans, float _r, float _zmin, float _zmax, float _pm, ObjIO* _obj):Gemometry( _obj), Radius(_r),
						Model_Rotate(_rot), Model_trans(_trans), Zmin(_zmin), Zmax(_zmax), PhiMax(_pm)
{
	Vector3f Boxmin = Model_Rotate*Vector3f(-_r, -_r,_zmin);
	Vector3f BoxMax = Model_Rotate*Vector3f(_r,_r,_zmax);
	BoxMax += Model_trans;
	Boxmin += Model_trans;
	box = BBox(Boxmin, BoxMax);
	Zmin = clamp(min(_zmin,_zmax), -Radius, Radius);
	Zmax = clamp(max(_zmin,_zmax), -Radius, Radius);
	thetaMin = acosf(Zmin/Radius);
	thetaMax = acosf(Zmax/Radius);

	Model_Rotate_invT = (Model_Rotate.transpose()).inverse();
	Model_Rotate_inv  = (Model_Rotate.transpose());
}

bool Sphere::Intersect(const Ray& ray, float* Hit,Vector3f& HitNormal, MaterialIO& material) const
{
	//float Phi;
	//Vector3f PhiHit;
	Ray objRay;
	WorldToObject(ray, objRay);
	Vector3f _Dir = objRay.Direction;
	Vector3f _Ori = objRay.Origin;
	//std::cout<<_Dir<<std::endl;
	//std::cout<<" "<<std::endl;
	//calculate quadric param
	float A = _Dir(0)*_Dir(0) + _Dir(1)*_Dir(1) + _Dir(2)*_Dir(2);
	float B = 2.f*(_Dir(0)*_Ori(0) + _Dir(1)*_Ori(1) + _Dir(2)*_Ori(2));
	float C = _Ori(0)*_Ori(0) + _Ori(1)*_Ori(1) + _Ori(2)*_Ori(2) - Radius*Radius;
	float t0,t1;
	if(!SovleQuadric(A, B, C, &t0,&t1))
		return false;
	//whether intersect param in range
	if(t0 > objRay.tmax || t1 < objRay.tmin)
		return false;
	float thit = t0;
	if(t0 < objRay.tmin)
	{
		thit = t1;
		if(thit > objRay.tmax) return false;
	}
	//compute sphere hit position
	Vector3f phit = objRay(thit);
	float phi = atan2f(phit(1), phit(0));
	if(phi < 0.f) phi += 2.f*M_PI;
	//test the intersection with clipping param
	if(phit(2) < Zmin || phit(2) > Zmax || phi > PhiMax)
	{
		if(thit == t1) return false;
		if(t1 > objRay.tmax) return false;
		thit = t1;	//if first out of range, test with the second one
		phit = objRay(thit);
		phi = atan2f(phit(1), phit(0));
		if(phit(2) < Zmin || phit(2) > Zmax || phi > PhiMax)
			return false;
	}
	//find param representation of hit position
	float u = phi / PhiMax;
	float theta = acosf(phit(2)/Radius);
	float v = (theta - thetaMin) / (thetaMax - thetaMin);

	*Hit = thit;
	//get normal
	HitNormal = phit;
	HitNormal.normalize();
	//translate to world cord
	HitNormal = Model_Rotate_invT * HitNormal;
	MaterialAssign(GObject->material, material);
#ifdef CHECKBORD_SHADER
	int ipa1 = u*20;
	int ipa2 = v*20;
	MaterialAssign(GObject->material, material);
	if(ipa1%2 == ipa2%2)
	{
		ColorAssign(material.ambColor, Vector3f(1.f,1.f,1.f));
		ColorAssign(material.diffColor, Vector3f(1.f,1.f,1.f));
		ColorAssign(material.emissColor, Vector3f(1.f,1.f,1.f));
		ColorAssign(material.specColor, Vector3f(1.f,1.f,1.f));

	}
	else
	{	ColorAssign(material.ambColor, Vector3f(0.f,0.f,0.f));
	ColorAssign(material.diffColor, Vector3f(0.f,0.f,0.f));
	ColorAssign(material.emissColor, Vector3f(0.f,0.f,0.f));
	ColorAssign(material.specColor, Vector3f(0.f,0.f,0.f));
	}
#endif

#ifdef INTERSECTION_SHADER

	int ipa1 = u*20;
	int ipa2 = v*20;
	if(ipa1%2 == ipa2%2)
		return true;
	else return false;

#endif

	return true;
}


TrangleMeshNew::TrangleMeshNew(PolySetIO* plset, ObjIO* _obj) : Gemometry(_obj)
{	
	float xmax, xmin, ymax, ymin, zmax, zmin;
	xmax = - FLT_MAX; xmin = FLT_MAX; ymin = FLT_MAX; ymax = - FLT_MAX;
	zmin = FLT_MAX; zmax = - FLT_MAX;

 	Vector3f BoxMax = Vector3f(xmax, ymax, zmax);
 	Vector3f BoxMin = Vector3f(xmin, ymin, zmin);
 	box = BBox(BoxMin, BoxMax);

}
void TrangleMeshNew::ConvetplsetToTrangleSoup(PolySetIO* pplset, ObjIO* _obj, std::vector<Gemometry*>& geoms)
{
	bool perVertexN = false;
	bool perVertexM = false;
	if(pplset->normType == PER_VERTEX_NORMAL ) perVertexN = true;
	if(pplset->materialBinding == PER_VERTEX_MATERIAL) perVertexM = true;
	for(int i=0;i<pplset->numPolys;i++)
	{
		PolygonIO ply =  pplset->poly[i];
		Triangle *tri = new Triangle;
		TransPoly2Tri(&ply, tri);
		TriangleNew* newtri = new TriangleNew(tri, _obj, perVertexN, perVertexM);
		geoms.push_back(newtri);
	}
}
bool TrangleMeshNew::Intersect(const Ray& _ray, float* tHit, Vector3f& HitNormal, MaterialIO& material) const
{
	Ray ray;
	ray = _ray;
	bool intersect = false;
	Triangle* ftri = NULL;
	for(int i=0;i<TotalTri;i++)
	{
		Triangle* tri = tris[i];
		float w2;
		float w3;
		bool result = IntersectTri(ray,tHit,&w2,&w3, tri->vert[0].pos,tri->vert[1].pos,tri->vert[2].pos);
		if(result)
		{
			intersect = true;
			ftri = tri;
			ray.tmax = (*tHit);
		}
	}

	if(intersect)
	{
//		float w2,w3;
		Vector3f pos1,pos2,pos3;
		//interpolate normal
		//getTriVetNormal(ftri->vert[0], n1);
		//getTriVetNormal(ftri->vert[1], n2);
		//getTriVetNormal(ftri->vert[2], n3);

		//calculate perface normal
		pos1 = ftri->vert[0].pos;
		pos2 = ftri->vert[1].pos;
		pos3 = ftri->vert[2].pos;

		HitNormal = GetTrangleNormal(pos1,pos2,pos3);
		MaterialAssign(GObject->material, material);
		//	HitNormal = (1.f - w2 - w3)*n1 + w2*n2 + w3*n3;
	}
	//none intersect
	return intersect;
}
 inline void TrangleMeshNew::TransPoly2Tri(const PolygonIO* poly, Triangle* triout)
{
	for(int i=0;i<3;i++)
	{
		getTriVetPos(poly->vert[i], triout->vert[i].pos);
		getTriVetNormal(poly->vert[i],triout->vert[i].norm);
	//	if(triout->vert[i].norm.norm() == 0) std::cout<<"in zerom nor "<<std::endl;
		
		triout->vert[i].materialIndex = poly->vert[i].materialIndex;
		triout->vert[i].s = poly->vert[i].s;
		triout->vert[i].t = poly->vert[i].t;
	}
}

TriangleNew::TriangleNew(Triangle* t, ObjIO* _obj, bool NType, bool MType):Gemometry(_obj), tri(t), PerVertexNormal(NType), PerVertexMaterial(MType)
{
	float xmax, xmin, ymax, ymin, zmax, zmin;
	xmax = - FLT_MAX; xmin = FLT_MAX; ymin = FLT_MAX; ymax = - FLT_MAX;
	zmin = FLT_MAX; zmax = - FLT_MAX;
	for(int i=0;i<3;i++)
	{
		Vector3f pt = tri->vert[i].pos;
		xmax = max(xmax, pt(0));
		ymax = max(ymax, pt(1));
		zmax = max(zmax, pt(2));
		xmin = min(xmin, pt(0));
		ymin = min(ymin, pt(1));
		zmin = min(zmin, pt(2));
	}

	Vector3f BoxMax = Vector3f(xmax, ymax, zmax);
	Vector3f BoxMin = Vector3f(xmin, ymin, zmin);
	//PerVertexNormal = false;
	box = BBox(BoxMin,BoxMax);
}

bool TriangleNew::Intersect(const Ray& ray, float* tHit, Vector3f& HitNormal, MaterialIO& material) const
{
	float w2,w3;
	
	bool result = IntersectTri(ray,tHit,&w2,&w3, tri->vert[0].pos,tri->vert[1].pos,tri->vert[2].pos);
	if(*tHit < RAY_EPSILON_2) return false;
	//if(result) 
	if(result)
	{
		//float w2,w3;
		Vector3f pos1,pos2,pos3;
		//interpolate normal
		Vector3f n1,n2,n3;
		n1 = tri->vert[0].norm;
		n2 = tri->vert[1].norm;
		n3 = tri->vert[2].norm;
//		std::cout<<tri->vert[0].norm.norm()<<std::endl;
		//calculate perface normal
		pos1 = tri->vert[0].pos;
		pos2 = tri->vert[1].pos;
		pos3 = tri->vert[2].pos;
		
		Vector3f SDir;
	//	SDir = GetTrangleNormal(pos1,pos2,pos3);
	//	if((w2 + w3) < 0.f && (w2 + w3) > 1.f) std::cout<<"sa "<<std::endl;
		
		if(PerVertexNormal)
		{
			HitNormal = (1.f - w2 - w3)*n1 + w2*n2 + w3*n3;
			if(HitNormal.norm()!=0)
 				HitNormal.normalize();
 			if(HitNormal.norm() == 0.)
 				std::cout<<"zero nomr"<<std::endl;
		}
		else
		{
			HitNormal = GetTrangleNormal(pos1,pos2,pos3);	
		}

		if(!PerVertexMaterial)
			MaterialAssign(GObject->material, material);
		else
		{
			int ind1 = tri->vert[0].materialIndex;
			int ind2 = tri->vert[1].materialIndex;
			int ind3 = tri->vert[2].materialIndex;
			MaterialInterpolate(&GObject->material[ind1], &GObject->material[ind2], &GObject->material[ind3],
				(1.f - w2 - w3), w2, w3, material);
		}
	}
	//none intersect
	return result;
}