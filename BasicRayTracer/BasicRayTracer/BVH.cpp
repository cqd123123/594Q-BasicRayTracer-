#include "BVH.h"
#include <cassert>
#include <cstdlib>

bool Scenes::Intersect1(Ray& ray, Intersect& iset) const
{

	bool res = bvh->Intersect1(ray, iset);
	return res;
}


struct {
	bool operator()(IngInters a, IngInters b)
	{   
		return a.dist < b.dist;
	}   
} cmpIng;
//std::sort(s.begin(), s.end(), customLess);

void Scenes::ComputeShadowRay(Ray& ShadowRay, Vector3f& ShadowFactor, float& thit, Vector3f& hitnorm) const
{
	MaterialIO material;
	std::vector<Gemometry*>& geoms = bvh->geoms;
	
	std::vector<IngInters> intersectGeoms;
	
	float hhit = 0.f;
	bool is_zero = false;
	bvh->ComputeShadowRayIntersection(ShadowRay, intersectGeoms, is_zero);
	if(is_zero){ ShadowFactor.setZero(); return;}
	thit = hhit;
	//sort the intersected geom
	int intGoms = intersectGeoms.size();
	std::sort(intersectGeoms.begin(),intersectGeoms.end(), cmpIng);

	//composite the color
	for(int i=0;i<intGoms;i++)
	{
		material =intersectGeoms[i].material;
		float tpancy = material.ktran;
		assert(tpancy <1.0f);
		Vector3f Ndifcol;
		PointToVector3f(material.diffColor, Ndifcol);
		if(Ndifcol.norm() != 0.f)
		{
			Ndifcol = Ndifcol / max(max(Ndifcol(0), Ndifcol(1)), Ndifcol(2));
		}
		ShadowFactor = comp_mult(ShadowFactor, tpancy * Ndifcol);
	}

	intersectGeoms.clear();
}
void BVH_Node::ComputeShadowRayIntersection(Ray& ShadowRay, std::vector<IngInters>& integes, bool& is_zero) const
{
	if(is_zero) return;
	if(!has_child)	//leaf node
	{
		float thit;
		Vector3f hitNormal;
		bool Intersect = false;
		float bthit0, bthit1;
		MaterialIO material;
		Vector3f hitnorm;
		//first test against bb
		if(!box.Intersect(ShadowRay,&bthit0, &bthit1)) return;

		float hhit = 0.f;
		for(int i=0;i<geoms.size();i++)
		{
			if(geoms[i]->Intersect(ShadowRay,&hhit,hitnorm, material))
			{
			//	material = geoms[i]->GetMaterial();
				float tpancy = material.ktran;
				if(tpancy == 0.f ) {integes.clear();is_zero = true ; return;}	//shadow ray is blocked
				IngInters ingI;
				//ingI.geom = geoms[i];
				ingI.material = material;
				ingI.dist = hhit;
				integes.push_back(ingI);
			}
		}

	}
	else
	{
		float hit0,hit1;
		if(!box.Intersect(ShadowRay,&hit0,&hit1))		//intersect bounding box
			return ;
		left->ComputeShadowRayIntersection(ShadowRay,integes, is_zero);
		right->ComputeShadowRayIntersection(ShadowRay, integes, is_zero);
	}
}
bool BVH_Node::Intersect1(Ray& ray, Intersect& iset ) const
{
	if(!has_child)	//leaf node
	{
//		if(!bailed ) return false;   //only render bailed node for now
		float thit;
		Vector3f hitNormal;
		bool Intersect = false;
		float bthit0, bthit1;
		MaterialIO material;
		//first test against bb

		if(!box.Intersect(ray,&bthit0, &bthit1)) return false;

		for(int k=0;k<geoms.size();k++)
		{
			Gemometry* _geom = geoms[k];
			if(_geom->Intersect(ray,&thit, hitNormal, material))
			{
				Intersect = true; 
				iset.geoms = _geom;
				iset.materl = material;
				iset.HitPos = ray.position(thit);
				iset.SufNormal = hitNormal;
				iset.Incident_Dir = ray.Direction;
				ray.tmax = thit;
			}
		}

		return Intersect;
	}
	else 
	{
		//intersect with bounding box
		float hit0,hit1;
		if(!box.Intersect(ray,&hit0,&hit1))
			return false;
		else
		{
			bool hitleft = left->Intersect1(ray, iset);
			bool hitrigh = right->Intersect1(ray, iset);
//  			if(hitleft && hitrigh)
//  				std::cout<<"ccccrt"<<std::endl;
			return hitleft || hitrigh;
		}
	}
}

 BVH_Node* RecusiveBuild( std::vector<BVH_Prim*>& buildarr,const int start, const int end, int& totalNodes)
 {
	 int mid,dim;
	 int TotalElm = end - start;
//	 if(TotalElm == 0) return NULL;
	 BVH_Node* newNode = NULL;
	 BVH_Node* lNode   = NULL;
	 BVH_Node* rNode   = NULL;
//	 std::cout<<"start "<<start<<" end  "<<end<<std::endl;
 	 if(start == 2541 && end == 2569)
 		 std::cout<<"hshhs"<<std::endl;
	 //spit the build arr
	 //compute the bounding box for all primitives
	 BBox totoalBox;
	 for(int i=start; i<end; i++)
	 {
		BVH_Prim* prim = buildarr[i];
		totoalBox = Union(totoalBox, prim->centro);
	 }
	 //get split axis
	 dim = totoalBox.MaxExtent();
	 if(totoalBox.SurfaceArea() > 10000000.f)
	 {
		 std::cout<<start<<" --------- "<<end<<std::endl; 
		 std::cout<<TotalElm<<std::endl;
	 }
	 //put the primitives into bucket
	 float dimleng = (totoalBox.Boxmax - totoalBox.Boxmin)(dim);
	 float dimS    = (totoalBox.Boxmin)(dim);
	 float dimE    = (totoalBox.Boxmax)(dim);
		 
	 const int numBucket = 12;
	 BVH_bucket bucket[numBucket];
	 //init bucket
	 //int enndl = end;
	 for(int i0=start;i0<end; i0++)
	 {
		 BVH_Prim* prim = buildarr[i0];
		 float centdim  = prim->centro(dim);
		 int b = (centdim - dimS) / dimleng * numBucket;
		 if(b >= numBucket) b = numBucket -1;
		// if(b < 0 || b > numBucket) std::cout<<"bb"<<std::endl;
		 if(b < 0) b = 0;
		/* assert(b>=0 && b<numBucket);*/
		 //put stuff inside bucket
		 bucket[b].count ++;
		 bucket[b].box = Union(bucket[b].box, prim->geom->box);
	 }

	 float cosbuck [numBucket-1];
	 //calculate along bucket bvd
	 bool AllOne_buck = false;
	 for(int i=0;i<numBucket -1;i++)
	 {
			 BBox b0, b1;
			 int count0 = 0, count1 = 0;
			 for(int j=0;j<=i;j++)
			 {
				 b0 = Union(b0, bucket[j].box);
				 count0 += bucket[j].count;
			 }
			 for(int j = i+1;j<=numBucket -1;j++)
			 {
				 b1 = Union(b1, bucket[j].box);
				 count1 += bucket[j].count;
			 }
			 //over flow float number multiply by zero is still over flow float number
			 //calculate const
			 float Area0, Area1;
			 if(count0 == 0) Area0 = 0.f;
			 else Area0 = count0 * b0.SurfaceArea();
			 if(count1 == 0) Area1 = 0.f;
			 else Area1 = count1 * b1.SurfaceArea();
			 
			 cosbuck[i] = .125f + (Area0 + Area1) /
				 totoalBox.SurfaceArea();
	 }
	 for(int i=0;i<numBucket-1;i++)
	 {
		 if(bucket[i].count == TotalElm) { AllOne_buck = true; break;}
	 }
	
	 //find minimal cut axis
	 int MinSplitBox = 0;
	 float CosMin = cosbuck[0];
	 for(int i=0;i<numBucket - 1;i++)
	 {
		 if(cosbuck[i] < CosMin)
		 {
			 CosMin = cosbuck[i];
			 MinSplitBox = i;
		 }
	 }
	// std::cout<<"split min box "<<MinSplitBox<<std::endl;
	// std::cout<<"split cos min "<<CosMin<<std::endl;
	 bool rbll = false;
	  if(AllOne_buck && (CosMin < TotalElm || TotalElm > 10)) { rbll = true; std::cout<<"All in one bucket,BVH bail, assign "<<TotalElm<<" geoms in one leaf node"<<std::endl;
	  std::cout<<"start  "<<start<<"  end" <<end<<std::endl;}
	 if((CosMin < TotalElm || TotalElm > 10 )&&!AllOne_buck)
	 {
		 //split and recursive build
		 std::vector<BVH_Prim*> FArr;
		 std::vector<BVH_Prim*> BArr;
		// newArr.resize(end - start);
		
		 for(int i=start;i<end;i++)
		 {
			 BVH_Prim* prim = buildarr[i];
			 float centdim  = prim->centro(dim);
			 int b = (centdim - dimS) / dimleng * numBucket;
			 if(b == numBucket) b = numBucket -1;
			 assert(b>=0 && b<numBucket);
			 if(b <= MinSplitBox)
				 FArr.push_back(prim);
			 else BArr.push_back(prim);
		 }
		 int Fsize = FArr.size();
		 int Bsize = BArr.size();
		 
		 assert((Fsize + Bsize) == (end - start));
		 //reorder
		 for(int i = start; i< start + Fsize ; i++)
			 {
				 buildarr[i] = FArr[i - start];
			 }
			 for(int i = start + Fsize; i<end;i++)
			 {
				 buildarr[i] = BArr[i - start - Fsize];
			 }

			 int midSpI = start + Fsize;
		//	  std::cout<<" split "<<Fsize<<" as "<<Bsize<<" k " <<" start " <<start <<" end "<<end<<" mid spi "<<midSpI<<std::endl;
			 //build Left
			 totalNodes++;
			 lNode = RecusiveBuild(buildarr, start, midSpI, totalNodes);
			 rNode = RecusiveBuild(buildarr, midSpI, end, totalNodes);
			 newNode = new BVH_Node();
			 newNode->left = lNode;
			 newNode->right = rNode;
			 newNode->has_child = true;
			 newNode->box = Union(lNode->box, rNode->box); ////////////////////////
			
			 return newNode;
		 }
		 else		//build leaf node
		 {
			 newNode = new BVH_Node();
			 newNode->left = NULL;
			 newNode->right = NULL;
			 newNode->has_child = false;
			 newNode->bailed = rbll;
			 for(int i=start;i<end;i++)
			 {
				  BVH_Prim* prim = buildarr[i];
				  newNode->PushObjects(prim->geom);
			 }
			 totalNodes++;
			 return newNode;
		 }
 }
 void BVH_Node::BVH_Normal_Check() const
 {
	 if(!has_child)
	 {
		 for(int i=0;i<geoms.size();i++)
		 {
			 geoms[i]->CheckNormal();
		 }
	 }
	 else
	 {
		 left->BVH_Normal_Check();
		 right->BVH_Normal_Check();
	 }
 }

 BVH_Node* Scenes::BuildBVH()
 {
	 BVH_Node* node = NULL;
	 std::vector<BVH_Prim*> buildArr;
	 std::vector<Gemometry*>& geoms = bvh->geoms;
	 for(int i=0;i<geoms.size();i++)
	 {
		 BVH_Prim* prim = new BVH_Prim(geoms[i]);
		 buildArr.push_back(prim);
	 }
	 int TotalNodes=0;
	
	 node = RecusiveBuild(buildArr, 0, buildArr.size(),TotalNodes);
     bvh = node;

	 std::cout<<"Total BVH nodes "<<TotalNodes<<std::endl;
//	 bvh->BVH_Normal_Check();
	 return node;
 }