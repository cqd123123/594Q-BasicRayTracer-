#include <windows.h>
#include <stdio.h>
#include "scene_io.h"
#include "Timer.h"
#include <iostream>
#include "Ray.h"
#include "Gemetry.h"
#include "util.h"
#include <vector>
#include <Eigen/Eigen>
#include "Image_Buffer.h"
#include "Transform.h"
#include "light.h"
#include "whitted.h"
#include "Path_tracer.h"
#include "BVH.h"
#include "BRDF.h"
#include "volume.h"
#include "volumeIntegrator.h"
#include "MeshReader.h"

#include <omp.h>


#define IMAGE_WIDTH		1500
#define IMAGE_HEIGHT	1500
#define MAX_RAY_TRACE_DEPTH 4
const int sample_x  = 4;
const int sample_y  = 4;

//#define RAY_TRACE_DEBUG

typedef unsigned char u08;

SceneIO *scene = NULL;

std::vector<Gemometry* > geoms;
std::vector<Light*> scene_light;

Lights lights;
Scenes* scene_new = NULL;

using namespace Eigen;

static void loadScene(char *name) {


	Blinn* bil = new Blinn(10.f);
	Fresnel* frs = new FresnelConductor(Vector3f(1.3f,1.1f,0.5f),Vector3f(1.f,2.f,4.f));
	Microfacet* MIC = new Microfacet(Vector3f(1.3f,1.1f,0.5f), frs, bil);


	/* load the scene into the SceneIO data structure using given parsing code */
	scene = readScene(name);

	/* hint: use the Visual Studio debugger ("watch" feature) to probe the
	   scene data structure and learn more about it for each of the given scenes */
	CameraIO* camera = scene->camera;
	LightIO* lig = scene->lights;
	ObjIO*  objs = scene->objects;
	std::vector<ObjIO*> objects;

	//get objs
	while(objs != NULL)
	{
		objects.push_back(objs);
		objs = objs->next;
	}
	while(lig != NULL)
	{
		lights._lights.push_back(lig);
		lig = lig->next;
	}
	for(int i=0;i<objects.size();i++)
	{
		ObjType typ = objects[i]->type;
	
		if(typ == SPHERE_OBJ)
		{
			std::cout<<"sphere get"<<std::endl;
			SphereIO* sphe = (SphereIO *) objects[i]->data;
			Point pt;
			PointAssign(pt, sphe->origin);
			Vector3f Trans(pt[0],pt[1],pt[2]);
			Matrix3f Matrot; Matrot.setZero(); Matrot(0,0) = 1.f; Matrot(1,1) = 1.f; Matrot(2,2) = 1.f;
			//std::cout<<Trans<<std::endl;
			//std::cout<<sphe->radius<<std::endl;
			Sphere* newsph = new Sphere(Matrot,Trans,sphe->radius,- sphe->radius,sphe->radius,2*M_PI, objects[i]);
			//newsph->material = objects[i]->material;
			newsph->numMaterials = objects[i]->numMaterials;
			newsph->name = objects[i]->name;
			//newsph->brdf = MIC;
			geoms.push_back(newsph);
		}
		else if(typ == POLYSET_OBJ)
		{
			std::cout<<"poly set get"<<std::endl;
			PolySetIO* set = (PolySetIO* )objects[i]->data;
			
			MaterialIO* mat = objects[i]->material;
			//std::cout<<mat->ambColor[0]<<"  "<<mat->ambColor[1]<<" "<<mat->ambColor[2]<<std::endl;
			
			if(set->type == POLYSET_TRI_MESH)
			{
				TrangleMeshNew* mesh = new TrangleMeshNew(set, objects[i]);
			 
				mesh->ConvetplsetToTrangleSoup(set, objects[i], geoms);
				delete mesh;
				//geoms.push_back(mesh);
			}
			else{
				std::cout<<"not trangle mesh, not supported "<<std::endl;
			}
		}
		else{
			std::cout<<"unknown object"<<std::endl;
		}
	}
	//convert light
	for(int i=0;i<lights._lights.size();i++)
	{
		LightIO* lig = lights._lights[i];
		Vector3f pos, dir, col;
		float cutAngle, dropoff;
		cutAngle = lig->cutOffAngle;
		dropoff  = lig->dropOffRate;
		PointToVector3f(lig->position, pos);
		PointToVector3f(lig->color,col);
		PointToVector3f(lig->direction, dir);

		if(lig->type == SPOT_LIGHT)
		{
			SpotLight* lig = new SpotLight(pos,col,dir,dropoff,cutAngle);
			scene_light.push_back(lig);
		}
		else if(lig->type == DIRECTIONAL_LIGHT)
		{
			DirecionLight* lig = new DirecionLight(dir,col, dropoff);
			scene_light.push_back(lig);
		}
		else if(lig->type == POINT_LIGHT)
		{
			PointLight* lig = new PointLight(pos, col,dropoff);
			scene_light.push_back(lig);
		}
	}
//   	Env_map* envmap = new Env_map("stpeters_probe.pfm");
//   	scene_light.push_back(envmap);
 	QuadLight* Ql = new QuadLight(Vector3f(-0.5f,4.95f, -0.5f),Vector3f(-0.5f,4.95f, 0.5f), Vector3f(0.5f,4.95f,-0.5f), Vector3f(2.7f,2.7f,2.7f));
 	scene_light.push_back(Ql);
//  	QuadLight* Ql = new QuadLight(Vector3f(-0.5f,9.f, -0.5f),Vector3f(-0.5f, 9.f, 0.5f), Vector3f(0.5f,9.f,-0.5f), Vector3f(1.0f,1.0f,1.0f));
// 	QuadLight* Q2 = new QuadLight(Vector3f(-3.5f,-3.5f, 9.f),Vector3f(-3.5f, -2.5f, 9.f), Vector3f(-2.5f,-3.5f,9.f), Vector3f(2.5f,2.5f,2.5f));
// 	QuadLight* Q3 = new QuadLight(Vector3f(-1.5f,-1.5f, 9.f),Vector3f(-1.5f, -0.5f, 9.f), Vector3f(-0.5f,-1.5f,9.f), Vector3f(2.5f,2.5f,2.5f));
// //	Ql->is_spot = true;
// //	Ql->cutAngle = 0.997;
// 	Q2->is_spot = true;
// 	Q2->cutAngle = 0.997;
// 	Q3->is_spot = true;
// 	Q3->cutAngle = 0.997;
//  	scene_light.push_back(Ql);
// 	scene_light.push_back(Q2);
// 	scene_light.push_back(Q3);

	//load models//
	Matrix3f scale, rot;
	SetMatToIdentity(rot);
	rot = Mat3FromQuaterion(Vector3f(0.f,1.f,0.f), M_PI);
	SetMatToIdentity(scale); scale(0,0) = 30.f; scale(1,1) = 30.f; scale(2,2) = 30.f;
	rot *= scale;
	Vector3f trans; trans.setZero();
	trans(0) = -3; trans(1) = -6; trans(2) = 0.f;
	MReader* m_reader = new MReader("../Scenes/dragon.ply",geoms,objects[objects.size() - 1], trans, rot);


	scene_new = new Scenes();

	for(int i=0;i<geoms.size();i++)
	{
		scene_new->push_geometry(geoms[i]);
	}
	
	std::cout<<"num of lights"<<lights._lights.size()<<std::endl;
	std::cout<<"num of geometrys "<<geoms.size()<<std::endl;
	std::cout<<"------- start building BVH ---------"<<std::endl;
	scene_new->BuildBVH();
	
	/* write any code to transfer from the scene data structure to your own here */
	/* */
	return;
}


/* just a place holder, feel free to edit */
void render(void) {

#ifdef RAY_TRACE_DEBUG
	int Image_width = 400;
	int Image_Heigh = 400;
#else
	int Image_width = 600;
	int Image_Heigh = 600;
#endif

	/* cleanup */
	CameraIO* camera = scene->camera;
	Vector3f Campos;
	Vector3f CamDir;
	Vector3f CamUpv;
	float FovDeg;
	PointToVector3f(camera->position , Campos);
	PointToVector3f(camera->viewDirection, CamDir);
	PointToVector3f(camera->orthoUp, CamUpv);
	FovDeg = camera->verticalFOV / M_PI * 180;




	Image_Buffer *buf = new Image_Buffer(Image_width,Image_Heigh);
	TransformV *trans = new TransformV();

	CamDir = Campos + CamDir*10.f;
	//	CamUpv = Vector3f(0.f,0.f,1.f);

	trans->LookAt(Campos,CamDir,CamUpv);
	std::cout<<"beigin rendering"<<std::endl;
	trans->SetViewProjectMatrix(60,0.1,100);
	//trans->EnableDOF(9.0f);
	//trans->RotateCamera(Vector3f(0,0,1), M_PI);
	//pos -6.19481 -2.52771 0.67082
	

	BBox box(Vector3f(-5.f,-5.f,-5.f), Vector3f(5.f,5.f,5.f));
	Matrix3f mat;
	SetMatToIdentity(mat);
	Vector3f tt(0.f,0.f,0.f);

	HomogeneousVolumeDensity *homo = new HomogeneousVolumeDensity(Vector3f(0.1,0.1,0.1), Vector3f(0.1,0.1,0.1), 1.0, Vector3f(0.0f,0.0f,0.0f),
		box,mat, tt);

	VolumeFileReader* vfr = new VolumeFileReader("../Scenes/density.ascii");
	BBox newBox(Vector3f(vfr->minx, vfr->miny, vfr->minz), Vector3f(vfr->maxx, vfr->maxy, vfr->maxz));
	std::cout<<newBox.Boxmax<<std::endl;
	VolumeGridDensity* vgrid = new VolumeGridDensity(Vector3f(1.0,1.0,1.0), Vector3f(0.5,0.5,0.5), 1.0, Vector3f(0.0,0.0,0.0),
		newBox, mat, tt, vfr->nx, vfr->ny, vfr->nz, vfr->d);

	VolumeIntegrator* ems= new Emission(0.5);
	VolumeIntegrator* ss = new SingleScatter(0.5);

	WhittedRender* wrender = new WhittedRender(MAX_RAY_TRACE_DEPTH);
	PathTracer* ptracer    = new PathTracer(3);

	scene_new->volume = vgrid;
	wrender->volumeInt = ss;
	ptracer->volumeInt = ss;


	std::cout<<"beigin rendering"<<std::endl;

	sampler2D* sm2d = new sampler2D(sample_x,sample_y);
	sm2d->GenerateRndStratifiedSmps();
	
	sampler2D* shadows = new sampler2D(2,2);
	shadows->GenerateRndStratifiedSmps();

	Vector3f Hitpos;
	Vector3f hitNormal;
#ifndef RAY_TRACE_DEBUG 
	#pragma omp parallel for ordered schedule(dynamic)
#endif
	for(int j=0;j<Image_Heigh;j++)
	{
		for(int i=0;i<Image_width;i++)
		{
			Vector3f LI_OUT(0.f,0.f,0.f);
			for(int sy = 0;sy < sm2d->get_yspp();sy++)
			{
				for(int sx = 0; sx < sm2d->get_xspp();sx++)
				{
					float imgx = ((float)i + (*sm2d)(sx, sy)(0))/ Image_width;
					float imgy = ((float)j + (*sm2d)(sx, sy)(1))/ Image_Heigh;

					Ray ray;
					Intersect iset;
					float thit;
					trans->GenerateRay(imgx,imgy, &ray, 0.7f, Image_width, Image_Heigh);

					if(scene_new->Intersect1(ray, iset))
					{
						//LI_OUT += wrender->Li(scene_new, scene_light, ray, iset,sm2d ,trans,wrender);
						LI_OUT += ptracer->Li(scene_new, scene_light, ray, iset, shadows, trans, ptracer);
					}
					else
					{
						LI_OUT += ptracer->AreaLi(scene_new, scene_light, ray, iset, shadows, trans, ptracer);
					}
				}
			}

			if(LI_OUT.norm()!= 0.f)
			{
				LI_OUT = LI_OUT/sm2d->totalSamples();
				buf->take_frag_color(Image_width - i - 1,  j, LI_OUT,0.f,1.f);
			}
		}
	}

	buf->save_image_buffer("buf.bmp");

	return;
}



int main(int argc, char *argv[]) {
	Timer total_timer;
	total_timer.startTimer();
	Timer pre_timer;
	pre_timer.startTimer();
	loadScene("../Scenes/boxe.ascii");
	pre_timer.stopTimer();
	std::cout<<"Prepossing time "<<pre_timer.getTime()<<std::endl;
	/* write your ray tracer here */
	render();

	total_timer.stopTimer();
	std::cout<<"Rendering time "<<total_timer.getTime() - pre_timer.getTime()<<std::endl;
	fprintf(stderr, "Total time: %.5lf secs\n\n", total_timer.getTime());
	
	if (scene != NULL) {
		deleteScene(scene);
	}

// 	Blinn* bil = new Blinn(1.f);
// 	Fresnel* frs = new FresnelConductor(Vector3f(1.3f,1.1f,0.5f),Vector3f(1.f,2.f,4.f));
// 	Microfacet* MIC = new Microfacet(Vector3f(1.3f,1.1f,0.5f), frs, bil);
// 
// 
// 	for(int i=0;i<20;i++)
// 	{
// 		float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 -1]
// 		float randy = ((float) rand()) / (float) RAND_MAX;
// 		float rand1 = ((float) rand()) / (float) RAND_MAX;
// 		float rand2 = ((float) rand()) / (float) RAND_MAX;
// 		float pdf;
// 		Vector3f wo, wi;
// 		CosineSampleSphere(wo,randx,randy,pdf);
// 		pdf = MIC->Pdf(wo,wi);
// 		CosineSampleSphere(wi,rand1, rand2, pdf);
// 		std::cout<<MIC->f(wo,wi).norm() / pdf <<std::endl;
// 		//MIC->f()
// 		
// 	}

// 	Vector3f Acc(0.f,0.f,0.f);
// 	float accx = 0.f;
// 	float accy = 0.f;
// 	int numofS = 1000;
// 	float accDx = 0.f;
// 	float accDy = 0.f;
// 	CImg<float> image;
// 	image.resize(500,500,1,1);
// 
// 	for(int i=0;i<numofS;i++)
// 	{
// 		Vector3f sample;
// 		float sxx;
// 		float syy;
// 		float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 - 1]
// 		float randy = ((float) rand()) / (float) RAND_MAX;
// 		CosineSampleSphere(sample, randx, randy);
// 		int xcord = (sample(0)*0.5f + 0.5f)*500;
// 		int ycord = (sample(1)*0.5f + 0.5f)*500;
// 		image(xcord,ycord) = 255.f;
// 		accx += randx;
// 		accy += randy;
// 		Acc += sample;
// 		ConcentricSampleDisk(&sxx, &syy, randx, randy);
// 		accDx += sxx;
// 		accDy += syy;
// 	}
// 	image.save("sample.bmp");
	return 1;
}


/*	Point p1 = {0,0,0};
Point p2 = {1,0,0};
Point p3 = {0,1,0};
Ray ray;
ray.Origin = Vector3f(0,0,-10);
ray.Direction = Vector3f(0,0,1);
ray.tmax = FLT_MAX;
ray.tmin = 0.f;
float b1,b2, thit;
if(IntersectTri(ray,&thit,&b1,&b2,p1,p2,p3))
{
std::cout<<"hit "<<b1<<"  "<<b2<<std::endl;
}
ray.Origin = Vector3f(1,0,-10);
if(IntersectTri(ray,&thit,&b1,&b2,p1,p2,p3))
{
std::cout<<"hit "<<b1<<"  "<<b2<<std::endl;
}
ray.Origin = Vector3f(0,1,-10);
if(IntersectTri(ray,&thit,&b1,&b2,p1,p2,p3))
{
std::cout<<"hit "<<b1<<"  "<<b2<<std::endl;
}
ray.Origin = Vector3f(0.5f,0.5f,-10.f);
if(IntersectTri(ray,&thit,&b1,&b2,p1,p2,p3))
{
std::cout<<"hit "<<b1<<"  "<<b2<<std::endl;
}
ray.Origin = Vector3f(1/3.f,1/3.f,-10.f);
if(IntersectTri(ray,&thit,&b1,&b2,p1,p2,p3))
{
std::cout<<"hit "<<b1<<"  "<<b2<<std::endl;
}
Vector3f boxmin(0.f,0.f,0.f);
Vector3f boxmax(1.f,1.f,1.f);
BBox box(boxmin, boxmax);
Ray testray;
testray.Origin = Vector3f(0.5f,0.5f,0.5f);
testray.Direction = Vector3f(0.f,0.f,1.f);
float thit0,thit1;
if(box.Intersect(testray,&thit0,&thit1))
{
std::cout<<"te "<<thit0<<"  "<<thit1<<std::endl; 
}
*/