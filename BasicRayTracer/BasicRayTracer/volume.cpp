#include "volume.h"
#include <iostream>

float PhaseHG(const Vector3f &w, const Vector3f &wp, float g) {
	float costheta = w.dot(wp);
	return 1.f / (4.f * M_PI) *
		(1.f - g*g) / powf(1.f + g*g - 2.f * g * costheta, 1.5f);
}


void HomogeneousVolumeDensity::WorldToObject(const Ray &Rin, Ray &Rout) const
{
	Vector3f origin = Rin.Origin;
	Vector3f _D = Rin.Direction;
	Rout.Origin = origin - Model_trans;
	Rout.Direction = Model_Rotate_inv*_D;
	Rout.tmin = Rin.tmin;
	Rout.tmax = Rin.tmax;
}
bool HomogeneousVolumeDensity::IntersectP(const Ray &ray, float *t0, float *t1) const
{
	Ray objRay;
	WorldToObject(ray, objRay);
	return bbox.Intersect(objRay, t0, t1);
}

Vector3f HomogeneousVolumeDensity::sigma_a(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return sig_a;
	else
		return Vector3f(0.f,0.f,0.f);
}

Vector3f HomogeneousVolumeDensity::sigma_s(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return sig_s;
	else return Vector3f(0.f,0.f,0.f);
}

Vector3f HomogeneousVolumeDensity::sigma_t(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return sig_s+sig_a;
	else
		return Vector3f(0.f,0.f,0.f);
}

Vector3f HomogeneousVolumeDensity::Lve(const Vector3f& pos) const
{
	if(bbox.Inside(pos))
		return le;
	else return Vector3f(0.f,0.f,0.f);
}

float HomogeneousVolumeDensity::p(const Vector3f & pos, const Vector3f & wi, const Vector3f & wo) const
{
	if(!bbox.Inside(pos))
		return 0.f;
	else
		return PhaseHG(wi, wo, g);
}
//optical distance, must use exponent of exp
Vector3f HomogeneousVolumeDensity::tau(const Ray &ray, float step /* = 1.f */, float offset /* = 0.5 */) const
{
	float t0, t1;
	if (!IntersectP(ray, &t0, &t1)) return Vector3f(0.f,0.f,0.f);
	Vector3f pos0 = ray(t0);
	Vector3f pos1 = ray(t1);
	float dist = (pos0 - pos1).norm();
	return dist * (sig_a + sig_s);
}
void VolumeGridDensity::WorldToObject(const Ray &Rin, Ray &Rout) const
{
	Vector3f origin = Rin.Origin;
	Vector3f _D = Rin.Direction;
	Rout.Origin = origin - Model_trans;
	Rout.Direction = Model_Rotate_inv*_D;
	Rout.tmin = Rin.tmin;
	Rout.tmax = Rin.tmax;
}
bool VolumeGridDensity::IntersectP(const Ray &ray, float *t0, float *t1) const
{
	Ray objRay;
	WorldToObject(ray, objRay);
	return bbox.Intersect(objRay, t0, t1);
}
float VolumeGridDensity::Density(const Vector3f &Pobj) const
{
	if(!bbox.Inside(Pobj)) return 0;
	// Compute voxel coordinates and offsets for _Pobj_
	Vector3f vox = bbox.Offset(Pobj);
	vox(0) = vox(0) * nx - .5f;
	vox(1) = vox(1) * ny - .5f;
	vox(2) = vox(2) * nz - .5f;
	int vx = (int)(vox(0)), vy = int(vox(1)), vz = int(vox(2));
	float dx = vox(0) - vx, dy = vox(1) - vy, dz = vox(2) - vz;

	// Trilinearly interpolate density values to compute local density
	float d00 = Lerp(dx, D(vx, vy, vz),     D(vx+1, vy, vz));
	float d10 = Lerp(dx, D(vx, vy+1, vz),   D(vx+1, vy+1, vz));
	float d01 = Lerp(dx, D(vx, vy, vz+1),   D(vx+1, vy, vz+1));
	float d11 = Lerp(dx, D(vx, vy+1, vz+1), D(vx+1, vy+1, vz+1));
	float d0 = Lerp(dy, d00, d10);
	float d1 = Lerp(dy, d01, d11);
	return Lerp(dz, d0, d1);
}

Vector3f VolumeGridDensity::sigma_a(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return Density(pos) * sig_a;
	else
		return Vector3f(0.f,0.f,0.f);
}

Vector3f VolumeGridDensity::sigma_s(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return Density(pos) * sig_s;
	else return Vector3f(0.f,0.f,0.f);
}

Vector3f VolumeGridDensity::sigma_t(const Vector3f & pos) const
{
	if(bbox.Inside(pos))
		return Density(pos) * (sig_s+sig_a);
	else return Vector3f(0.f,0.f,0.f);
}

Vector3f VolumeGridDensity::Lve(const Vector3f& pos) const
{
	if(bbox.Inside(pos))
	{return Density(pos) * le;}
	else return Vector3f(0.f,0.f,0.f);
}

float VolumeGridDensity::p(const Vector3f & pos, const Vector3f & wi, const Vector3f & wo) const
{
	if(!bbox.Inside(pos))
		return 0.f;
	else
		return PhaseHG(wi, wo, g);
}

Vector3f VolumeGridDensity::tau(const Ray &ray, float step /* = 1.f */, float u /* = 0.5 */) const
{
	float t0, t1;
	
	if (!IntersectP(ray, &t0, &t1)) return Vector3f(0.f,0.f,0.f);
	Vector3f tau(0.f,0.f,0.f);
	t0 += u * step;
	while (t0 < t1) {
		tau += sigma_t(ray(t0));
		t0 += step;
	}
	return tau * step;
}

VolumeFileReader::VolumeFileReader(std::string name)
{
	FILE* in = fopen(name.c_str(), "r");
	char buf[100];

	fscanf(in, "%s", buf);
	if(strcmp(buf, "Volume") != 0)
		printf("not volume");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "volumegrid") != 0)
		printf("volumegrid missing");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "nx") != 0)
		printf("nx missing");
	fscanf(in, "%s", buf);
	nx = atoi(buf);
	fscanf(in, "%s", buf);
	if(strcmp(buf, "ny") != 0)
		printf("ny missing");
	fscanf(in, "%s", buf);
	ny = atoi(buf);
	fscanf(in, "%s", buf);
	if(strcmp(buf, "nz") != 0)
		printf("nz missing");
	fscanf(in, "%s", buf);
	nz = atoi(buf);

	fscanf(in, "%s", buf);
	if(strcmp(buf, "p0") != 0)
		printf("p0 missing");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "[") != 0)
		printf("[ missing");
	fscanf(in, "%s", buf);
	minx = atof(buf);
	fscanf(in, "%s", buf);
	miny = atof(buf);
	fscanf(in, "%s", buf);
	minz = atof(buf);

	fscanf(in, "%s", buf);
	if(strcmp(buf, "]") != 0)
		printf("] missing");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "p1") != 0)
		printf("p1 missing");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "[") != 0)
		printf("[ missing");

	fscanf(in, "%s", buf);
	maxx = atof(buf);
	fscanf(in, "%s", buf);
	maxy = atof(buf);
	fscanf(in, "%s", buf);
	maxz = atof(buf);

	fscanf(in, "%s", buf);
	if(strcmp(buf, "]") != 0)
		printf("] missing");

	fscanf(in, "%s", buf);
	if(strcmp(buf, "density") != 0)
		printf("density missing");
	fscanf(in, "%s", buf);
	if(strcmp(buf, "[") != 0)
		printf("[ missing");
	//begin reading density
	d = new float[nx*ny*nz];
	int total = nx*ny*nz;
	for(int i=0;i<total;i++)
	{
		fscanf(in, "%s", buf);
		d[i] = atof(buf);
	}
	fscanf(in, "%s", buf);
	if(strcmp(buf, "]") != 0)
		printf("] missing");
}