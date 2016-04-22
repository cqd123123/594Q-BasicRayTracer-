#ifndef VOLUME_H
#define VOLUME_H

#include "Ray.h"
#include "BoundingBox.h"

class VolumeRegion {
public:
	// VolumeRegion Interface
 //	virtual ~VolumeRegion();
 //	virtual BBox WorldBound() const = 0;
 	virtual bool IntersectP(const Ray &ray, float *t0, float *t1) const = 0;
 	virtual Vector3f sigma_a(const Vector3f &) const = 0;
 	virtual Vector3f sigma_s(const Vector3f &) const = 0;
 	virtual Vector3f Lve(const Vector3f &) const = 0;
 	virtual float p(const Vector3f &, const Vector3f &, const Vector3f &) const = 0;
 	virtual Vector3f sigma_t(const Vector3f &) const = 0;
 	virtual Vector3f tau(const Ray &ray, float step = 1.f,
 		float offset = 0.5) const = 0;

	int nSamples;

	//virtual float Reture_Alp() const = 0;
};

class HomogeneousVolumeDensity: public VolumeRegion
{
public:
	HomogeneousVolumeDensity(const Vector3f &sa, const Vector3f &ss, float gg,
		const Vector3f &emit, const BBox &e, const Matrix3f &v2w, const Vector3f & wordtrans) {
			//WorldToVolume = Inverse(v2w);
			sig_a = sa;
			sig_s = ss;
			g = gg;
			le = emit;
			bbox = e;
			Model_Rotate = v2w;
			Model_trans = wordtrans;
			Model_Rotate_invT = (Model_Rotate.transpose()).inverse();
			Model_Rotate_inv  = (Model_Rotate.transpose());
	}
	~HomogeneousVolumeDensity(){;}
	BBox WorldBound() {return bbox;}
private:
	void WorldToObject(const Ray &Rin, Ray &Rout) const;
	bool IntersectP(const Ray &r, float *t0, float *t1) const;
	Vector3f sigma_a(const Vector3f & pos) const ;
	Vector3f sigma_s(const Vector3f & pos) const ;
	Vector3f sigma_t(const Vector3f & pos) const ;
	Vector3f Lve(const Vector3f& pos) const;
	float    p(const Vector3f & pos, const Vector3f & wi, const Vector3f & wo) const;
	Vector3f tau(const Ray &ray, float step = 1.f, float offset = 0.5) const;

	Vector3f sig_a, sig_s, le;
	float g;
	BBox bbox;
	//Transform WorldToVolume;
	Matrix3f Model_Rotate;
	Matrix3f Model_Rotate_inv;
	Matrix3f Model_Rotate_invT;
	Vector3f Model_trans;
};

class VolumeFileReader
{
public:
	VolumeFileReader(std::string _n);
	~VolumeFileReader() {delete d;};
	float* d;
	int nx, ny, nz;
	float minx,miny,minz;
	float maxx, maxy, maxz;
private:
	std::string name;
};

class VolumeGridDensity : public VolumeRegion {
public:
	// VolumeGridDensity Public Methods
	VolumeGridDensity(const Vector3f &sa, const Vector3f &ss, float gg,
		const Vector3f &emit, const BBox &e, const Matrix3f &v2w, const Vector3f & wordtrans,
		int x, int y, int z, const float *d)
		:sig_a(sa), sig_s(ss), g(gg), le(emit), Model_Rotate(v2w), nx(x), ny(y), nz(z), bbox(e) {
			Model_trans = wordtrans;
			Model_Rotate_invT = (Model_Rotate.transpose()).inverse();
			Model_Rotate_inv  = (Model_Rotate.transpose());
			density = new float[nx*ny*nz];
			if(d != NULL)
				memcpy(density, d, nx*ny*nz*sizeof(float));
	}
	~VolumeGridDensity() { delete[] density; }
	BBox WorldBound() const { bbox; }
	bool IntersectP(const Ray &r, float *t0, float *t1) const;

	float Density(const Vector3f &Pobj) const;
	float D(int x, int y, int z) const {
		x = clamp(x, 0, nx-1);
		y = clamp(y, 0, ny-1);
		z = clamp(z, 0, nz-1);
		return density[z*nx*ny + y*nx + x];
	}
private:
	void WorldToObject(const Ray &Rin, Ray &Rout) const;
	Vector3f sigma_a(const Vector3f & pos) const ;
	Vector3f sigma_s(const Vector3f & pos) const ;
	Vector3f sigma_t(const Vector3f & pos) const ;
	Vector3f Lve(const Vector3f& pos) const;
	float    p(const Vector3f & pos, const Vector3f & wi, const Vector3f & wo) const;
	Vector3f tau(const Ray &ray, float step = 1.f, float offset = 0.5) const;

	Vector3f sig_a, sig_s, le;
	float g;
	float *density;
	const int nx, ny, nz;
	BBox bbox;
	Matrix3f Model_Rotate;
	Matrix3f Model_Rotate_inv;
	Matrix3f Model_Rotate_invT;
	Vector3f Model_trans;
};
#endif