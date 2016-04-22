#ifndef IMAGE_BUFFER
#define IMAGE_BUFFER

#include <stdlib.h>
#include "CImg.h"
#include "util.h"
#include <cassert>
#include "Eigen/Eigen"
#include <vector>

using namespace Eigen;


using namespace cimg_library;

struct Pixel{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Vector3f color;
	float Z_value;
	float Tpancy;
};

struct PixelArray{
	std::vector<Pixel> pixls;
};


class Image_Buffer
{
public:
	// 	inline float& operator()(int x, int y, int z) { 
	// 		assert(x<_Width&&x>=0&&y<_Height&&y>=0&&z<3&&z>=0);
	// 		return data_buffer[(x + y*_Width)*3 + z]; 
	// 	};
	inline float& operator()(int x, int y, int z){
		assert(x<_Width&&x>=0&&y<_Height&&y>=0&&z<3&&z>=0);
		return pixel2D[x + y*_Width].pixls[0].color(z);
	}
	inline PixelArray& operator()(int x, int y){
		assert(x<_Width&&x>=0&&y<_Height&&y>=0);
		return pixel2D[x + y*_Width];
	}

	Image_Buffer(int Width, int Height);

	int get_width() {return _Width;}
	int get_height() {return _Height;}

	//z buffer !! if near z value, replace the color
	void take_frag_color(int x, int y, Vector3f color, float _z, float _tranpcy);

	void save_image_buffer(std::string name);

	void Enable_Transpancy(){
		_transparent_enabled = true;
	}
	void Disable_transpancy(){
		_transparent_enabled = false;
	}
	bool Get_tranpancy(){return _transparent_enabled;}
private:
	int _Width;
	int _Height;
	float* z_buffer;
	std::vector<PixelArray> pixel2D;
	bool _transparent_enabled;

};

class Texture{
public:

	inline Vector3f& operator()(int u, int v){
		if(!(u<_w&&u>=0&&v<_h&&v>=0)){//std::cout<<u<<" "<<v<<std::endl;
			u = clamp(u,0,_w-1);
			v = clamp(v,0,_h-1);}
		return _data[u + v*_w];
	}

	inline Vector3f Linear_interpolate_val(Vector2f pos);

	//is_h whether is height map
	Texture(std::string filename, bool is_h);

	//is_h whether is height map
	Texture(int ww, int hh, bool is_h):_w(ww),_h(hh), is_height(is_h)
	{
		_data =  (Vector3f*)malloc(sizeof(Vector3f)*_w*_h);
	}
	bool is_height;
	bool is_deformation;
private:
	Vector3f* _data;

	int _w;
	int _h;
};



#endif