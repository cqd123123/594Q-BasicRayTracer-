#include "Image_Buffer.h"
#include <iostream>

Image_Buffer::Image_Buffer(int Width, int Height)
{
	int total_size = Width*Height*3;
	z_buffer = (float *) malloc(sizeof(float)*Width*Height);
	//memset(z_buffer, FLT_MAX, sizeof(float)*Width*Height);
	for(int i=0; i<Width*Height;i++)
		z_buffer[i] = FLT_MAX;

	_Width =Width;
	_Height = Height;

	//pixels = (Pixel*) malloc(sizeof(Pixel)*_Width*_Height);
	for(int i=0; i<_Width*_Height;i++)
	{
		Pixel pix;
		pix.color = Vector3f(0,0,0);
		pix.Tpancy = 0.0f;
		pix.Z_value = 0.0;

		PixelArray pa;
		pa.pixls.push_back(pix);

		pixel2D.push_back(pa);
	}
	_transparent_enabled = false;
}

void Image_Buffer::take_frag_color(int x, int y, Vector3f color, float _z, float _tranpcy)
{
	//compare z value with z buffer
	if(!_transparent_enabled)
	{
		//if(_z < z_buffer[x + y*_Width])
		//{
		z_buffer[x + y*_Width] = _z;
		(*this)(x,y).pixls[0].color = color;
		//}
	}
	else	//save every thing
	{		
		if(_tranpcy>1.0 || _tranpcy<0.0) std::cout<<"warning invalid transparency value"<<std::endl;

		Pixel pix;
		pix.color = color;
		pix.Z_value = _z;
		pix.Tpancy = _tranpcy;
		(*this)(x,y).pixls.push_back(pix);
	}
}
void Image_Buffer::save_image_buffer(std::string name)
{
	CImg<float> image;
	image.resize(_Width,_Height,1,3);


	if(!_transparent_enabled)
	{
		for(int j=0;j<_Height;j++)
		{
			for(int i=0;i<_Width;i++)
			{
				//for(int k=0;k<3;k++)
				//	image(i,j,k) = clamp((*this)(i,j,k)*255.f,0.f,255.f);
				Vector3f C;
				C(0) = (*this)(i,j,0);
				C(1) = (*this)(i,j,1);
				C(2) = (*this)(i,j,2);
				if(C.norm() > 3.f)
					C.normalize();
				image(i,j,0) = clamp( C(0) * 255.f, 0.f,255.f);
				image(i,j,1) = clamp( C(1) * 255.f, 0.f,255.f);
				image(i,j,2) = clamp( C(2) * 255.f, 0.f,255.f);
			}
		}
	}
	else{

		//sort pixels from far to near pixelarray [0] far [1] near
		for(int j=0;j<_Height;j++)
		{
			for(int i=0;i<_Width;i++)
			{
				PixelArray& pa = (*this)(i,j);
				for(int k=0;k<pa.pixls.size();k++)
				{
					for(int z=k;z<pa.pixls.size();z++)
					{
						Pixel tmp;
						if(pa.pixls[k].Z_value < pa.pixls[z].Z_value)
						{
							tmp = pa.pixls[k];
							pa.pixls[k] = pa.pixls[z];
							pa.pixls[z] = tmp;
						}
					}
				}
			}
		}

		//composite color
		for(int j=0;j<_Height;j++)
		{
			for(int i=0;i<_Width;i++)
			{
				PixelArray pa = (*this)(i,j);
				Vector3f fcolor = Vector3f(0,0,0);	//black background

				for(int k=0;k<pa.pixls.size();k++)
				{
					Pixel px = pa.pixls[k];
					fcolor = px.color*px.Tpancy + (1.f - px.Tpancy)*fcolor;

				}

				for(int k=0;k<3;k++)
					image(i,j,k) = clamp(fcolor(k)*255.f,0.f,255.f);
			}
		}
	}
//	image.display();
	image.save(name.c_str());
}
Texture::Texture(std::string filename, bool is_h):is_height(is_h){
	CImg<float> _tex(filename.c_str());
	_w = _tex.width();
	_h = _tex.height();

	_data =  (Vector3f*)malloc(sizeof(Vector3f)*_w*_h);

	for(int j=0;j<_h;j++)
	{
		for(int i=0;i<_w;i++)
		{

			Vector3f texcol;
			texcol(0) = _tex(i,j,0);
			texcol(1) = _tex(i,j,1);
			texcol(2) = _tex(i,j,2);
			(*this)(i,j) = texcol / 255.0f; //scale to [0-1]
		}
	}
	is_deformation = false;
}
inline Vector3f Texture::Linear_interpolate_val(Vector2f pos)
{
	//assert(pos(0)<=1.f&&pos(0)>=0.f&&pos(1)<=1.f&&pos(1)>=0.f);
	if(!(pos(0)<=1.f&&pos(0)>=0.f&&pos(1)<=1.f&&pos(1)>=0.f))
	{
		//std::cout<<pos(0)<<"  "<<pos(1)<<std::endl;
		//pos(0) = clamp(pos(0),0.f,1.f);
		//pos(1) = clamp(pos(1),0.f,1.f);
		//return Vector3f(0,0,0);
		pos(0) = rand()/RAND_MAX;
		pos(1) = rand()/RAND_MAX;
	}
	float u = pos(0)*_w;
	float v = pos(1)*_h;

	int u0 = (int)(pos(0)*_w);
	int v0 = (int)(pos(1)*_h);
	int u1 = min(u0+1, _w -1);
	int v1 = min(v0+1, _h -1);

	//interpolate

	Vector3f cx0 = LinearInterpolate(u, u0, u1, (*this)(u0,v0), (*this)(u1,v0));
	Vector3f cx1 = LinearInterpolate(u, u0, u1, (*this)(u0,v1), (*this)(u1,v1));
	Vector3f fcl = LinearInterpolate(v, v0, v1, cx0, cx1);
	return fcl;
}