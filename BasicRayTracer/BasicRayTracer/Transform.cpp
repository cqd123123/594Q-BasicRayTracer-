#include "Transform.h"

TransformV::TransformV()
{
	ViewPointMatrix.setZero();
	World2CameraMatrix.setZero();
	RotationMatrix.setZero();
	TotalTransMatrix.setZero();

	for(int i=0;i<4;i++)	//init to indentity
	{
		ViewPointMatrix(i,i) = 1.0;
		World2CameraMatrix(i,i) = 1.0;
		RotationMatrix(i,i) = 1.0;
		TotalTransMatrix(i,i) = 1.0;
	}
	//init camera at zero, faceing z + axis
	CameraPos = Vector3f(0,0,0);
	TotalTransMatrix = ViewPointMatrix * World2CameraMatrix;

	DOF_Enableed = false;
}
void TransformV::SetViewProjectMatrix(float angleOfView, float nearClip, float farClip)
{
	Eigen::Matrix4f mat;
	mat.setZero();
	float scale = 1.f / tan(angleOfView * 0.5f * M_PI / 180.f);
	float temp = 1.f / 2.f;
	mat(0,0) = scale;
	mat(1,1) = scale;
	mat(2,2) = - farClip / (farClip - nearClip);
	mat(3,2) = - 1.0;
	mat(2,3) =  farClip*nearClip/(farClip - nearClip);

	ViewPointMatrix = mat;
	TotalTransMatrix = ViewPointMatrix * World2CameraMatrix;

	RaDFov = angleOfView*M_PI / 180.f;
}
void TransformV::TransformCamera(Vector3f dir)
{
	World2CameraMatrix(0,3) += dir(0);
	World2CameraMatrix(1,3) += dir(1);
	World2CameraMatrix(2,3) += dir(2);
	CameraPos -= dir;
	TotalTransMatrix = ViewPointMatrix * World2CameraMatrix;
	//std::cout<<TotalTransMatrix<<std::endl;
}

void TransformV:: LookAt(Vector3f pos,Vector3f look,Vector3f up)
{
	Matrix4f mat;
	mat(0,3) = pos(0);
	mat(1,3) = pos(1);
	mat(2,3) = pos(2);
	mat(3,3) = 1.f;
	Vector3f dir = (look - pos);
	dir.normalize(); up.normalize();
	Vector3f right = dir.cross(up);
	Vector3f NewUp = right.cross(dir);
	mat(0,0) = right(0);
	mat(1,0) = right(1);
	mat(2,0) = right(2);
	mat(3,0) = 0.f;
	mat(0,1) = NewUp(0);
	mat(1,1) = NewUp(1);
	mat(2,1) = NewUp(2);
	mat(3,1) = 0.f;
	mat(0,2) = dir(0);
	mat(1,2) = dir(1);
	mat(2,2) = dir(2);
	mat(3,2) = 0.f;

	World2CameraMatrix = mat.inverse();
}

inline void TransformV::TransVertex(Vector3f in, Vector3f& out)
{
	Matrix4f& mat = this->TotalTransMatrix;	//homogenious coordinates
	out(0) = mat(0,0)*in(0) + mat(0,1)*in(1) + mat(0,2)*in(2) + mat(0,3);
	out(1) = mat(1,0)*in(0) + mat(1,1)*in(1) + mat(1,2)*in(2) + mat(1,3);
	out(2) = mat(2,0)*in(0) + mat(2,1)*in(1) + mat(2,2)*in(2) + mat(2,3);
	float w = mat(3,0)*in(0) + mat(3,1)*in(1) + mat(3,2)*in(2) + mat(3,3);
	if(w!=1.0 && w!=0.0)
	{
		out = out/w;
	}
}

inline void TransformV::TransVertexNew(Vector3f& in, Vector3f& out)
{
	Matrix4f& mat = this->TotalTransMatrix;	//homogenious coordinates
	out(0) = mat(0,0)*in(0) + mat(0,1)*in(1) + mat(0,2)*in(2) + mat(0,3);
	out(1) = mat(1,0)*in(0) + mat(1,1)*in(1) + mat(1,2)*in(2) + mat(1,3);
	out(2) = mat(2,0)*in(0) + mat(2,1)*in(1) + mat(2,2)*in(2) + mat(2,3);
	float w = mat(3,0)*in(0) + mat(3,1)*in(1) + mat(3,2)*in(2) + mat(3,3);
	if(w!=1.0 && w!=0.0)
	{
		out(0) = out(0)/w;
		out(1) = out(1)/w;
	}
}

inline bool TransformV::QuadOutofFrustum(Vector3f* _in)
{
	//both of the four vertices out of range
	bool is_out = true;
	for(int i=0;i<4;i++)
	{
		if(!OutofImagePlane(_in[i]))	//either of the point is inside the frustum
			return false;
	}
	return true;	//four is out of frustum
}
float TransformV::GenerateRay(const float& imgx,const float &imgy, Ray* ray, const float & c, const int _w, const int _h) const
{
	//sample pos relative to camera
	float Ratio = float(_w)/ _h;
	Vector3f ppos;
	Ray insideRay;
	//Vector3f HorizonV(1,0,0);
	float Theta = RaDFov * Ratio;
	float  Y = c*tanf(RaDFov * 0.5f);
	float  X = c*tanf(Theta * 0.5f);

	ppos(0) = (2.f*imgx - 1.f)*X;	//scaled to [-X, X]
	ppos(1) = (2.f*imgy - 1.f)*Y;   //scaled to [-Y, Y]
	ppos(2) = -c;			//in camera cord, camera  at (0,0,0)
	insideRay.Origin = ppos;
	ppos.normalize();
	insideRay.Direction = - ppos;

	if(DOF_Enableed)
	{
		float t = (Focus_D + c)/c;
		Vector3f PP = insideRay.Origin + insideRay.Direction * t;
		float randx = ((float) rand()) / (float) RAND_MAX;	// in the range [0 - 1]
		float randy = ((float) rand()) / (float) RAND_MAX;
		randx = 2.f*randx - 1.f;		//[-1, 1]
		randy = 2.f*randy - 1.f;
		Vector3f DofOri;
		DofOri(0) = randx*X;
		DofOri(1) = randy*Y;
		DofOri(2) = 0;
		Vector3f DofDir;
		DofDir = PP - DofOri;
		DofDir.normalize();
		insideRay.Direction = DofDir;
		insideRay.Origin = DofOri;
	}
	insideRay.tmin = 0;
	insideRay.tmax = FLT_MAX;
	//Ray result;
	Camera2World(insideRay, *ray);
	return 1.f;
}

void TransformV::Camera2World(const Ray& _in, Ray& _out) const
{
	Matrix4f c2w = World2CameraMatrix.inverse();
	Vector3f _ori = _in.Origin;
	Vector3f _dir = _in.Direction;
	Vector4f homori(_ori(0),_ori(1),_ori(2),1.f);
	Vector4f homdir(_dir(0),_dir(1),_dir(2),0.f);
	Vector4f Thomori = c2w * homori;
	Vector4f Thomdir = c2w * homdir;
	assert(Thomdir(3) == 0.f && Thomori(3) != 0.f);
	Thomori = Thomori / Thomori(3);
	_out.Origin = Vector3f(Thomori(0), Thomori(1), Thomori(2));
	_out.Direction = Vector3f(Thomdir(0), Thomdir(1), Thomdir(2));
	_out.tmin = _in.tmin;
	_out.tmax = _in.tmax;
}