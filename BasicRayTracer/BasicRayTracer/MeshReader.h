#ifndef MESH_READER_H
#define MESH_READER_H

#include <iostream>
#include <Eigen/Eigen>
#include "Gemetry.h"

using namespace Eigen;

class MReader{
public:
	MReader(std::string name, std::vector<Gemometry* >& outGeom, ObjIO*  buuObj, const Vector3f& _T, const Matrix3f& _r);
	~MReader(){_vlist.clear();}
	bool readNumPlyVerts(FILE *&inFile, int& nVerts);
	bool readNumPlyTris(FILE *&inFile, int& nTris);
	bool readPlyHeader(FILE *&inFile);
	bool readPlyVerts(FILE *&inFile);
	bool readPlyTris(FILE *&inFile);
	bool loadFromFile(const char* filename);
	void ChangeStrToLower(char* pszUpper)
	{
		for(char* pc = pszUpper; pc < pszUpper + strlen(pszUpper); pc++) {
			*pc = (char)tolower(*pc);
		}
	}
	int _numVerts;
	int _numTriangles;
	std::vector<Vector3f> _vlist;
	std::vector<Vector3i> _tlist;
	Vector3f trans;
	Matrix3f rot;
};

#endif