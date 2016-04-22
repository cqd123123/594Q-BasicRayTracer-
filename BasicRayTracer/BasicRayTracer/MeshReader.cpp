#include "MeshReader.h"
#include <cassert>

MReader::MReader(std::string name, std::vector<Gemometry* >& outGeom, ObjIO* dumobj, const Vector3f& _t, const Matrix3f& _r):trans(_t), rot(_r)
{
	bool perVertexN = false;
	bool perVertexM = false;
	if(!loadFromFile(name.c_str()))
	{
		printf("load file failed");
		return;
	}
	else
	{
// 		Triangle *tri = new Triangle;
// 		TransPoly2Tri(&ply, tri);
// 		TriangleNew* newtri = new TriangleNew(tri, _obj, perVertexN, perVertexM);
// 		geoms.push_back(newtri);
		for(int i=0;i<_numTriangles;i++)
		{
			Vector3i tt = _tlist[i];
			Vector3f pos1 = _vlist[tt(0)];
			Vector3f pos2 = _vlist[tt(1)];
			Vector3f pos3 = _vlist[tt(2)];
			Triangle *tri = new Triangle;
			tri->vert[0].pos = rot * pos1 + trans;
			tri->vert[1].pos = rot * pos2 + trans;
			tri->vert[2].pos = rot * pos3 + trans;
			TriangleNew* newtri = new TriangleNew(tri, dumobj, perVertexN, perVertexM);
			outGeom.push_back(newtri);
		}
	}
}

bool MReader::loadFromFile(const char* filename)
{
	FILE* inFile = fopen(filename, "rt");
	if (inFile == NULL)
	{
		printf( "%s does not exist!\n", filename );
		return false;
	}

	if (!readPlyHeader(inFile))
	{
		return false;
	}

	// read vertex data from PLY file
	if (!readPlyVerts(inFile))
	{
		return false;
	}

	// read triangle data from PLY file
	if (!readPlyTris(inFile))
	{
		return false;
	}

	fclose(inFile); // close the file

	return true;
}

bool MReader::readPlyHeader(FILE *&inFile)
{
	char tempStr[1024];

	// Read "ply" string
	do
	{
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			printf("Reached End of File and the string \"ply\" NOT FOUND!!\n");
			return false;
		}
		ChangeStrToLower(tempStr); // change tempStr to lower case 
	} while (strncmp(tempStr, "ply", 3));

	// Read # of verts
	if (!readNumPlyVerts(inFile, _numVerts))
	{
		return false;
	}

	// Read # of triangles
	if (!readNumPlyTris(inFile, _numTriangles))
	{
		return false;
	}

	// get end_header,读取文件结束标志
	do
	{
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			printf("Reached End of File and string \"end_header\" not found!\n");
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);
	} while (strncmp(tempStr, "end_header", 10));

	////////// end of header
	return true;
}

bool MReader::readNumPlyTris(FILE *&inFile, int& nTris) 
{
	bool bElementFound = false;
	/* Get number of faces in mesh*/
	for(;;)
	{
		char tempStr[1024];
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			printf("Reached End of File and string \"element face\" not found!\n");
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);

		if (bElementFound && !strncmp(tempStr, "face", 4))
		{
			break;
		}

		if (!strncmp(tempStr, "element", 7))
		{
			bElementFound = true;
			continue;
		}
	}

	fscanf(inFile, "%d", &nTris);
	if (feof(inFile))
	{
		printf("Reached End of File before list of vertices found!\n");
		return false;
	}
	return true;
}

bool MReader::readNumPlyVerts(FILE *&inFile, int& nVerts)
{
	// Read # of verts
	bool bElementFound = false;
	/* Get number of vertices in mesh*/
	for(;;)
	{       
		char tempStr[1024];
		fscanf(inFile, "%s", tempStr);
		if (feof(inFile))
		{
			printf("Can't read PlyVerts.\n");
			return false;
		}

		/* change tempStr to lower case */
		ChangeStrToLower(tempStr);

		if (bElementFound && !strncmp(tempStr, "vertex", 6))
		{
			break;
		}

		if (!strncmp(tempStr, "element", 7))
		{
			bElementFound = true;
			continue;
		}
	}

	fscanf(inFile, "%d", &nVerts); 
	if (feof(inFile))
	{
		printf("Reached End of File before \"element face\" found!\n");
		return false;
	}
	return true;
}
bool MReader::readPlyVerts(FILE *&inFile)
{
	int i;
	// read vertices
	for ( i = 0; i < _numVerts; i++)
	{
		char tempStr[1024];

#pragma warning(disable:4244)		/* disable double -> float warning */
		fscanf(inFile, "%s", tempStr);
		float x = atof(tempStr); 
		fscanf(inFile, "%s", tempStr);
		float y = atof(tempStr); 
		fscanf(inFile, "%s", tempStr);
		float z = atof(tempStr); 
#pragma warning(default:4244)		/* double -> float */

		Vector3f v(x,y,z);
		_vlist.push_back(v);

		if (feof(inFile))
		{
			printf("Reached End of File before all vertices found!\n");
			return false;
		}

		// read until end of line
		while (fgetc(inFile) != '\n');
	}
	return true;
}
bool MReader::readPlyTris(FILE *&inFile)
{
	int i;
	// read triangles
	for (i = 0; i < _numTriangles; i++)
	{
		int v1, v2, v3;
		int nVerts;
		fscanf(inFile, "%d", &nVerts);
		if (3 != nVerts)
		{
			printf("Error:  Ply file contains polygons which are not triangles!\n");
			return false;
		}
		fscanf(inFile, "%d", &v1);   // get value for vertex A
		fscanf(inFile, "%d", &v2);   // get value for vertex B
		fscanf(inFile, "%d", &v3);   // get value for vertex C

		// make sure verts in correct range
		assert(v1 < _numVerts && v2 < _numVerts && v3 < _numVerts);

		Vector3i t(v1,v2,v3);
		_tlist.push_back(t);

		// 		triangle t(this, v1, v2, v3);
		// 		t.setIndex(i);
		// 
		// 		_plist.push_back(t); // push_back puts a *copy* of the element at the end of the list
		// 
		// 		// update each vertex w/ its neighbors (vertrices & triangles)
		// 		_vlist[v1].addTriNeighbor(i);
		// 		_vlist[v1].addVertNeighbor(v2);
		// 		_vlist[v1].addVertNeighbor(v3);
		// 
		// 		_vlist[v2].addTriNeighbor(i);
		// 		_vlist[v2].addVertNeighbor(v1);
		// 		_vlist[v2].addVertNeighbor(v3);
		// 
		// 		_vlist[v3].addTriNeighbor(i);
		// 		_vlist[v3].addVertNeighbor(v1);
		// 		_vlist[v3].addVertNeighbor(v2);

		if (feof(inFile))
		{
			printf("Reached End of File before all faces found!\n");
			return false;
		}
		// read until end of line
		while (fgetc(inFile) != '\n');
	}
	return true;
}