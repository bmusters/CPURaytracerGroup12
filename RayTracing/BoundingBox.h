#pragma once
#include <vector>
#include "mesh.h"
#include "Vec3D.h"
#include "Geometry.h"

class BoundingBox {
public:
	BoundingBox();
	BoundingBox(Vec3Df, Vec3Df, std::vector<Triangle>, int, Mesh);
	BoundingBox(std::vector<Triangle>, int, Mesh);

	void resetColor();
	void drawBox(int, bool, bool);
	bool intersect(BoundingBox*, Ray, Vec3Df& , Vec3Df&);
	bool hasChildren;
	bool contains(Vec3Df);
	Vec3Df point1;
	Vec3Df point2;
	std::vector<Triangle> triangles;
	BoundingBox* leftChild;
	BoundingBox* rightChild;

private:
	
	int boxSize;
	Mesh* myMesh;
	void generateBoxes();
	Vec3Df color;

	float calculateScalar(float plane, float origin, float dir);
	float minfloat(float f1, float f2);
	float minfloat(float f1, float f2, float f3);
	float maxfloat(float f1, float f2);
	float maxfloat(float f1, float f2, float f3);
};

/*#pragma once
#include <vector>
#include "Vec3D.h"
#include "mesh.h"

class BoundingBox
{
public:
	BoundingBox(Vec3Df point1, Vec3Df point2, std::vector<Triangle> triangles, int boxSize, Mesh* myMesh);
	BoundingBox(void) {};

private:
	Vec3Df point1;
	Vec3Df point2;
	int boxSize;
	std::vector<Triangle> triangles;
	BoundingBox* leftChild;
	BoundingBox* rightChild;
	Mesh* myMesh;

	void generateBoxes();
};

*/