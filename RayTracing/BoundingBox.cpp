#include "BoundingBox.h"
#include "Vec3D.h"
#include "mesh.h"
#include <vector>
#include "GL/glut.h"
#include <iostream>
#include "GL/glut.h"

using namespace std;

Vec3Df point1;
Vec3Df point2;
int boxSize;
vector<Triangle> triangles;
BoundingBox* leftChild;
BoundingBox* rightChild;
bool hasChildren;
Mesh myMesh;
float boxOffset = 0.0001f;
Vec3Df color;

// constructor
BoundingBox::BoundingBox() {}

//constructor
BoundingBox::BoundingBox(Vec3Df min, Vec3Df max, vector<Triangle> list,  int size, Mesh mesh)
{
	point1 = min;
	point2 = max;
	triangles = list;
	boxSize = size;
	myMesh = &mesh;
	hasChildren = false;
	color = Vec3Df(0.0f, 0.0f, 1.0f);

	generateBoxes();
}

// constructor boudingbox with the size of the scene
BoundingBox::BoundingBox(std::vector<Triangle> list, int size, Mesh mesh) {
	triangles = list;
	boxSize = size;
	myMesh = &mesh;
	hasChildren = false;
	color = Vec3Df(0.0f, 0.0f, 1.0f);
	
	float floatMin = numeric_limits<float>::min();
	float floatMax = numeric_limits<float>::max();

	Vec3Df min = Vec3Df(floatMax, floatMax, floatMax);
	Vec3Df max = Vec3Df(floatMin, floatMin, floatMin);

	for (Triangle triangle : list) {
		// for each vertex in the triangle
		for (int i = 0; i < 3; i++) {
			// myMesh points to mesh, get the right point from the good vertex
			Vec3Df vertex = myMesh->vertices[triangle.v[i]].p;
			
			// For every plane in the vertex compare it with the max and min
			for (int j = 0; j < 3; j++) {
				min[j] = std::min(min[j], vertex[j]);
				max[j] = std::max(max[j], vertex[j]);
			}
		}
	}

	point1 = min;
	point2 = max;

	generateBoxes();
}

// generates left en right children
void BoundingBox::generateBoxes() {
	if (triangles.size() <= boxSize) {
		return;
	}

	// calculate the longest axis
	float axisLength;
	float length = 0;
	int index;
	for (int i = 0; i < 3; i++) {
		axisLength = abs(point1[i] - point2[i]);
		if (length <= axisLength) {
			length = axisLength;
			index = i;
		}
	}

	Vec3Df point3;
	Vec3Df point4;

	//calculate point 3 and 4 for the boxes
	for (int i = 0; i < 3; i++) {
		if (i == index) {
			point3[i] = point2[i] - (length / 2);
			point4[i] = point1[i] + (length / 2);
		}
		else {
			point3[i] = point2[i];
			point4[i] = point1[i];
		}
	}

	float splitValue;
	// determine splitValue
	splitValue = point3[index];
	

	vector<Triangle> leftTriangles;
	vector<Triangle> rightTriangles;
	// determine for all triangles in which list they should go
	for (Triangle triangle : triangles) {

		bool left = false;
		bool right = false;

		// for each vertex in the triangle
		for (int i = 0; i < 3; i++) {
			// myMesh points to mesh, get the right point from the good vertex
			float value = myMesh->vertices[triangle.v[i]].p[index];
			if ( value >= splitValue) {
				right = true;
			}
			if (value <= splitValue) {
				left = true;
			}

		}

		if (left) {
			leftTriangles.push_back(triangle);
		}
		if (right) {
			rightTriangles.push_back(triangle);
		}
	}
	hasChildren = true;
	leftChild = new BoundingBox(point1, point3, leftTriangles, boxSize, *myMesh);
	rightChild = new BoundingBox(point4, point2, rightTriangles, boxSize, *myMesh);
	
}

void BoundingBox::resetColor() {
	color = Vec3Df(0.0f, 0.0f, 1.0f);

	if (hasChildren) {
		leftChild->resetColor();
		rightChild->resetColor();
	}
};

// checks if a box contains a point
bool BoundingBox::contains(Vec3Df point) {

	for (int i = 0; i < 3; i++) {
		if (point[i] > point2[i] + boxOffset) {
			return false;
		}
		if (point[i] < point1[i] - boxOffset) {
			return false;
		}
	}
	
	color = Vec3Df(0.0f,1.0f,0.0f);
	return true;
}

// debug tool for drawing a boudingbox
void BoundingBox::drawBox(int divider, bool hideBlue, bool hideGreen) {
	if (divider == 0) {
		return;
	}

	if (hasChildren) {
		leftChild->drawBox(divider, hideBlue, hideGreen);
		rightChild->drawBox(divider, hideBlue, hideGreen);
	}
	else
	{
		if ((color[2] == 1.0f && !hideBlue) || (color[1] == 1.0f && !hideGreen)) {
			vector<Vec3Df> corners;
			Vec3Df length = point2 - point1;

			corners.push_back(Vec3Df(point1[0], point1[1], point1[2]) / divider);
			corners.push_back(Vec3Df(point1[0] + length[0], point1[1], point1[2]) / divider);
			corners.push_back(Vec3Df(point1[0] + length[0], point1[1] + length[1], point1[2]) / divider);
			corners.push_back(Vec3Df(point1[0], point1[1] + length[1], point1[2]) / divider);

			corners.push_back(Vec3Df(point1[0], point1[1], point1[2] + length[2]) / divider);
			corners.push_back(Vec3Df(point1[0] + length[0], point1[1], point1[2] + length[2]) / divider);
			corners.push_back(Vec3Df(point1[0] + length[0], point1[1] + length[1], point1[2] + length[2]) / divider);
			corners.push_back(Vec3Df(point1[0], point1[1] + length[1], point1[2] + length[2]) / divider);

			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_LIGHTING);
			glBegin(GL_LINES);

			glColor3f(color[0], color[1], color[2]);
			for (int j = 0; j < 2; j++) {
				for (int i = j * 4; i < 4 + j * 4; i++) {
					int nextIndex = i + 1;
					if (nextIndex > (3 + j * 4)) {
						nextIndex = 0 + j * 4;
					}
					glVertex3f(corners[i][0], corners[i][1], corners[i][2]);

					glVertex3f(corners[nextIndex][0], corners[nextIndex][1], corners[nextIndex][2]);
				}
			}

			for (int i = 0; i < 4; i++) {
				int nextIndex = i + 4;
				glVertex3f(corners[i][0], corners[i][1], corners[i][2]);
				glVertex3f(corners[nextIndex][0], corners[nextIndex][1], corners[nextIndex][2]);
			}

			glEnd();
			glPopAttrib();
		}
	}
}

// calculate scalar for intersection
float BoundingBox::calculateScalar(float plane, float origin, float dir) {
	if (dir == 0.f) {
		return NULL;
		
	}
	return (plane - origin) / dir;
}

// calculates the minimun
float BoundingBox::minfloat(float f1, float f2) {
	if (f1 == NULL) {
		return f2;
		
	}
	if (f2 == NULL) {
		return f1;
		
	}
		
	if (f1 < f2) {
		return f1;
		
	}
	else {
		return f2;	
	}
	
}

// calculates the minimum
float BoundingBox::minfloat(float f1, float f2, float f3) {
	return minfloat(minfloat(f1, f2), f3);
}

// calculates the maximum
float BoundingBox::maxfloat(float f1, float f2) {
	if (f1 == NULL) {
		return f2;
		
	}
	if (f2 == NULL) {
		return f1;
		
	}
	
	if (f1 > f2) {
		return f1;
		
	}
	else {
		return f2;
		
	}
}

// calculates the maximum
float BoundingBox::maxfloat(float f1, float f2, float f3) {
	return maxfloat(maxfloat(f1, f2), f3);
	
}

// calculates the intersection
bool BoundingBox::intersect(BoundingBox* box, Ray ray, Vec3Df& p_in, Vec3Df& p_out) {

	Vec3Df min = box->point1;
	Vec3Df max = box->point2;

	Vec3Df origin = ray.origin;
	Vec3Df dir = ray.direction;

	Vec3Df t_min;
	Vec3Df t_max;
	
	for (int i = 0; i < 3; i++) {
		t_min[i] = calculateScalar(min[i], origin[i], dir[i]);
		t_max[i] = calculateScalar(max[i], origin[i], dir[i]);
	}
	
	Vec3Df t_in;
	Vec3Df t_out;
	
	for (int i = 0; i < 3; i++) {
		t_in[i] = minfloat(t_min[i], t_max[i]);
		t_out[i] = maxfloat(t_min[i], t_max[i]);
	}
	
	float tin = maxfloat(t_in[0], t_in[1], t_in[2]);
	float tout = minfloat(t_out[0], t_out[1], t_out[2]);
	
	if (tin > tout || tout < 0) {
		return false;
	}
	
	p_in = origin + tin * dir;
	p_out = origin + tout * dir;
	
	return true;
}