#include "stdafx.h"
#pragma once

#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL\glut.h>
#include "raytracing.h"
#define _USE_MATH_DEFINES
#include <math.h>
#define cimg_use_png
#include "CImg.h"

using namespace cimg_library;
using namespace std;

//#define DEBUG

// Global variables
unsigned int maxLevel = 1;
float offset = 0.01f;
float reflection = 1.f;
float refraction = 1.f;
float Ia = 0.1f;
Vec3Df backGroundColor = Vec3Df(0, 0, 0);
std::string location = R"foo(C:\Users\Jim\Documents\TU Delft\TI 1806 Computer Graphics\Assignments\RaytracingProject\)foo";
std::string filename = "3_simple_cubes.obj";

// Map for texture lookup.
map<string, Texture> textures;

//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
Vec3Df testRayOrigin;
Vec3Df testRayDestination;

void printVec3Df(Vec3Df& v, const char* name)
{
#ifdef DEBUG
	printf("%s: 1: %.2f, 2: %.2f, 3: %.2f\n", name, v[0], v[1], v[2]);
#endif
}

// Texture point getter
Vec3Df getTextureAt(float x, float y, const char* texName)
{
	Texture& texture = textures[texName];
	unsigned int xcoord = x * texture.width;
	unsigned int ycoord = y * texture.height;
	return Vec3Df(
		texture.tex_img(xcoord, ycoord, 0, 0),
		texture.tex_img(xcoord, ycoord, 0, 1),
		texture.tex_img(xcoord, ycoord, 0, 2)) / 255;
}

//use this function for any preprocessing of the mesh.
void init()
{
	//load the mesh file
	//please realize that not all OBJ files will successfully load.
	//Nonetheless, if they come from Blender, they should, if they 
	//are exported as WavefrontOBJ.
	//PLEASE ADAPT THE LINE BELOW TO THE FULL PATH OF THE dodgeColorTest.obj
	//model, e.g., "C:/temp/myData/GraphicsIsFun/dodgeColorTest.obj", 
	//otherwise the application will not load properly
	MyMesh.loadMesh((filename).c_str(), true, textures);
	MyMesh.computeVertexNormals();

	//one first move: initialize the first light source
	//at least ONE light source has to be in the scene!!!
	//here, we set it to the current location of the camera
	MyLightPositions.push_back(MyCameraPosition);
}

// Generic clamp function.
// First argument is input, second and third number are the lower and upper bounds respectively.
template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
	return max(lower, min(n, upper));
}

// Ambient method.
Vec3Df ambient(Vec3Df Ka)
{
	Vec3Df result = Ka * Ia;
	//printVec3Df(result, "Ambient");
	return result;
}

// Diffuse method, takes light, normal and Kd vector.
Vec3Df diffuse(Vec3Df& L, Vec3Df& normal, Vec3Df Kd)
{
	normal.normalize();
	float dot = clamp(Vec3Df::dotProduct(normal, L), 0.f, INFINITY);
	Vec3Df result = Kd * dot;
	//printVec3Df(result, "Diffuse");
	return result;
}

// Specular method.
Vec3Df specular(Vec3Df& L, Vec3Df Camera, Vec3Df& normal, Vec3Df Ks, float shininess)
{
	normal.normalize();
	Vec3Df R = 2 * Vec3Df::dotProduct(L, normal) * normal - L;
	if (abs(acos(Vec3Df::dotProduct(L, normal)) / (L.getLength() * normal.getLength())) * 180 / M_PI > 90) return Vec3Df(0, 0, 0);
	if (abs(acos(Vec3Df::dotProduct(Camera, normal)) / (Camera.getLength() * normal.getLength())) * 180 / M_PI > 90) return Vec3Df(0, 0, 0);
	float dot = clamp(Vec3Df::dotProduct(Camera, R) / (R.getLength() * Camera.getLength()), 0.f, INFINITY);
	Vec3Df result = Ks * pow(dot, shininess);
	return result;
}

// Computes the direct light color of a surface.
Vec3Df computeDirectLight(Ray &ray, Vertex &hit, Material &material, Vec3Df &texture)
{
	Vec3Df color = Vec3Df(0, 0, 0);
	if (material.has_Ka() && material.has_illum() && (material.illum() == 1 || material.illum() == 2))
	{
		if (!material.textureName().empty())
		{
			color += ambient(texture);
		}
		else
		{
			color += ambient(material.Ka());
		}
	}

	if (material.has_Kd())
	{
		Vec3Df factor = Vec3Df(1, 1, 1);
		if (material.has_illum() && material.illum() >= 3)
		{
			factor = factor - material.Ka();
		}

		for (int i = 0; i < MyLightPositions.size(); ++i)
		{
			Vec3Df L = MyLightPositions[i] - hit.p;
			L.normalize();
			if (!material.textureName().empty())
			{
				color += diffuse(L, hit.n, texture) * factor;
			}
			else
			{
				color += diffuse(L, hit.n, material.Kd()) * factor;
			}
			if (material.has_Ks() && material.has_Ns() && material.has_illum() && material.illum() >= 2)
			{
				color += specular(L, ray.origin - hit.p, hit.n, material.Ks(), material.Ns());
			}
		}
	}
	//printVec3Df(color, "Color");
	return color;
}

// Computes the reflected light of a surface.
Vec3Df computeReflectedRay(int level, Ray &ray, Vertex &hit, Material &material)
{
	Vec3Df v = hit.p - ray.origin;
	v.normalize();
	Vec3Df n = hit.n;
	n.normalize();
	Vec3Df r = v - 2 * Vec3Df::dotProduct(n, v) * n;
	Ray reflected = Ray(hit.p, hit.p + r);
	return trace(level + 1, reflected) * material.Ka();
}

// Computes the refracted light of a surface.
Vec3Df computeRefractedRay(Ray &ray, Vertex &hit, Material &material)
{
	// TODO (optional)
	return backGroundColor;
}

// Shade method, returns the color by adding direct, reflected and refracted light.
Vec3Df Shade(unsigned int level, Ray &ray, Vertex &hit, Material &material, Vec3Df& texture)
{
	Vec3Df color = Vec3Df(0, 0, 0);

	for (int i = 0; i < MyLightPositions.size(); ++i)
	{
		color += computeDirectLight(ray, hit, material, texture);
	}

	if (material.has_illum() && material.illum() >= 3 && level <= maxLevel)
	{
		color += computeReflectedRay(level, ray, hit, material) * reflection;
	}

	if (material.has_illum() && material.illum() >= 4 && level <= maxLevel)
	{
		// TODO optional: refraction.
	}

	return color;
}

// Intersect method, returns true on intersect and returns hit and material values in corresponding arguments.
bool intersect(unsigned int level, Ray &ray, Vertex &hit, Material &material, Vec3Df& texture)
{
	// Check if we reached max recursion depth.
	if (level > maxLevel)
	{
		return false;
	}

	float distance = INFINITY;

	// Loop through all triangles.
	for (unsigned int i = 0; i < MyMesh.triangles.size(); ++i)
	{
		// Store vertices of triangle in temp variables.
		Vec3Df a = MyMesh.vertices[MyMesh.triangles[i].v[0]].p;
		Vec3Df b = MyMesh.vertices[MyMesh.triangles[i].v[1]].p;
		Vec3Df c = MyMesh.vertices[MyMesh.triangles[i].v[2]].p;

		// Store normals of vertices in temp variables.
		Vec3Df an = MyMesh.vertices[MyMesh.triangles[i].v[0]].n;
		Vec3Df bn = MyMesh.vertices[MyMesh.triangles[i].v[1]].n;
		Vec3Df cn = MyMesh.vertices[MyMesh.triangles[i].v[2]].n;

		Vec3Df ac = c - a;
		Vec3Df ab = b - a;

		// Calculate plane.
		Vec3Df n = Vec3Df::crossProduct(ac, ab);
		n.normalize();
		float D = Vec3Df::dotProduct(n, a);

		// Calculate intersection ray with plane.
		float t = (D - Vec3Df::dotProduct(ray.origin, n)) / Vec3Df::dotProduct(ray.direction, n);

		// This is the resulting intersection point.
		Vec3Df p = ray.origin + t * ray.direction;

		// Barycentric coordinate calculations.
		float areaABC = (Vec3Df::dotProduct(n, Vec3Df::crossProduct(ab, ac)));
		float areaPBC = (Vec3Df::dotProduct(n, Vec3Df::crossProduct(b - p, c - p)));
		float areaPCA = (Vec3Df::dotProduct(n, Vec3Df::crossProduct(c - p, a - p)));

		float alpha = areaPBC / areaABC;
		float beta = areaPCA / areaABC;
		float gamma = 1 - alpha - beta;

		if (isnan(alpha) || isnan(beta))
		{
			continue;
		}

		// Perform checks whether intersection is in triangle.
		if (alpha <= 0)
		{
			continue;
		}
		if (beta <= 0)
		{
			continue;
		}
		if (alpha + beta > 1)
		{
			continue;
		}

		float curDist = t;

		// Intersection is in triangle.
		// Now check if it is closer than previous intersection.
		if (curDist > offset && curDist < distance)
		{
			distance = curDist;
			Vec3Df normal = alpha * an + beta * bn + cn * gamma;
			normal.normalize();
			hit = Vertex(p, normal);
			material = MyMesh.materials[MyMesh.triangleMaterials[i]];
			if (!material.textureName().empty())
			{
				Vec3Df at = MyMesh.texcoords[MyMesh.triangles[i].t[0]];
				Vec3Df bt = MyMesh.texcoords[MyMesh.triangles[i].t[1]];
				Vec3Df ct = MyMesh.texcoords[MyMesh.triangles[i].t[2]];
				Vec3Df texCoord = alpha * at + beta * bt + gamma * ct;
				texture = getTextureAt(texCoord[0], texCoord[1], material.textureName().c_str());
			}
			//printVec3Df(texture, "Texture");
			//printf("Texture: %s\n", MyMesh.materials[MyMesh.triangleMaterials[i]].textureName().c_str());
		}
	}

	// Check if distance is updated, if updated then there is a hit so return true otherwise false.
	return distance != INFINITY;
}

// Trace method, returns the color.
Vec3Df trace(unsigned int level, Ray ray)
{
	Vertex hit;
	Material material;
	Vec3Df texture;
	if (intersect(level, ray, hit, material, texture))
	{
		return Shade(level, ray, hit, material, texture);
	}
	return backGroundColor;
}

//return the color of your pixel.
Vec3Df performRayTracing(const Vec3Df & origin, const Vec3Df & dest)
{
	testRayOrigin = origin;
	testRayDestination = dest;
	return trace(0, Ray(origin, dest));
}



void yourDebugDraw()
{
	//draw open gl debug stuff
	//this function is called every frame

	//let's draw the mesh
	MyMesh.draw();

	//let's draw the lights in the scene as points
	glPushAttrib(GL_ALL_ATTRIB_BITS); //store all GL attributes
	glDisable(GL_LIGHTING);
	glColor3f(1, 1, 1);
	glPointSize(10);
	glBegin(GL_POINTS);
	for (int i = 0; i < MyLightPositions.size(); ++i)
		glVertex3fv(MyLightPositions[i].pointer());
	glEnd();
	glPopAttrib();//restore all GL attributes
				  //The Attrib commands maintain the state. 
				  //e.g., even though inside the two calls, we set
				  //the color to white, it will be reset to the previous 
				  //state after the pop.


				  //as an example: we draw the test ray, which is set by the keyboard function
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(0, 1, 1);
	glVertex3f(testRayOrigin[0], testRayOrigin[1], testRayOrigin[2]);
	glColor3f(0, 0, 1);
	glVertex3f(testRayDestination[0], testRayDestination[1], testRayDestination[2]);
	glEnd();
	glPointSize(10);
	glBegin(GL_POINTS);
	glVertex3fv(MyLightPositions[0].pointer());
	glEnd();
	glPopAttrib();

	//draw whatever else you want...
	////glutSolidSphere(1,10,10);
	////allows you to draw a sphere at the origin.
	////using a glTranslate, it can be shifted to whereever you want
	////if you produce a sphere renderer, this 
	////triangulated sphere is nice for the preview
}


//yourKeyboardFunc is used to deal with keyboard input.
//t is the character that was pressed
//x,y is the mouse position in pixels
//rayOrigin, rayDestination is the ray that is going in the view direction UNDERNEATH your mouse position.
//
//A few keys are already reserved: 
//'L' adds a light positioned at the camera location to the MyLightPositions vector
//'l' modifies the last added light to the current 
//    camera position (by default, there is only one light, so move it with l)
//    ATTENTION These lights do NOT affect the real-time rendering. 
//    You should use them for the raytracing.
//'r' calls the function performRaytracing on EVERY pixel, using the correct associated ray. 
//    It then stores the result in an image "result.ppm".
//    Initially, this function is fast (performRaytracing simply returns 
//    the target of the ray - see the code above), but once you replaced 
//    this function and raytracing is in place, it might take a 
//    while to complete...
void yourKeyboardFunc(char t, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	//here, as an example, I use the ray to fill in the values for my upper global ray variable
	//I use these variables in the debugDraw function to draw the corresponding ray.
	//try it: Press a key, move the camera, see the ray that was launched as a line.
	testRayOrigin = rayOrigin;
	testRayDestination = rayDestination;

	// do here, whatever you want with the keyboard input t.

	//...


	std::cout << t << " pressed! The mouse was in location " << x << "," << y << "!" << std::endl;
}
