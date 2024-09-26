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
unsigned int maxLevel = 3;
float offset = 0.01f;
float reflection = 1.f;
float refraction = 1.f;
float Ia = 0.1f;
Vec3Df backGroundColor = Vec3Df(0, 0, 0);
std::string filename = "CubeScene.obj";

// Map for texture lookup.
map<string, Texture> textures;

//temporary variables
//these are only used to illustrate 
//a simple debug drawing. A ray 
//Vec3Df testRayOrigin;
//Vec3Df testRayDestination;
vector<Ray> raysToDraw;

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
		if (material.has_illum() && material.illum() >= 3 && material.illum() != 9)
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

float cost(float cosi, float n1, float n2)
{
	float sin2t = pow(n1 / n2, 2) * (1.f - (cosi*cosi));
	return sqrtf(1.f - sin2t);
}

Vec3Df refract(Vec3Df i, Vec3Df n, float cosi, float n1, float n2)
{
	return (n1 / n2)*i + (n1 / n2*cosi - cost(cosi, n1, n2))*n;
}

float reflectance(const Vec3Df& normal, const Vec3Df& incident, float n1, float n2)
{
	const double n = n1 / n2;
	const double cosI = -Vec3Df::dotProduct(normal, incident);
	const double sinT2 = n * n * (1.f - cosI * cosI);
	if (sinT2 > 1.f) return 1.f;
	const double cosT = sqrt(1.0 - sinT2);
	const double rOrth = clamp((n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT), 0.0, 1.0);
	const double rPar = clamp((n2 * cosI - n1 * cosT) / (n2 * cosI + n1 * cosT), 0.0, 1.0);
	float res = (rOrth * rOrth + rPar * rPar) / 2.f;
	return res;
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
	float cosi = Vec3Df::dotProduct(-v, n);
	float n1 = ray.ior, n2 = material.Ni();
	float refl = 1.f;
	if (material.illum() >= 4 && material.illum() != 9) refl = reflectance(n, v, n1, n2);
	return trace(level + 1, reflected) * material.Ka() * refl;
}

// Computes the refracted light of a surface.
Vec3Df computeRefractedRay(int level, Ray &ray, Vertex &hit, Material &material)
{
	Vec3Df n = hit.n;
	n.normalize();
	Vec3Df v = hit.p - ray.origin;
	v.normalize();
	float ndotv = Vec3Df::dotProduct(n, v);
	float n1 = ray.ior, n2 = material.Ni();
	float trans = material.Tr();
	//Vec3Df t = n1 / n2 * (v - (ndotv * n)) - (n * sqrt(1 - (n1*n1*(1 - (ndotv*ndotv)) / (n2*n2))));
	float cosi = Vec3Df::dotProduct(-v, n);
	Vec3Df t = refract(v, n, cosi, n1, n2);
	Ray refract = Ray(hit.p, hit.p + t, n2);
	float refr = 1.f;
	if(material.illum() != 9) refr -= reflectance(n, v, n1, n2);
	return trace(level + 1, refract) * trans * refr;
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
		if (t < offset) {
			continue;
		}

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

// Shade method, returns the color by adding direct, reflected and refracted light.
Vec3Df Shade(unsigned int level, Ray &ray, Vertex &hit, Material &material, Vec3Df& texture)
{
	Vec3Df color = Vec3Df(0, 0, 0);

	Vertex dontcare;
	Material donotcareeither;
	Vec3Df IsaidIdonotcare;

	for (int i = 0; i < MyLightPositions.size(); ++i)
	{

		//if (intersect(level, Ray(hit.p, MyLightPositions[i]), dontcare, donotcareeither, IsaidIdonotcare)) {
		//	continue;
		//}

		color += computeDirectLight(ray, hit, material, texture);
	}

	if (material.has_illum() && material.illum() >= 3 && material.illum() != 9 && level <= maxLevel)
	{
		color += computeReflectedRay(level, ray, hit, material);
	}

	if (material.has_illum() && material.illum() >= 4 && level <= maxLevel)
	{
		color += computeRefractedRay(level, ray, hit, material);
	}

	return color;// / MyLightPositions.size();
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


		// For each ray in the raysToDraw vector, draw it!	
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	for (Ray curRay : raysToDraw) {
		// Set openGL's working color to the color the ray has.
		glColor3f(curRay.color[0], curRay.color[1], curRay.color[2]);
		// Draw a vertex at the origin of the ray
		glVertex3f(curRay.origin[0], curRay.origin[1], curRay.origin[2]);
		// Set openGL's working color to the color the ray has. (Maybe this one can be omitted,
		// but if you want a gradient you can select another color).
		glColor3f(curRay.color[0], curRay.color[1], curRay.color[2]);
		// Draw a vertex at the destination of the ray
		glVertex3f(curRay.destination[0], curRay.destination[1], curRay.destination[2]);
	}
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
void yourKeyboardFunc(char key, int x, int y, const Vec3Df & rayOrigin, const Vec3Df & rayDestination)
{

	switch (key)
	{
	//Clears the scene from debug rays.
	case 'c':
		raysToDraw.clear();
		std::cout << "Cleared the scene from debug rays" << std::endl;
		break;
	case 'd': //for debug
		Vertex hit;
		Material material;
		Vec3Df texture;
		Vec3Df rayColor;
		unsigned int level = 0;
		Ray ray = Ray(rayOrigin, rayDestination);
		if (intersect(level, ray, hit, material, texture)) // Intersect() sets the hit vertex, the material and the texture. 
		{
			rayColor = Shade(level, ray, hit, material, texture); // Shade() uses this hit vertex, material and texture.
		}
		else {
			rayColor = backGroundColor; // if the ray doesn't intersect anything, show the background color.
		}

		// The ray: Draw a ray from the camera to the hitpoint/intersectionpoint.
		raysToDraw.push_back(Ray(rayOrigin, hit.p, rayColor));

		// The Normal (red): Draw a ray from the hitpoint/intersectionpoint to that point plus the normal
		raysToDraw.push_back(Ray(hit.p, hit.p + hit.n, Vec3Df(1, 0, 0)));

		// The light ray: From each light source draw a ray to the hitpoint
		for (Vec3Df lightpos : MyLightPositions) {
			raysToDraw.push_back(Ray(lightpos, hit.p, Vec3Df(1, 1, 1)));
		}

		std::cout << "Added a debug ray (color of mesh),"<<endl<< "the normal on the plane (RED)" << endl << "and a ray to the light (mostly WHITE)." << std::endl;

		break;
	}

	std::cout << key << " pressed! The mouse was in location " << x << "," << y << "!" << std::endl;
}
