#pragma once

#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "Vec3D.h"
#define cimg_use_png
#include "CImg.h"

/*
* Ray class to use instead of origin, destination coordinates.
*/
class Ray
{
public:
	Vec3Df origin;
	Vec3Df direction;
	float ior;
	Vec3Df destination;
	Vec3Df color = Vec3Df(1.f, 1.f, 1.f);

	Ray(Vec3Df o, Vec3Df dest) : origin(o)
	{
		destination = dest;
		direction = dest - o;
		direction.normalize();
		ior = 1;
	}
	Ray(Vec3Df o, Vec3Df dest, Vec3Df col) : origin(o)
	{
		destination = dest;
		color = col;
		direction = dest - o;
		direction.normalize();
		ior = 1;
	}

	Ray(Vec3Df o, Vec3Df dest, float i) : origin(o), ior(i)
	{
		direction = dest - o;
		direction.normalize();
	}
};

/*
* Texture class for storing a texture image and its dimensions.
*/
class Texture
{
public:
	int width;
	int height;
	cimg_library::CImg<float> tex_img;

	Texture(cimg_library::CImg<float>& ti) : tex_img(ti)
	{
		width = ti.width();
		height = ti.height();
	}

	Texture()
	{
		width = height = 0;
	}
};
#endif