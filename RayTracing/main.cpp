#include "stdafx.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL\glut.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "raytracing.h"
#include "mesh.h"
#include "traqueboule.h"
#include "imageWriter.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <time.h>
#include <thread>
#include <algorithm>
#include <cstdlib> 


//This is the main application
//Most of the code in here, does not need to be modified.
//It is enough to take a look at the function "drawFrame",
//in case you want to provide your own different drawing functions



Vec3Df MyCameraPosition;

//MyLightPositions stores all the light positions to use
//for the ray tracing. Please notice, the light that is 
//used for the real-time rendering is NOT one of these, 
//but following the camera instead.
std::vector<Vec3Df> MyLightPositions;
std::vector<Vec3Df> MyLightColors; //idices correspond to the positions

//Main mesh 
Mesh MyMesh;


unsigned static int THREADS = std::thread::hardware_concurrency() + 20;
static int NUMBER_OF_LIGHTS_PER_SPHERE = 400;
static float RADIUS_OF_LIGHT_SPHERE = 0.1f;
bool DEBUGGING = false;
bool DRAW_NORMALS = true;
bool DRAW_RAYS_TO_LIGHT = true;

unsigned int WindowSize_X = 1024;  // resolution X
unsigned int WindowSize_Y = 768;  // resolution Y




								  /**
								  * Main function, which is drawing an image (frame) on the screen
								  */
void drawFrame()
{
	yourDebugDraw();
}

//animation is called for every image on the screen once
void animate()
{
	MyCameraPosition = getCameraPosition();
	glutPostRedisplay();
}



void display(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

/**
* Main Programme
*/
int main(int argc, char** argv)
{
	glutInit(&argc, argv);

	//framebuffer setup
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	// positioning and size of window
	glutInitWindowPosition(200, 100);
	glutInitWindowSize(WindowSize_X, WindowSize_Y);
	glutCreateWindow(argv[0]);

	//initialize viewpoint
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, 0, -4);
	tbInitTransform();     // This is for the trackball, please ignore
	tbHelp();             // idem
	MyCameraPosition = getCameraPosition();

	//activate the light following the camera
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	int LightPos[4] = { 0,0,2,0 };
	int MatSpec[4] = { 1,1,1,1 };
	glLightiv(GL_LIGHT0, GL_POSITION, LightPos);

	//normals will be normalized in the graphics pipeline
	glEnable(GL_NORMALIZE);
	//clear color of the background is black.
	glClearColor(0.0, 0.0, 0.0, 0.0);


	// Activate rendering modes
	//activate depth test
	glEnable(GL_DEPTH_TEST);
	//draw front-facing triangles filled
	//and back-facing triangles as wires
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_LINE);
	//interpolate vertex colors over the triangles
	glShadeModel(GL_SMOOTH);


	// glut setup... to ignore
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutDisplayFunc(display);
	glutMouseFunc(tbMouseFunc);    // trackball
	glutMotionFunc(tbMotionFunc);  // uses mouse
	glutIdleFunc(animate);


	init();

	cout << "Detected and utilising " << THREADS << " threads" << endl;

	//main loop for glut... this just runs your application
	glutMainLoop();

	return 0;  // execution never reaches this point
}











/**
* OpenGL setup - functions do not need to be changed!
* you can SKIP AHEAD TO THE KEYBOARD FUNCTION
*/
//what to do before drawing an image
void display(void)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);//store GL state
									 // Effacer tout
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear image

	glLoadIdentity();

	tbVisuTransform(); // init trackball

	drawFrame();    //actually draw

	glutSwapBuffers();//glut internal switch
	glPopAttrib();//return to old GL state
}
//Window changes size
void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho (-1.1, 1.1, -1.1,1.1, -1000.0, 1000.0);
	gluPerspective(50, (float)w / h, 0.01, 10);
	glMatrixMode(GL_MODELVIEW);
}


//transform the xloc, yloc position on the screen into the corresponding 3D world position
void produceRay(int x_I, int y_I, Vec3Df * origin, Vec3Df * dest)
{
	int viewport[4];
	double modelview[16];
	double projection[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //recuperer matrices
	glGetDoublev(GL_PROJECTION_MATRIX, projection); //recuperer matrices
	glGetIntegerv(GL_VIEWPORT, viewport);//viewport
	int y_new = viewport[3] - y_I;

	double x, y, z;

	gluUnProject(x_I, y_new, 0, modelview, projection, viewport, &x, &y, &z);
	origin->p[0] = float(x);
	origin->p[1] = float(y);
	origin->p[2] = float(z);
	gluUnProject(x_I, y_new, 1, modelview, projection, viewport, &x, &y, &z);
	dest->p[0] = float(x);
	dest->p[1] = float(y);
	dest->p[2] = float(z);
}

// X loop
void yxLoop(unsigned int WindowSize_X, unsigned int yStart, unsigned int yEnd, Vec3Df origin, Vec3Df& origin00, Vec3Df& origin01, Vec3Df &origin10, Vec3Df &origin11, Vec3Df dest, Vec3Df &dest00, Vec3Df &dest01, Vec3Df &dest10, Vec3Df &dest11, Image &result)
{
	//printf("At: %d\n", y);
	for (unsigned int y = yStart; y < yEnd; ++y)
	{
		for (unsigned int x = 0; x < WindowSize_X; ++x)
		{
			//produce the rays for each pixel, by interpolating 
			//the four rays of the frustum corners.
			float xscale = 1.0f - float(x) / (WindowSize_X - 1);

			float yscale = 1.0f - float(y) / (WindowSize_Y - 1);
			origin = yscale*(xscale*origin00 + (1 - xscale)*origin10) +
				(1 - yscale)*(xscale*origin01 + (1 - xscale)*origin11);
			dest = yscale*(xscale*dest00 + (1 - xscale)*dest10) +
				(1 - yscale)*(xscale*dest01 + (1 - xscale)*dest11);

			//launch raytracing for the given ray.
			Vec3Df rgb = performRayTracing(origin, dest);
			//store the result in an image 
			result.setPixel(x, y, RGBValue(rgb[0], rgb[1], rgb[2]));
		}
	}
}

// Helper method for for_each, wait for thread to complete.
void thread_join(std::thread& thread)
{
	thread.join();
}

int numberInput() {
	int number;
	std::cin >> number;

	// Check if the number is a valid unsigned integer
	while ((number < 0) || ((int) number > 255))
	{
		cout << "Invalid number. Please enter again" << endl;
	}
	return number;
}

Vec3Df inputColor() {
	cout << "Please input a color. " << endl;
	cout << "First enter a number between 0 and 255 for the red component of the color: " << endl;
	float red = (float) numberInput() / 255.f;
	cout << "First enter a number between 0 and 255 for the green component of the color: " << endl;
	float green = (float) numberInput() / 255.f;
	cout << "First enter a number between 0 and 255 for the red component of the color: " << endl;
	float blue = (float) numberInput() / 255.f;

	return Vec3Df(red, green, blue);
}

void addSphereLight(float radius,  int numberOfLightPoints, Vec3Df color, Vec3Df &lightposition) {
	for (int i = 0; i < numberOfLightPoints; i++) {
		int randx = rand() % 200;
		int randy = rand() % 200;
		int randz = rand() % 200;
		Vec3Df randDirection = Vec3Df(100 - randx, 100 - randy, 100 - randz);
		randDirection.normalize();

		Vec3Df finalLightPosition = lightposition + radius * randDirection;

		MyLightPositions.push_back(finalLightPosition);
		MyLightColors.push_back(color);
	}
}

// react to keyboard input
void keyboard(unsigned char key, int xloc, int yloc)
{
	printf("key %d pressed at %d,%d\n", key, xloc, yloc);
	fflush(stdout);
	switch (key)
	{
		//add/update a light based on the camera position.
	case 'L':
		MyLightPositions.push_back(getCameraPosition());
		MyLightColors.push_back(inputColor());
		break;
	case 'l':
		MyLightPositions[MyLightPositions.size() - 1] = getCameraPosition();
		MyLightColors[MyLightColors.size() - 1] = inputColor();
		break;
	case 'W':
		MyLightPositions.push_back(getCameraPosition());
		MyLightColors.push_back(Vec3Df(1.f,1.f,1.f));
		break;
	case 'w':
		MyLightPositions[MyLightPositions.size() - 1] = getCameraPosition();
		MyLightColors[MyLightColors.size() - 1] = Vec3Df(1.f, 1.f, 1.f);
		break;
	case 's':
		addSphereLight(RADIUS_OF_LIGHT_SPHERE, NUMBER_OF_LIGHTS_PER_SPHERE, Vec3Df(1, 1, 1), getCameraPosition());
		break;
	case 'm':
		DRAW_NORMALS = !DRAW_NORMALS;
		break;
	case ',':
		DRAW_RAYS_TO_LIGHT = !DRAW_RAYS_TO_LIGHT;
		break;
	case 'r':
	{
		//Pressing r will launch the raytracing.
		cout << "Raytracing" << endl;


		//Setup an image with the size of the current image.
		Image result(WindowSize_X, WindowSize_Y);

		//produce the rays for each pixel, by first computing
		//the rays for the corners of the frustum.
		Vec3Df origin00, dest00;
		Vec3Df origin01, dest01;
		Vec3Df origin10, dest10;
		Vec3Df origin11, dest11;
		Vec3Df origin, dest;


		produceRay(0, 0, &origin00, &dest00);
		produceRay(0, WindowSize_Y - 1, &origin01, &dest01);
		produceRay(WindowSize_X - 1, 0, &origin10, &dest10);
		produceRay(WindowSize_X - 1, WindowSize_Y - 1, &origin11, &dest11);

		clock_t first;
		double total;
		first = clock();
		unsigned int n = THREADS;
		std::vector<std::thread> threads;
		threads.resize(n);

		std::vector<unsigned int> boundaries;
		boundaries.resize(n + 1);

		for (int i = 0; i <= n; ++i)
		{
			boundaries[i] = floor((float)WindowSize_Y * i / (float)n + 0.5f);
		}

		for (int i = 0; i < n; ++i)
		{
			threads[i] = std::thread(yxLoop, WindowSize_X, boundaries[i], boundaries[i + 1], origin, std::ref(origin00), std::ref(origin01), std::ref(origin10), std::ref(origin11), dest, std::ref(dest00), std::ref(dest01), std::ref(dest10), std::ref(dest11), std::ref(result));
		}

		for_each(threads.begin(), threads.end(), thread_join);

		total = (clock() - first) / (double)CLOCKS_PER_SEC;
		double hours = total/3600;
		int hoursInt = std::floor(hours);
		double minutes = ((hours - hoursInt) * 60);
		int minutesInt = std::floor(minutes);
		double seconds = ((minutes - minutesInt) * 60);
		int secondsInt = std::round(seconds);
		printf("Time spent: %.1i hours and %.1i minutes and %.1i seconds\n", hoursInt, minutesInt, secondsInt);

		result.writeImage("result.ppm");
		break;
	}
	case 27:     // touche ESC
		exit(0);
	default:
		break;
	}


	//produce the ray for the current mouse position
	Vec3Df testRayOrigin, testRayDestination;
	produceRay(xloc, yloc, &testRayOrigin, &testRayDestination);

	yourKeyboardFunc(key, xloc, yloc, testRayOrigin, testRayDestination);
}