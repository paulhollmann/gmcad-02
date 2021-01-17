// ========================================================================= //
// Authors: Felix Schuwirth                                                  //
// mailto:felix.schuwirth@igd.fraunhofer.de                                  //
//                                                                           //
// GRIS - Graphisch Interaktive Systeme                                      //
// Technische Universit�t Darmstadt                                          //
// Fraunhoferstrasse 5                                                       //
// D-64283 Darmstadt, Germany                                                //
//                                                                           //
// Creation Date: 06.12.2016                                                 //
// Content: Simple and extendable openGL program offering                    //
//   * basic navigation and basic pipeline rendering                         //
// ========================================================================= //

#include "main.h"		// this header

#include <stdlib.h>		// standard library
#include <cmath>		// fmod
#include <stdio.h>		// cout
#include <iostream>		// cout

#include <CurveRendering.h>


// ==============
// === BASICS ===
// ==============

int main(int argc, char** argv)
{
	// initialize openGL window
	glutInit(&argc, argv);
	glutInitWindowPosition(300, 200);
	glutInitWindowSize(600, 400);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("TU Darmstadt, GMCAD");
	// link functions to certain openGL events
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(mousePressed);
	glutMotionFunc(mouseMoved);
	glutKeyboardFunc(keyPressed);
	// further initializations
	setDefaults();
	createCurves();
	initializeGL();
	// activate main loop
	glutMainLoop();
	return 0;
}

void setDefaults()
{
	// scene Information
	transX = -1.0f;
	transY = -1.0f;
	transZ = -4.0f;
	angleX = 0.0f;
	angleY = 0.0f;
	// mouse information
	mouseX = 0;
	mouseY = 0;
	mouseButton = 0;
	mouseSensitivy = 1.0f;
	evalParameter = 0.5f;
}

void initializeGL()
{
	// black screen
	glClearColor(0, 0, 0, 0);
	// enable depth buffer
	glEnable(GL_DEPTH_TEST);
	// Use Point Smoothing
	glEnable(GL_POINT_SMOOTH);
	glPointSize(5.0f);
	glLineWidth(2.0f);
	// set shading model
	glShadeModel(GL_SMOOTH);
	// key bindings => cout
	coutHelp();
}

void createCurves()
{
	activeBezier = 0;
	bezierCurves.clear();
	// TODO: create at least one bezier and one degree 2 quarter circle rational bezier curve
	// ==========================================================================

	std::vector<Vec3f> pts;

	// einfaches Beispiel Bezierkurve
	pts.push_back(Vec3f(1.0f, 1.0f, 1.0f));
	pts.push_back(Vec3f(2.0f, 2.0f, 2.0f));
	pts.push_back(Vec3f(3.0f, 1.0f, 1.0f));
	pts.push_back(Vec3f(3.0f, 0.0f, 2.0f));
	bezierCurves.push_back(BezierCurve(pts, false));
	pts.clear();

	// Parabel quadratische Bezierkurve
	pts.push_back(Vec3f(1.0f, 0.0f, 0.0f));
	pts.push_back(Vec3f(0.0f, 0.0f, 0.0f));
	pts.push_back(Vec3f(0.0f, 1.0f, 0.0f));
	bezierCurves.push_back(BezierCurve(pts, false));
	pts.clear();

	// Halbkreis rationale Bezierkurve
	pts.push_back(Vec3f(1.0f, 0.0f, 1.0f));
	pts.push_back(Vec3f(1.0f, 1.0f, 1.0f));
	pts.push_back(Vec3f(0.0f, 2.0f, 2.0f));
	bezierCurves.push_back(BezierCurve(pts, true));
	pts.clear();

	// Viertelkreis 2. Grad rationale Bezierkurve
	pts.push_back(Vec3f(1.0f, 0.0f, 1.0f));
	pts.push_back(Vec3f(0.0f, 1.0f, 0.0f));
	pts.push_back(Vec3f(-1.0f, 0.0f, 1.0f));
	bezierCurves.push_back(BezierCurve(pts, true));
	pts.clear();

	// Ellipse rationale Bezierkurve
	pts.push_back(Vec3f(1.0f, 0.0f, 1.0f));
	pts.push_back(Vec3f(0.0f, 0.0f, 0.5f));
	pts.push_back(Vec3f(0.0f, 1.0f, 1.0f));
	bezierCurves.push_back(BezierCurve(pts, true));
	pts.clear();

	// Hyperbel rationale Bezierkurve
	pts.push_back(Vec3f(1.0f, 0.0f, 1.0f));
	pts.push_back(Vec3f(0.0f, 0.0f, 2.0f));
	pts.push_back(Vec3f(0.0f, 1.0f, 1.0f));
	bezierCurves.push_back(BezierCurve(pts, true));
	pts.clear();

	bezierCurves.clear(); // !!!!


	// ==========================================================================
	for (auto &b : bezierCurves)
		std::cout << b << std::endl;
	
	nurbsCurves.clear();
	// TODO: set values to describe a degree 2 quarter circle in first quadrant, XY-plane
	// ==================================================================================
	std::vector<Vec4f> npts;
	npts.push_back(Vec4f(0.0f, 0.0f, 0.0f, 1.70f));
	npts.push_back(Vec4f(0.0f, 1.0f, 1.0f, 1.0f));
	npts.push_back(Vec4f(0.0f, 2.0f, 1.0f, 1.0f));
	npts.push_back(Vec4f(0.0f, 3.0f, 0.0f, 1.20f));
	npts.push_back(Vec4f(0.0f, 2.0f, -1.0f, 1.0f));
	npts.push_back(Vec4f(0.0f, 1.0f, -1.0f, 1.0f));
	npts.push_back(Vec4f(0.0f, 0.0f, 0.0f, 1.90f));
		
	
	
	nurbsCurves.push_back(NURBSCurve(npts, std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0}, 3));
	//nurbsCurves.push_back(NURBSCurve(npts, std::vector<float>{0.0,0.0,0.0,0.0,0.1,0.6241,0.9,1.0,1.0,1.0,1.0},3));


	nurbsCurves.clear(); // !!!!!!!
	npts.clear(); // !!!!!!!


	npts.push_back(Vec4f(0.0f, 0.0f, 0.0f, 1.0f));
	npts.push_back(Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
	npts.push_back(Vec4f(2.0f, 0.0f, 0.0f, 1.0f));
	npts.push_back(Vec4f(3.0f, 3.0f, 0.0f, 1.0f));

	NURBSCurve nurbs1 = NURBSCurve(npts, std::vector<float>{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}, 2);
	std::cout << nurbs1 << std::endl;
	nurbsCurves.push_back(nurbs1);

	npts.clear();

	npts.push_back(Vec4f(0.0f, 0.0f, 4.0f, 1.0f));
	npts.push_back(Vec4f(1.0f, 1.0f, 4.0f, 1.0f));
	npts.push_back(Vec4f(2.0f, 0.0f, 4.0f, 1.0f));
	npts.push_back(Vec4f(3.0f, 3.0f, 4.0f, 1.0f));
	NURBSCurve nurbs2 = NURBSCurve(npts, std::vector<float>{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}, 2);
	nurbs2.insertKnot(0.75);
	nurbs2.insertKnot(0.75);
	nurbsCurves.push_back(nurbs2);

	nurbsCurves.clear(); // !!!!!!!
	npts.clear(); // !!!!!!!

	npts.push_back(Vec4f(0.0f, 0.0f, 0.0f, 2.0f));
	npts.push_back(Vec4f(1.0f, 1.0f, 0.0f, 3.0f));
	npts.push_back(Vec4f(2.0f, 0.0f, 0.0f, 4.0f));
	npts.push_back(Vec4f(3.0f, 3.0f, 0.0f, 2.0f));

	nurbs1 = NURBSCurve(npts, std::vector<float>{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}, 2);
	std::cout << nurbs1 << std::endl;
	nurbsCurves.push_back(nurbs1);

	npts.clear();

	npts.push_back(Vec4f(0.0f, 0.0f, 4.0f, 2.0f));
	npts.push_back(Vec4f(1.0f, 1.0f, 4.0f, 3.0f));
	npts.push_back(Vec4f(2.0f, 0.0f, 4.0f, 4.0f));
	npts.push_back(Vec4f(3.0f, 3.0f, 4.0f, 2.0f));

	nurbs2 = NURBSCurve(npts, std::vector<float>{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0}, 2);
	nurbs2.insertKnot(0.75);
	nurbs2.insertKnot(0.75);
	nurbsCurves.push_back(nurbs2);


	std::cout << nurbs2 << std::endl;

	// ==================================================================================
	for (auto &n : nurbsCurves)
		std::cout << n << std::endl;
}

void reshape(GLint width, GLint height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (float)width / (float)height, 0.1f, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// =================
// === RENDERING ===
// =================

void drawCS()
{
	glBegin(GL_LINES);
	// red X
	glColor3f(0.8f, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0.5, 0, 0);
	// green Y
	glColor3f(0, 0.8f, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.5, 0);
	// blue Z
	glColor3f(0, 0, 0.8f);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 0.5);
	glEnd();
}

void drawObjects()
{
	for(unsigned int i = 0; i < bezierCurves.size(); ++i)
	{
		renderBezier(bezierCurves[i]);
		if(i == activeBezier)
			renderBezierEvaluation(bezierCurves[i], evalParameter);
	}
	for(unsigned int i = 0; i < nurbsCurves.size(); ++i)
	{
		renderNURBS(nurbsCurves[i]);
		if(i == activeNURBS)
			renderNURBSEvaluation(nurbsCurves[i], evalParameter);
	}
}

void renderScene()
{
	// clear and set camera
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	// translate scene in viewing direction
	glTranslatef(transX, transY, transZ);
	// rotate scene
	glRotatef(angleX, 0.0f, 1.0f, 0.0f);
	glRotatef(angleY, 1.0f, 0.0f, 0.0f);
	// draw coordinate system without lighting
	glDisable(GL_LIGHTING);
	drawCS();
	drawObjects();
	// swap Buffers
	glFlush();
	glutSwapBuffers();
}

// =================
// === CALLBACKS ===
// =================

void keyPressed(unsigned char key, int x, int y)
{
	switch(key)
	{
	// esc => exit
	case 27:
		exit(0);
		break;
	// help file
	case 'h' :
	case 'H' :
		coutHelp();
		break;
	// reset view
	case 'r' :
	case 'R' :
		setDefaults();
		glutPostRedisplay();	// use this whenever 3d data changed to redraw the scene
		break;
	// TODO: place custom functions on button events here to present your results
	// like changing the active Bbezier/NURBS curve (activeNURBS, activeBezier)
	// and varying the evaluation parameter (evalParameter) for the bezier curve
	// ==========================================================================
	case 'n' :
	case 'N':
		selectNextCurve();
		glutPostRedisplay();
		break;
	case 'p':
	case 'P':
		selectPrevCurve();
		glutPostRedisplay();
		break;

	case '>':
		shiftEvalRight();
		glutPostRedisplay();
		break;
	case '<':
		shiftEvalLeft();
		glutPostRedisplay();
		break;

	// ==========================================================================
	}
}

void mousePressed(int button, int state, int x, int y)
{
	mouseButton = button;
	mouseX = x;
	mouseY = y;
}

void mouseMoved(int x, int y)
{
	// rotate (cap angleY within [-85�, +85�])
	if(mouseButton == GLUT_LEFT_BUTTON)
	{
		angleX = fmod(angleX + (x - mouseX) * mouseSensitivy, 360.0f);
		angleY += (y - mouseY) * mouseSensitivy;
		if(angleY > 85) angleY = 85;
		if(angleY < -85) angleY = -85;
		glutPostRedisplay();
	}
	// zoom (here translation in z)
	if(mouseButton == GLUT_RIGHT_BUTTON)
	{
		transZ -= 0.2f * (y - mouseY) * mouseSensitivy;
		glutPostRedisplay();
	}
	// translation in xy
	if(mouseButton == GLUT_MIDDLE_BUTTON)
	{
		transX += 0.2f * (x - mouseX) * mouseSensitivy;
		transY -= 0.2f * (y - mouseY) * mouseSensitivy;
		glutPostRedisplay();
	}
	// update mouse for next relative movement
	mouseX = x;
	mouseY = y;
}

// ===============
// === VARIOUS ===
// ===============

void coutHelp()
{
	std::cout << std::endl;
	std::cout << "====== KEY BINDINGS ======" << std::endl;
	std::cout << "ESC: exit" << std::endl;
	std::cout << "H: show this (H)elp file" << std::endl;
	std::cout << "R: (R)eset view" << std::endl;
	// TODO: update help text according to your changes
	// ================================================


	// ================================================
	std::cout << "==========================" << std::endl;
	std::cout << std::endl;
}

void selectNextCurve()
{
	activeBezier++;
	activeNURBS++;
	if (activeBezier == bezierCurves.size()) 
	{
		activeBezier--;
		activeNURBS--;
	}
}



void selectPrevCurve()
{
	if (activeBezier > 0)
	{
		activeBezier--;
		activeNURBS--;
	}
}

void shiftEvalRight()
{
	evalParameter += 0.1f;
	if(evalParameter > 1) evalParameter = 1.0f;
}

void shiftEvalLeft()
{
	evalParameter -= 0.1f;
	if (evalParameter < 0) evalParameter = 0.0f;
}
