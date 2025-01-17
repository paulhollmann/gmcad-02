#define RADPERDEG 0.0174533

#include "CurveRendering.h"

#include <GL/glut.h>
#include <BezierCurve.h>
#include <NURBSCurve.h>
#include <algorithm>
#include <iostream>	

#define EVALNR 50


void drawBezier(BezierCurve &bezierCurve, Vec3f color)
{
	// TODO: implement the visualization of the 3D bezier curve (e.g. with GL_LINE_STRIP)
	// ===============================================================================
	std::pair<std::vector<Vec3f>, std::vector<Vec3f>> samples = bezierCurve.evaluateCurve(50);
	std::vector<Vec3f> points = samples.first;
	std::vector<Vec3f> tangents = samples.second;
	
	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		glVertex3f(points.at(i).x, points.at(i).y, points.at(i).z);
	}
	glEnd();
	
	// ===============================================================================
}
void drawBezierCtrlPolygon(const BezierCurve &bezierCurve, Vec3f color)
{
	// TODO: implement the visualization of the 3D bezier curves control polygon (e.g. with GL_LINE_STRIP)
	// ===============================================================================
	// cps of the complete curve

	std::vector<Vec3f> points = bezierCurve.getControlPoints();
	glColor3f(color.x, color.y, color.z);
	
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		glVertex3f(points.at(i).x, points.at(i).y, points.at(i).z);
	}

	glEnd();

	// ===============================================================================
}
void drawRationalBezier(BezierCurve &bezierCurve, Vec3f color)
{
	if (bezierCurve.isRational())
	{
		// TODO: implement the visualization of the 2D rational bezier curve in the plane w=1 (e.g. with GL_LINE_STRIP)
		// ===============================================================================
		std::pair<std::vector<Vec3f>, std::vector<Vec3f>> samples = bezierCurve.evaluateCurve(EVALNR);
		std::vector<Vec3f> points = samples.first;
		std::vector<Vec3f> tangents = samples.second;

		glColor3f(color.x, color.y, color.z);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < points.size(); i++)
		{
			Vec3f point = points.at(i);
			point /= point.z;
			glVertex3f(point.x, point.y, point.z);
		}
		glEnd();

		// ===============================================================================
	}
}
void drawRationalBezierCtrlPolygon(const BezierCurve &bezierCurve, Vec3f color)
{
	if (bezierCurve.isRational())
	{
		// TODO: implement the visualization of the 2D rational bezier curves control polygon in the plane w=1 (e.g. with GL_LINE_STRIP)
		// ===============================================================================
		// cps of the complete curve
		std::vector<Vec3f> points = bezierCurve.getControlPoints();
		glColor3f(color.x, color.y, color.z);

		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < points.size(); i++)
		{
			Vec3f point = points.at(i);
			point /= point.z;
			glVertex3f(point.x, point.y, point.z);
		}

		glEnd();

		// ===============================================================================
	}
}

void drawNURBS(NURBSCurve &nurbsCurve, Vec3f color)
{
	// TODO: draw NURBS curve
	// NOT homogenized
	// ===================================================================================
	std::pair<std::vector<Vec4f>, std::vector<Vec4f>> samples = nurbsCurve.evaluateCurve(EVALNR);
	std::vector<Vec4f> points = samples.first;
	std::vector<Vec4f> tangents = samples.second;

	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		glVertex3f(points.at(i).x, points.at(i).y, points.at(i).z);
	}
	glEnd();

	// ===================================================================================
}
void drawNURBS_H(NURBSCurve &nurbsCurve, Vec3f color)
{
	// TODO: draw NURBS curve
	// homogenized
	// ===================================================================================
	std::pair<std::vector<Vec4f>, std::vector<Vec4f>> samples = nurbsCurve.evaluateCurve(EVALNR);
	std::vector<Vec4f> points = samples.first;
	std::vector<Vec4f> tangents = samples.second;

	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		Vec4f point = points.at(i);
		point /= point.w;
		glVertex3f(point.x, point.y, point.z);
	}
	glEnd();

	// ===================================================================================
}
void drawNURBSCtrlPolygon(const NURBSCurve &nurbsCurve, Vec3f color)
{
	// TODO: implement the visualization of the 
	// NOT homogenized 
	// NURBS' control polygon (e.g. with GL_LINE_STRIP)
	// =========================================================================================================
	std::vector<Vec4f> points = nurbsCurve.getControlPoints();
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		Vec4f point = points.at(i);
		glVertex3f(point.x, point.y, point.z);
	}

	glEnd();

	// =========================================================================================================
}
void drawNURBSCtrlPolygon_H(const NURBSCurve &nurbsCurve, Vec3f color)
{
	// TODO: implement the visualization of the 
	// homogenized 
	// NURBS' control polygon (e.g. with GL_LINE_STRIP)
	// =========================================================================================================
	std::vector<Vec4f> points = nurbsCurve.getControlPoints();
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < points.size(); i++)
	{
		Vec4f point = points.at(i);
		point /= point.w;
		glVertex3f(point.x, point.y, point.z);
	}

	glEnd();

	// =========================================================================================================
}

void renderBezier(BezierCurve &bezierCurve)
{
	//auto pointsAndTangents = bezierCurve.evaluateCurve(size_t(EVALNR));
	bool rational = bezierCurve.isRational();
	Vec3f color = Vec3f(0.0f, 1.0f, 1.0f);
	if (bezierCurve.isRational())
	{
		drawRationalBezier(bezierCurve, color);
	}
	if (rational)
		color = Vec3f(1.0f, 0.0f, 1.0f);
	drawBezier(bezierCurve, color);
}
void renderBezierEvaluation(BezierCurve &bezierCurve, float t)
{
	Vec3f color = { 1.0f,1.0f,1.0f };
	if (bezierCurve.getControlPoints().size() > 1)
	{
		bool rational = bezierCurve.isRational();
		auto twoBezierCurves = bezierCurve.separateCurveAt(t);

		glLineWidth(2.0f);

		// first partial curve cps
		if (rational)color = Vec3f(0.8f, 5.0f, 0.4f);
		else		color = Vec3f(0.8f, 0.0f, 0.4f);
		drawBezierCtrlPolygon(twoBezierCurves.first, color);
		// second partial curve cps		
		if (rational)color = Vec3f(0.4f, 5.0f, 0.8f);
		else		color = Vec3f(0.4f, 0.0f, 0.8f);
		drawBezierCtrlPolygon(twoBezierCurves.second, color);

		glLineWidth(1.0f);
		// cps of the complete curve
		if (rational)color = Vec3f(0.3f, 0.2f, 0.7f);
		else		color = Vec3f(0.3f, 0.7f, 0.7f);
		drawBezierCtrlPolygon(bezierCurve, color);

		glLineWidth(2.0f);
		if (bezierCurve.isRational())
		{

			auto twoBezierCurves2 = bezierCurve.separateCurveAt(t);
			// first partial curve cps
			color = Vec3f(0.8f, 0.0f, 0.4f);
			drawRationalBezierCtrlPolygon(twoBezierCurves2.first, color);
			// second partial curve cps
			color = Vec3f(0.4f, 0.0f, 0.8f);
			drawRationalBezierCtrlPolygon(twoBezierCurves2.second, color);
			glLineWidth(1.0f);
			// cps of the complete curve
			color = Vec3f(0.3f, 0.7f, 0.7f);
			drawRationalBezierCtrlPolygon(bezierCurve, color);

		}
	}

}

void renderNURBS(NURBSCurve &nurbsCurve)
{
	if (nurbsCurve.getControlPoints().size() > 1 && nurbsCurve.isValidNURBS())
	{
		// draw curve (x, y, w) not homogenized
		Vec3f color = Vec3f(1.0f, 0.8f, 0.0f);
		drawNURBS(nurbsCurve, color);
		color = Vec3f(0.8f, 1.0f, 0.0f);
		drawNURBS_H(nurbsCurve, color);
	}
}
void renderNURBSEvaluation(NURBSCurve &nurbsCurve, float t)
{
	Vec3f color = Vec3f(1.0f, 1.0f, 1.0f);
	if (!nurbsCurve.isValidNURBS())
		return;

	auto pointsAndTangents = nurbsCurve.evaluateCurve(size_t(EVALNR));
	auto points = pointsAndTangents.first;
	auto tangents = pointsAndTangents.second;

	if (points.size() > 1 && nurbsCurve.getControlPoints().size() > 1)
	{

		// draw control polygon (x, y, w) not homogenized
		color = Vec3f(0.7f, 0.5f, 0.0f);
		drawNURBSCtrlPolygon(nurbsCurve, color);
		// draw control polygon (x/w, y/w, w=1) homogenized
		color = Vec3f(0.7f, 0.0f, 0.7f);
		drawNURBSCtrlPolygon_H(nurbsCurve, color);

		// draw tangents of the not homogenized curve
		glColor3f(0.5f, 0.35f, 0.0f);
		glBegin(GL_LINES);
		for (unsigned int i = 0; i < tangents.size(); ++i)
		{
			auto p = points[i];
			auto t = tangents[i];
			t = Vec4f(t.w * p.x + p.w * t.x, t.w * p.y + p.w * t.y, t.w * p.z + p.w * t.z, 1.0f);
			glVertex3fv(&p.x);
			t = (t - p).normalized() + p;
			glVertex3fv(&t.x);
		}
		glEnd();
		// draw tangents of the homogenized curve
		glColor3f(0.5f, 0.0f, 0.5f);
		glBegin(GL_LINES);
		for (unsigned int i = 0; i < tangents.size(); ++i)
		{
			auto p = points[i];
			auto t = tangents[i];
			p = p / p.w;
			t = t / t.w;
			t = p + t;
			glVertex3fv(&p.x);
			t = (t - p).normalized() + p;
			glVertex3fv(&t.x);
		}
		glEnd();
		// draw the current evalPoint
		Vec4f tang;
		Vec4f point = nurbsCurve.evaluteDeBoor(t, tang);
		{
			glPushMatrix();
			glTranslated(point.x, point.y, point.z);
			GLUquadricObj* quadObj;
			quadObj = gluNewQuadric();
			gluQuadricDrawStyle(quadObj, GLU_FILL);
			gluQuadricNormals(quadObj, GLU_SMOOTH);
			gluSphere(quadObj, 0.02f, 16, 16);
			gluDeleteQuadric(quadObj);
			glPopMatrix();
		}
		{
			glPushMatrix();
			point /= point.w;
			glTranslated(point.x, point.y, point.z);
			GLUquadricObj* quadObj;
			quadObj = gluNewQuadric();
			gluQuadricDrawStyle(quadObj, GLU_FILL);
			gluQuadricNormals(quadObj, GLU_SMOOTH);
			gluSphere(quadObj, 0.02f, 16, 16);
			gluDeleteQuadric(quadObj);
			glPopMatrix();
		}
	}
}
