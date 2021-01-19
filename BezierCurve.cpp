#include "BezierCurve.h"

#include <stdio.h>		// cout
#include <iostream>		// cout

BezierCurve::BezierCurve(bool isRational_)
	: rational(isRational_)
{
}

BezierCurve::BezierCurve(const std::vector<Vec3f>& controlPoints_, bool isRational_)
	: controlPoints(controlPoints_)
	, rational(isRational_)
{
}
// splits the bezier curve into two at u=t
std::pair<BezierCurve, BezierCurve> BezierCurve::separateCurveAt(const float t)
{
	std::vector<Vec3f> cps1, cps2;
	
	// TODO: implement the de Casteljau algorithm and create two separated bezier curves with the new control points.
	// ==========================================================================================================
	size_t n = this->controlPoints.size();
	std::vector<Vec3f> currentCps = this->controlPoints;
	for (int i = 0; i < n; i++) {
		cps1.push_back(currentCps.front());
		cps2.push_back(currentCps.back());
		std::vector<Vec3f> bufferCps;
		for (size_t j = 0; j < currentCps.size()-1; j++)
		{
			//std::cout << (1 - t) * currentCps.at(j) + t * currentCps.at(j + 1) << std::endl;
			bufferCps.push_back((1 - t) * currentCps.at(j) + t * currentCps.at(j + 1));
		}
		currentCps = bufferCps;
	}
	// ==========================================================================================================
	BezierCurve curve1(cps1, rational);
	std::reverse(cps2.begin(), cps2.end());
	BezierCurve curve2(cps2, rational);
	//std::cout << "split a:" << cps1;
	//std::cout << "split b:" << cps2;
	return std::pair<BezierCurve, BezierCurve>(curve1, curve2);
}
// evaluates the bezier curve at u=t, returns that point and calculates its tangent
Vec3f BezierCurve::evaluateCurveAt(const float t, Vec3f &tangent)
{
	Vec3f point;
	// TODO: implement the evaluation of the bezier curve and the tangent at t.
	// Note: use the seperateCurveAt(t) function.
	// ==========================================================================================================
	//std::cout << "base: " << base << std::endl;
	std::pair<BezierCurve, BezierCurve> split = separateCurveAt(t);
	BezierCurve bezierCurveA = split.first;
	BezierCurve bezierCurveB = split.second;

	if (! this->isRational()) {
		Vec3f pointDiff = bezierCurveA.getControlPoints().back() - bezierCurveB.getControlPoints().front();
		Vec3f tangentA = bezierCurveA.getControlPoints().back() - bezierCurveA.getControlPoints().at(bezierCurveA.getControlPoints().size() - 2);
		Vec3f tangentB = bezierCurveB.getControlPoints().front() - bezierCurveB.getControlPoints().at(1);
		Vec3f tangentDiff = tangentA - tangentB;
		if (pointDiff.length() < 0.0001f)
			point = bezierCurveA.getControlPoints().back();

		if (tangentDiff.length() < 0.0001f)
			tangent = tangentA;
	} else {
		Vec3f point = bezierCurveA.getControlPoints().back();
		Vec3f pointL = bezierCurveA.getControlPoints().at(bezierCurveA.getControlPoints().size() - 2);

		Vec3f tangentL = point / point.z - pointL / pointL.z;
		tangentL.z = point.z / pointL.z;
		tangent = tangentL;
	}




	// ==========================================================================================================
	return point;
}

std::pair<std::vector<Vec3f>, std::vector<Vec3f>> BezierCurve::evaluateCurve(const size_t numberSamples)
{
	int _numberSamples = numberSamples;
	if (numberSamples < 2) _numberSamples = 2;
	std::vector<Vec3f> points;
	points.reserve(_numberSamples);
	std::vector<Vec3f> tangents;
	tangents.reserve(_numberSamples);
	// TODO: implement evaluation of the bezier curve at 'numberSamples' equidistant points
	// Note: use the 'evaluateCurveAt' function. 
	// ==========================================================================================================
	for (int i = 0; i < _numberSamples; i++) {
		float t =  i / (_numberSamples - 1.0f);
		Vec3f tangent;
		points.push_back(evaluateCurveAt(t, tangent));
		tangents.push_back(tangent);
	}
	// ==========================================================================================================
	return std::make_pair(points,tangents);
}

std::ostream& operator<< (std::ostream& os, BezierCurve& bezierCurve)
{
	if(bezierCurve.controlPoints.empty())
	{
		os << "Bezier curve without control points!";
		return os;
	}
	// degree
	os << "Bezier curve, degree " << bezierCurve.controlPoints.size() - 1 << "\n";
	// control points
	for(size_t i = 0; i < bezierCurve.controlPoints.size(); i++) os << "  " << bezierCurve.controlPoints[i] << std::endl;
	return os;
}