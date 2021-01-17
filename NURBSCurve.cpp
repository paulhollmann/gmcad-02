#include "NURBSCurve.h"

#include <stdio.h>		// cout
#include <iostream>		// cout

NURBSCurve::NURBSCurve()
{
	
	isValidNURBS();
}

// constructor which takes given control points P, knot vector U and degree p
NURBSCurve::NURBSCurve(const std::vector<Vec4f>& controlPoints_, const std::vector<float>& knotVector_, const unsigned int degree_)
	: controlPoints(controlPoints_)
	, knotVector(knotVector_)
	, degree(degree_)
{
	isValidNURBS();
}

bool NURBSCurve::isValidNURBS()
{
	// knot vector verification
	bool validU = true;
	for (unsigned int i = 1; i < knotVector.size() && validU == true; i++) if (knotVector[i] < knotVector[i-1])
	{
		std::cout << "INVALID (unsorted) knotVector.\n";
		validU = false;
		break;
	}
	// size verification
	bool validSize = true;
	if (controlPoints.size() + degree + 1 != knotVector.size()) 
	{
		std::cout << "INVALID sizes (controlPoints.size() + degree + 1 != knotVector.size()).\n";
		validSize = false;
	}
	return (validU && validSize);
}


Vec4f NURBSCurve::insertKnot(const float newKnot)
{
	unsigned int k = 0; // index of the first knot vector element smaller than newKnot
	unsigned int s = 0; // number of knot vector elements at same position
	float alpha;
	const size_t p = degree-1;

	while (newKnot > knotVector.at(k))
	{
		k += 1;
	}
	while (newKnot == knotVector.at(k + s)) {
		s += 1;
	}
	k -= 1;

	//std::cout<< "ubar" << newKnot << " at " << k << " -> " << knotVector.at(k) << std::endl;

	std::vector<Vec4f>(Q); //the new control points
	Q.reserve(controlPoints.size() + 1);

	for (unsigned int i = 0; i <= k - p + s; i++) Q.push_back( controlPoints.at(i));
	for (unsigned int i = k - p + 1 + s; i <= k + s; i++) 
	{
		float alpha = calculateAlpha(newKnot, p, i, k);
		Q.push_back(alpha * controlPoints.at(i) + (1 - alpha) * controlPoints.at(i-1));
	}
	for (unsigned int i = k + 1 + s; i < controlPoints.size() + 1; i++) Q.push_back(controlPoints.at(i-1));


	std::vector<float>(U);
	U.reserve(knotVector.size() + 1);

	for (unsigned int i = 0; i <= k; i++) U.push_back(knotVector.at(i));
	U.push_back(newKnot);
	for (unsigned int i = k + 1; i < knotVector.size(); i++) U.push_back(knotVector.at(i));



	// all done save the new points and knot vector

	controlPoints = Q;
	knotVector = U;

	// =====================================================
	if (k + s  >= Q.size())
		return Q.back();
	return Q.at(k+s);

}

float NURBSCurve::calculateAlpha(const float ubar, const int p, const int i, const int k) {

	return (ubar - knotVector.at(i)) / (knotVector.at(i + p) - knotVector.at(i));
}

/*
float NURBSCurve::calculateAlpha(const float newKnot, int p, int r, int j, int k) {
	return (newKnot - knotVector.at(j + k - p)) / (knotVector.at(j + 1 + k - r) - knotVector.at(j + k - p));
}*/

Vec4f NURBSCurve::evaluteDeBoor(const float t, Vec4f& tangent)
{
	// create a copy of this NURBS curve
	NURBSCurve tempNURBS(*this);
	Vec4f point;
	// TODO: use insertKnot to evaluate the curve and its tangent. Take care to NOT modify this NURBS curve. Instead use the temporary copy.
	// =====================================================================================================================================

	// insert a point the degree
	if (t == 1) return tempNURBS.controlPoints.back();
	if (t == 0) return tempNURBS.controlPoints.front();

	for (unsigned int i = 0; i < tempNURBS.getDegree(); i++)
		point = tempNURBS.insertKnot(t);

	//std::cout << tempNURBS;
	//point = Vec4f(0, 0, 0, 1);
	// =====================================================================================================================================
	return point;
}


std::pair<std::vector<Vec4f>, std::vector<Vec4f>> NURBSCurve::evaluateCurve(const size_t numberSamples)
{
	int _numberSamples = numberSamples;
	if (numberSamples < 2) _numberSamples = 2;
	float steps = 1.0f / ((float)_numberSamples - 1.0f);
	std::vector<Vec4f> points;
	points.reserve(numberSamples);
	std::vector<Vec4f> tangents;
	tangents.reserve(numberSamples);
	// TODO: implement evaluation of the NURBS curve at 'numberSamples' equidistant points
	// Note: use the evaluteDeBoor(t) function. 
	// ==========================================================================================================

	for (int i = 0; i < _numberSamples-1; i++) {
		float t = i * steps;
		Vec4f tangent;
		points.push_back(evaluteDeBoor(t, tangent));
		tangents.push_back(tangent);
	}
	
	// ==========================================================================================================

	return std::make_pair(points,tangents);
}

std::ostream& operator<< (std::ostream& os, NURBSCurve& nurbs)
{
	// degree
	os << "NURBS curve, degree " << nurbs.getDegree() << "\n";
	// control points
	os << "  " << nurbs.getControlPoints().size() << " controlPoints:\n";
	for (unsigned int i = 0; i < nurbs.getControlPoints().size(); i++) os << "    " << nurbs.getControlPoints()[i] << "\n";
	// knot vector
	os << "  " << nurbs.getKnotVector().size() << " knotVector: ";
	for (unsigned int i = 0; i < nurbs.getKnotVector().size(); i++) os << nurbs.getKnotVector()[i] << ", ";
	os << "\n";
	// knot vector verification
	nurbs.isValidNURBS();
	return os;
}