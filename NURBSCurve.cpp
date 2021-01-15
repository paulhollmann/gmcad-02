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
	int k = 0;
	int s = 0;
	float alpha;
	const size_t p = degree + 1;

	while (newKnot >= knotVector.at(k))
	{
		k += 1;
		if (newKnot == knotVector.at(k)) {
			s += 1;
		}
	}
	k -= 1;

	//std::cout << "t " << newKnot << std::endl;
	std::vector<Vec4f> tempContolPoints{};
	tempContolPoints.reserve(p);
	for (int j = 0; j <= degree; j++) {
		tempContolPoints.push_back(controlPoints.at(j + k - degree));
	}
	
	// one for loop to much??
	for (int r = 1; r <= degree; r++) {
		for (int j = degree; j >=r ; j--) {
			alpha = calculateAlpha(newKnot, degree, r, j, k);
			Vec4f p1 = tempContolPoints.at(j);
			Vec4f p2 = tempContolPoints.at(j - 1.0f);

			Vec4f vec1 = p1 * alpha;
			Vec4f vec2 = p2 * (1.0f - alpha);

			Vec4f res = vec1 + vec2;
			tempContolPoints.at(j) = res;
		}
	}

	// all done save the new points and knot vector

	//controlPoints = tempContolPoints;
	//knotVector = ...

	// =====================================================
	
	return tempContolPoints.at(degree);

}

float NURBSCurve::calculateAlpha(const float newKnot, int p, int r, int j, int k) {
	return (newKnot - knotVector.at(j + k - p)) / (knotVector.at(j + 1 + k - r) - knotVector.at(j + k - p));
}

Vec4f NURBSCurve::evaluteDeBoor(const float t, Vec4f& tangent)
{
	// create a copy of this NURBS curve
	NURBSCurve tempNURBS(*this);
	Vec4f point;
	// TODO: use insertKnot to evaluate the curve and its tangent. Take care to NOT modify this NURBS curve. Instead use the temporary copy.
	// =====================================================================================================================================
	point = tempNURBS.insertKnot(t);

	
	// =====================================================================================================================================
	return point;
}


std::pair<std::vector<Vec4f>, std::vector<Vec4f>> NURBSCurve::evaluateCurve(const size_t numberSamples)
{
	int _numberSamples = numberSamples;
	if (numberSamples < 2) _numberSamples = 2;
	float steps = 1.0f /  ((float) _numberSamples - 1.0f);
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