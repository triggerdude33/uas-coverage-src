#ifndef BENCHMARK_MODULE_CPP
#define BENCHMARK_MODULE_CPP

#include "benchmarkModule.hpp"

/**
	@author Matteus Berg
	@date 2025-01-16
*/

benchmarkModule::benchmarkModule(
	SplitOperatorPtr losWpIn,
	SplitOperatorPtr tourWpIn,
	SplitOperatorPtr routeIn,
	string benchmarkDirOut) :
	losWpIn(losWpIn), tourWpIn(tourWpIn), routeIn(routeIn), benchmarkDirOut(benchmarkDirOut), 
	waypointCalcTime(0), tourCalcTime(0), routeCalcTime(0), numWaypoints(0), 
	avgWpEdgeDist(0), avgWpRouteDist(0), routeDist(0), simNum(0), ratioNonFoundRoutes(0)
{}

void benchmarkModule::benchmark(int seedNum){
	this->simNum = seedNum+1;
	int benchmarkNum = 1;

	// calculate number of waypoints and distances
	updateWaypointsRoutes();
	updateMetrics();

	// create output string
	string output = "";
	output.append(std::to_string(benchmarkNum));
	output.append(",");
	output.append(std::to_string(simNum));
	output.append(",");
	output.append(std::to_string(waypointCalcTime.count()));
	output.append(",");
	output.append(std::to_string(tourCalcTime.count()));
	output.append(",");
	output.append(std::to_string(routeCalcTime.count()));
	output.append(",");
	output.append(std::to_string(numWaypoints));
	output.append(",");
	output.append(std::to_string(avgWpEdgeDist));
	output.append(",");
	output.append(std::to_string(avgWpRouteDist));
	output.append(",");
	output.append(std::to_string(varWpEdgeDist));
	output.append(",");
	output.append(std::to_string(varWpRouteDist));
	output.append(",");
	output.append(std::to_string(routeDist));
	output.append(",");
	output.append(std::to_string(ratioNonFoundRoutes));
	output.append("\n");

	// write output string to file
	std::ofstream(benchmarkDirOut, std::ios_base::app) << output;
}

void benchmarkModule::setWaypointCalcTime(milliseconds time)
{
	this->waypointCalcTime = time;
}

void benchmarkModule::setTourCalcTime(milliseconds time)
{
	this->tourCalcTime = time;
}

void benchmarkModule::setRouteCalcTime(milliseconds time)
{
	this->routeCalcTime = time;
}

/** 
	fetches all features from waypoints and routes read operators anew and
	updates their respective vector member variables
*/
void benchmarkModule::updateWaypointsRoutes()
{
	FeatureEnumeratorPtr newLosPoints = losWpIn->getFeatures(Crs::wgs84LongLat());
	losPoints.clear();
	while (newLosPoints->moveNext())
		losPoints.push_back(newLosPoints->current());

	FeatureEnumeratorPtr newTourPoints = tourWpIn->getFeatures(Crs::wgs84LongLat());
	tourPoints.clear();
	while (newTourPoints->moveNext())
		tourPoints.push_back(newTourPoints->current());

	FeatureEnumeratorPtr newRoutes = routeIn->getFeatures(Crs::wgs84LongLat());
	routes.clear();
	while (newRoutes->moveNext())
		routes.push_back(newRoutes->current());

	return;
}

/**
	This function updates member variables. 
	number of waypoints, average edge distance in tour, average route distance in tour (between waypoints),
	total tour distance.
*/
void benchmarkModule::updateMetrics() 
{
	// update amount of waypoints
	this->numWaypoints = losPoints.size();

	// update the average edge distance in the tour
	vector<double> edgeDist = vector<double>(0);
	double edgeDistSum = 0;
	for (int i = 0; i < int(tourPoints.size()); i++)
	{
		int startIndex = i;
		int endIndex = i + 1 < tourPoints.size() ? i + 1 : 0;

		Point startPoint = dynamic_cast<PointGeometry*>(tourPoints[startIndex]->geometry().get())->point();
		Point endPoint = dynamic_cast<PointGeometry*>(tourPoints[endIndex]->geometry().get())->point();
		Point ratio = tourPoints[startIndex]->crs()->metersXY(startPoint);
		double distance = sqrt(
			(pow(ratio.x() * (startPoint.x() - endPoint.x()), 2.0)) +
			(pow(ratio.y() * (startPoint.y() - endPoint.y()), 2.0)) +
			pow(startPoint.z() - startPoint.z(), 2.0)
		);
		edgeDist.push_back(distance);
		edgeDistSum += distance;
	}
	this->avgWpEdgeDist = edgeDistSum / tourPoints.size();

	// update the variance edge distance in the tour
	double squareDeltaSumEdge = 0;
	for (vector<double>::iterator it = edgeDist.begin(); it < edgeDist.end(); it++)
		squareDeltaSumEdge += pow(*it - this->avgWpEdgeDist, 2);
	this->varWpEdgeDist = sqrt(squareDeltaSumEdge / (edgeDist.size() - 1));

	// update the average route distance in the tour
	vector<double> routeDist = vector<double>(0);
	double routeDistSum = 0;
	for (int i = 0; i < int(routes.size()); i++)
	{
		double dist = 0;
		if (!routes[i]->attributes()->get(LEG_DISTANCE_NAME).tryGetValue(dist))
			throw EngineException("Error in benchmarkModule::updateMetrics. Leg distance attribute for leg feature not found");
		routeDist.push_back(dist);
		routeDistSum += dist;
	}
	this->avgWpRouteDist = routeDistSum / routes.size();
	
	// update the variance route distance in the tour
	double squareDeltaSumRoute = 0;
	for (vector<double>::iterator it = routeDist.begin(); it < routeDist.end(); it++)
		squareDeltaSumRoute += pow(*it - this->avgWpRouteDist, 2);
	this->varWpRouteDist = sqrt(squareDeltaSumRoute / (routeDist.size() - 1));

	// update the total tour distance
	this->routeDist = routeDistSum;

	// update the ratio of found routes
	double totalRoutes = routes.size();
	int nonFoundRoutes = 0;

	for (int i = 0; i < int(routes.size()); i++)
	{
		bool foundRoute = 0;
		if (!routes[i]->attributes()->get(ROUTE_FOUND_NAME).tryGetValue(foundRoute))
			throw EngineException("Error in benchmarkModule::updateMetrics. Leg distance attribute for leg feature not found");
		nonFoundRoutes += !foundRoute;
	}

	this->ratioNonFoundRoutes = (nonFoundRoutes / totalRoutes) * 100;
}

#endif