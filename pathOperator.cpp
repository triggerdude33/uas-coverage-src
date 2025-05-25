#ifndef PATH_OPERATOR_CPP
#define PATH_OPERATOR_CPP

#include "pathOperator.hpp"

/*
@author Matteus Berg
@date 2024-12-28
*/

/**
	paths and pathsDistances are assumed to have been initalized beforehand
*/
pathOperator::pathOperator(
	std::string& baseUrl, 
	SplitOperatorPtr wpIn, 
	SplitOperatorPtr nfzIn,
	ReadOperatorPtr legsOut) :
	baseUrl(baseUrl), wpIn(wpIn), nfzIn(nfzIn), legsOut(legsOut)
{
	if (baseUrl.empty())
		throw EngineException("Error in pathOperator. Base URL cannot be empty");
}

bool pathOperator::featureUpdate()
{
	FeatureEnumeratorPtr waypointFeatures = wpIn->getFeatures(Crs::wgs84LongLat());
	FeatureEnumeratorPtr nfzFeatures = nfzIn->getFeatures(Crs::wgs84LongLat());

	if (inputModified(this->waypoints, this->noFlyZones, waypointFeatures, nfzFeatures))
	{
		// Create new NFZ JSON body for all future leg calculations
		nfzBody.clear();
		for (std::vector<FeaturePtr>::iterator it = noFlyZones.begin(); it < noFlyZones.end(); it++)
		{
			PointCollectionPtr pc;
			PolygonGeometryPtr pg;
			try
			{
				pg = dynamic_cast<PolygonGeometry*>((*it)->geometry().get());
			}
			catch (EngineException e)
			{
				throw EngineException("Error in pathOperator::featureUpdate. NoFlyZone does not contain valid polygon geometry");
			}

			// make NFZ a little smaller than in actuallity. So that HADO grid does not
			// mistakenly classify waypoints as unreachable
			MultiPolygonGeometryPtr polyBufColl = pg->bufferZone(-GEOFENCE_BUFFER_ROUTING, 4);
			PolygonGeometryPtr polyBuf = polyBufColl->polygons().get()->begin()->get();
			pc = polyBuf->rings()->begin()->get()->points();

			nlohmann::json nfzJson;
			nfzJson["bottomAltitude"] = ZONE_BOTTOM_ALTITUDE;
			nfzJson["topAltitude"] = ZONE_TOP_ALTITUDE;

			for (unsigned int i = 0; i < pc->size(); i++)
			{
				Point p = pc->get(i);
				nfzJson["polygon"]["points"].push_back({ {"x", p.x()}, {"y", p.y()}, {"z", 0} });
			}

			nfzBody.push_back(nfzJson);
		}

		// calculate new paths
		computePaths();

		return true;
	}
	else
		return false;
}

void pathOperator::computePaths()
{
	nlohmann::json requestBody =
	{
		{"crs", 0},
		{"legs",
			{
			
			}
		},
	    { "parameters", 
			{
				{"minVerticalClearance", 0},
				{"maxVerticalClearance", 120}, // regulation for how far you can fly a drone above the terrain in the EU
				{"geofenceManeuver", 0}, 
				{"axisManeuver", 0}
			} 
		}
	};
	requestBody["geofences"] = nfzBody;

	// compute paths from one waypoint to the next
	for (int i = 0; i < int(waypoints.size()); i++)
	{
		int startIndex = i;
		int endIndex = i + 1 < waypoints.size() ? i + 1 : 0;

		if(DEBUG_WP_ID)
		{
			Carmenta::Engine::AttributeValue wpId1;
			if (!waypoints[startIndex]->attributes()->tryGetValue(WAYPOINT_NAME, wpId1))
				throw Carmenta::Engine::EngineException("Error in pathOperator::computePaths. No Id for input waypoint found");
			int wpIdVal1;
			if (!wpId1.tryGetValue(wpIdVal1))
				throw Carmenta::Engine::EngineException("Error in pathOperator::computePaths. Waypoint Id is in non-integer format");

			Carmenta::Engine::AttributeValue wpId2;
			if (!waypoints[endIndex]->attributes()->tryGetValue(WAYPOINT_NAME, wpId2))
				throw Carmenta::Engine::EngineException("Error in pathOperator::computePaths. No Id for input waypoint found");
			int wpIdVal2;
			if (!wpId2.tryGetValue(wpIdVal2))
				throw Carmenta::Engine::EngineException("Error in pathOperator::computePaths. Waypoint Id is in non-integer format");
		}

		Point p1 = dynamic_cast<PointGeometry*>(waypoints[startIndex]->geometry().get())->point();
		Point p2 = dynamic_cast<PointGeometry*>(waypoints[endIndex]->geometry().get())->point();

		requestBody["legs"].push_back(
			{
				{"start", {
					{"point", {
						{"x", p1.x()},
						{"y", p1.y()},
						{"z", p1.z()}

					}}
				}},
				{"end", {
					{"point", {
						{"x", p2.x()},
						{"y", p2.y()},
						{"z", p2.z()}
					}}
				}},
				{"order", i+1}
			}
		);
	}

	nlohmann::json answer;
	MemoryDataSetPtr legsOutDataSet = dynamic_cast<MemoryDataSet*>(this->legsOut->dataSet().get());

	// hail HADO backend and get the legs
	if (!getLegs(requestBody, answer))
		throw Carmenta::Engine::EngineException("Error in pathOperator::computePaths. Routing server was unable to return successful request");

	/* parse and write all fetched legs to output. */

	{
		Guard guard(legsOutDataSet);
		legsOutDataSet->clear();
	}
	int lc = 0;

	// compute paths from one waypoint to the next
	for (int i = 0; i < int(waypoints.size()); i++)
	{
		int startIndex = i;
		int endIndex = i + 1 < waypoints.size() ? i + 1 : 0;

		PointCollectionPtr pc = new PointCollection();
		Point pStart = dynamic_cast<PointGeometry*>(waypoints[startIndex]->geometry().get())->point();
		Point pEnd = dynamic_cast<PointGeometry*>(waypoints[endIndex]->geometry().get())->point();
		bool routeFound = true;
		LineGeometryPtr lg;
		FeaturePtr f;

		// true when not all legs have been returned
		if (lc >= answer["legs"].size())
		{
			pc->add(pStart); pc->add(pEnd);
			routeFound = false;
		}
		else
		{
			nlohmann::json startCoord = answer["legs"][lc]["leg"]["start"]["point"];
			Point pStartJson = Point(startCoord["x"], startCoord["y"], startCoord["z"]);
			PointGeometryPtr pgStartJson = new PointGeometry(pStartJson);
			FeaturePtr fStartJson = new Feature(pgStartJson, Crs::wgs84LongLat());
			nlohmann::json endCoord = answer["legs"][lc]["leg"]["end"]["point"];
			Point pEndJson = Point(endCoord["x"], endCoord["y"], endCoord["z"]);
			PointGeometryPtr pgEndJson = new PointGeometry(pEndJson);
			FeaturePtr fEndJson = new Feature(pgEndJson, Crs::wgs84LongLat());

			// if the returned HADO route is not the expected route, the expected route has not been found by HADO.
			if (!(pointEquality(waypoints[startIndex], fStartJson, POINT_EQUALITY_PRECISION) || pointEquality(waypoints[endIndex], fEndJson, POINT_EQUALITY_PRECISION)))
			{
				pc->add(pStart); pc->add(pEnd);
				routeFound = false;
			}
		}

		// set the legDistance to the bee-line between the two waypoints, if HADO does not return route
		Point startPoint = dynamic_cast<PointGeometry*>(waypoints[startIndex]->geometry().get())->point();
		Point endPoint = dynamic_cast<PointGeometry*>(waypoints[endIndex]->geometry().get())->point();
		Point ratio = waypoints[startIndex]->crs()->metersXY(startPoint);
		double distance = sqrt(
			(pow(ratio.x() * (startPoint.x() - endPoint.x()), 2.0)) +
			(pow(ratio.y() * (startPoint.y() - endPoint.y()), 2.0)) +
			pow(startPoint.z() - startPoint.z(), 2.0)
		);
		AttributeValue legDistance = distance;

		// if HADO returned a valid route for this leg, get its distance and intermediate points
		if (routeFound)
		{	
			legDistance = AttributeValue(((double)answer["legs"][lc]["distance"]));
			pc->add(pStart);
			for (size_t k = 0; k < answer["legs"][lc]["intermediatePoints"].size(); k++)
			{
				nlohmann::json coord = answer["legs"][lc]["intermediatePoints"][k];
				Point p = Point(coord["x"], coord["y"], coord["z"]);
				pc->add(p);
			}
			pc->add(pEnd);
			// increment leg counter, so that we access the next leg in the next iteration
			lc++;
		}

		lg = new LineGeometry(pc);
		f = new Feature(lg, Crs::wgs84LongLat());
		
		// set attributes and insert leg into dataset
		f->attributes()->set(Atom(LEG_DISTANCE_NAME), legDistance);
		f->attributes()->set(Atom(ROUTE_FOUND_NAME), routeFound);
		f->attributes()->set(Atom(LEG_WAYPOINT_1_NAME), waypoints[startIndex]->attributes()->get(WAYPOINT_NAME));
		f->attributes()->set(Atom(LEG_WAYPOINT_2_NAME), waypoints[endIndex]->attributes()->get(WAYPOINT_NAME));
		
		{
			Guard guard(legsOutDataSet);
			legsOutDataSet->insert(f);
		}
	}
}

/**
* Function which hails the HADO rest gateway to fetch routes between all waypoints.
* On sucessful route fetch, mutates the paths and pathsDistance
* member variable to save all of the routes and their distances.
* 
* @body the json body to send to the rest API
* @answer the json body answer from the server. Mutates variable
* @return true if all routes where successfully fetched, otherwise false
*/
bool pathOperator::getLegs(const nlohmann::json& body, nlohmann::json& answer)
{
	std::string jsonRequest = body.dump();

	// send request to server
	answer = post(REST_ROUTING_ENDPOINT, body);

	std::string jsonAnswer = answer.dump();

	// parse answer
	if (answer["status"]["message"] != "Success")
		return false;

	return true;
}

/**
	Function which sends passed JSON body to passed REST endpoint.

	@endpoint the endpoint to send the JSON body to
	@body the JSON body
	@return the body sent back by the server
*/
nlohmann::json pathOperator::post(const std::string& endpoint, const nlohmann::json& body)
{
	httplib::Client client(baseUrl.c_str());
	auto res = client.Post(endpoint.c_str(), body.dump(), "application/json");

	if (!res)
		throw EngineException("Error in pathOperator. Failed to connect to the server");

	if (res->status != 200)
	{
		std::string err = "Error in pathOperator. Server responded with status code: ";
		err.append(std::to_string(res->status));
		throw EngineException(err.c_str());
	}
	
	return nlohmann::json::parse(res->body);
}

#endif