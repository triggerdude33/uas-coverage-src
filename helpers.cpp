#ifndef HELPERS_CPP
#define HELPERS_CPP

#include "helpers.hpp"

/**
* Helper method which checks if the NFZ zone and points stored in vectors are different
* from the Feature enumerator inputs. If they are, the old data is removed and the new
* data is copied in.
*
* @pointsIn the points enumerator gotten from input
* @nfzIn the No fly zones enumerator gotten from input
* @return true if any data differs, otherwise false
*/
bool inputModified(std::vector<FeaturePtr>& points, std::vector<FeaturePtr>& noFlyZones, FeatureEnumeratorPtr pointsIn, FeatureEnumeratorPtr nfzIn)
{
	// Check for noFlyZone mutation
	bool mutatedNfz = false;
	std::vector<Carmenta::Engine::FeaturePtr> noFlyZonesNew;
	while (nfzIn->moveNext())
		noFlyZonesNew.push_back(nfzIn->current());

	// unequal size
	if (noFlyZonesNew.size() != noFlyZones.size())
		mutatedNfz = true;

	// unequal content
	for (int i = 0; i < std::min<int>(noFlyZonesNew.size(), noFlyZones.size()); i++)
		if (!polygonEquality(noFlyZones[i], noFlyZonesNew[i]))
			mutatedNfz = true;

	// update content if mutated
	if (mutatedNfz)
	{
		noFlyZones.clear();
		for (int i = 0; i < noFlyZonesNew.size(); i++)
		{
			FeaturePtr nfzFeat = dynamic_cast<Feature*>(noFlyZonesNew[i]->clone().get());
			noFlyZones.push_back(nfzFeat);
		}
			
	}

	// check for Aoi point mutation
	bool mutatedPoints = false;

	std::vector<Carmenta::Engine::FeaturePtr> pointsNew;
	while (pointsIn->moveNext())
		pointsNew.push_back(pointsIn->current());

	// unequal size
	if (pointsNew.size() != points.size())
		mutatedPoints = true;

	// unequal content
	for (int i = 0; i < std::min<int>(pointsNew.size(), points.size()); i++)
		if (!pointEquality(points[i], pointsNew[i], 0))
			mutatedPoints = true;

	// update content if mutated
	if (mutatedPoints)
	{
		points.clear();
		for (int i = 0; i < pointsNew.size(); i++)
			points.push_back(pointsNew[i]);
	}

	return mutatedNfz || mutatedPoints;
}

/**
* Helper method. Computes intersection of two pitch and yaw intervals.
* If it returns true, the answer vector has been mutated to the intersection
* of the intervals. Using answer vector when function has returned false
* results in undefined behavior.
*
* @a vector storing pitch&yaw interval 1
* @b vector storing pitch&yaw interval 2
* @answer used to output intersection
* @return true if intersection exists, otherwise false
*/
bool pitchYawIntervalJoiner(const std::vector<int>& a, const std::vector<int>& b, std::vector<int>& answer)
{
	if (answer.size() < 4)
		throw EngineException("Error in pitchYawIntervalJoiner. OutputVector has too small size");

	// Join pitch
	answer[0] = std::max<double>(a[0], b[0]);
	answer[1] = std::min<double>(a[1], b[1]);
	if (answer[0] > answer[1])
		return false;

	// join yaw

	// edge case. Only happens for waypoints who don't have an AOiPoint assigned yet
	if ((a[2] == YAW_MIN && a[3] == YAW_MAX) || (b[2] == YAW_MIN && b[3] == YAW_MAX))
	{
		answer[2] = a[2] == YAW_MIN ? b[2] : a[2];
		answer[3] = a[3] == YAW_MAX ? b[3] : a[3];
		return true;
	}

	if ((a[2] > a[3] && b[2] <= b[3]) && (b[2] < 270 && b[3] < 270))
	{
		answer[2] = b[2];
		answer[3] = std::min<int>(a[3], b[3]);
	}
	else if ((b[2] > b[3] && a[2] <= a[3]) && (a[2] < 270 && a[3] < 270))
	{
		answer[2] = a[2];
		answer[3] = std::min<int>(a[3], b[3]);
	}
	else if ((a[3] < a[2] && b[3] >= b[2]) && (b[2] > 90 && b[3] > 90))
	{
		answer[2] = std::max<int>(a[2], b[2]);
		answer[3] = b[3];
	}
	else if ((b[3] < b[2] && a[3] >= a[2]) && (a[2] > 90 && a[3] > 90))
	{
		answer[2] = std::max<int>(a[2], b[2]);
		answer[3] = a[3];
	}
	else {
		answer[2] = std::max<int>(a[2], b[2]);
		answer[3] = std::min<int>(a[3], b[3]);
	}

	if ((answer[2] > answer[3]) && !(a[2] > a[3] && b[2] > b[3]))
		return false;

	// pitch and yaw joined, return true
	return true;
}

/**
* Helper method. Searches for which pixel in a volume encoding raster
* contains a given point. Assumes that the raster data ascends in x-direction
* and starts at lowest x and y value.
*
* @vsc ViewShedCell. The volume encoding raster to search in
* @p the point we want to find inside the
* @return true if the point is covered by the raster. Otherwise false
*/
bool isPointInVer(const FeaturePtr vsc, const Point p)
{
	RasterGeometryPtr geom = dynamic_cast<RasterGeometry*>(vsc->geometry().get());
	Carmenta::Engine::Rectangle bound = geom->bounds();

	if (bound.xMin() > p.x() || bound.xMax() < p.x() ||
		bound.yMin() > p.y() || bound.yMax() < p.y())
	{
		return false;
	}


	Point crsRatio =
		vsc->crs()->metersXY(Point(bound.xMin(), bound.yMin()));

	// get the elevation data as pixels (short)
	int viewshedCellSideLen = (int)(crsRatio.x() * std::abs(bound.xMax() - bound.xMin()));
	int numPixels =
		(viewshedCellSideLen * PIXEL_SIZE) * (viewshedCellSideLen * PIXEL_SIZE);
	int pixelRow = viewshedCellSideLen * PIXEL_SIZE;

	// pixel offsets
	int pXOff = int(std::round(crsRatio.x() * (p.x() - bound.xMin()) * PIXEL_SIZE));
	int pYOff = int(std::round(crsRatio.y() * (p.y() - bound.yMin()) * PIXEL_SIZE));

	const unsigned char* data = geom->raster()->readOnlyData();

	// multiply by two two times because a short datatype is two bytes
	// and each pixel contains two shorts (high and low elev value)
	short lowZ = *(data + ((pYOff * pixelRow + pXOff) * 2) * 2);
	short highZ = *(data + ((pYOff * pixelRow + pXOff) * 2 + 1) * 2);

	if (lowZ == 0 && highZ == 0)
		return false;
	else if (lowZ > p.z() || p.z() > highZ)
		return false;
	else
		return true;
}

/**
* Helper method. Searches for which volume encoding raster in a vector that
* contains a given point. Assumes for ver that the vector is ordered
* in descending raster X-cell order starting from highest cell X and Y cell coordinates.
*
* @ver The volume encoding raster vector to search in
* @p the point we want to find inside the raster vector
* @return The index in ver for the viewshedCell that contains p. -1 if p outside of vsc
*/
int findPointInVerV(const std::vector<FeaturePtr>& ver, const Point p)
{
	if (int(std::sqrt(ver.size())) - std::sqrt(ver.size()) != 0.0)
		throw EngineException("Error in losWaypointOperator::findPointInVer. Ver vector does not have square size");

	int vscRow = int(std::sqrt(ver.size()));
	int lx = 0, mx = 0, rx = vscRow - 1, ly = 0, my = 0, ry = vscRow - 1;
	bool foundX = false, foundY = false;
	std::vector<FeaturePtr>::const_iterator it = ver.begin();

	// Search for the correct viewshedCell using 2D binary search
	while ((lx <= rx && ly <= ry) && (!(foundX && foundY)))
	{
		if (!foundX) mx = int((lx + rx) / 2);
		if (!foundY) my = int((ly + ry) / 2);
		FeaturePtr vsc = (it + (int)((vscRow - 1 - my) * vscRow + (vscRow - 1 - mx)))->get();
		Carmenta::Engine::Rectangle bound = dynamic_cast<RasterGeometry*>(vsc->geometry().get())->bounds();

		if (bound.xMax() < p.x())
			lx = mx + 1;
		else if (bound.xMin() > p.x())
			rx = mx - 1;
		else
			foundX = true;

		if (bound.yMax() < p.y())
			ly = my + 1;
		else if (bound.yMin() > p.y())
			ry = my - 1;
		else
			foundY = true;
	}

	if (foundX && foundY)
		return (int)((vscRow - 1 - my) * vscRow + (vscRow - 1 - mx));

	return -1;
}

/**
* Helper method. Determines if a given point has line of sight to any other point in
* a vector of points. Vector of polygons are also given which obstruct the line of sight.
* This function does calculations in 2D. All z-values are ignored. Each fog polygon
* is assumed to only have one ring.
* 
* @pf the point feature to check if it has LOS to any other point
* @observers the point vector that pf will check LOS for all points
* @fog vector of polygon features which obstruct LOS
* @return true if pf has LOS of at least one point. Otherwise false.
*/
bool pointLOSObserversFog(const FeaturePtr& pf, 
	const std::vector<FeaturePtr> observers, const std::vector<FeaturePtr> fog)
{
	Point p1 = dynamic_cast<PointGeometry*>(pf->geometry().get())->point();
	// for each observer in vector
	for (std::vector<FeaturePtr>::const_iterator oIt = observers.begin(); oIt < observers.end(); oIt++)
	{
		Point p2 = dynamic_cast<PointGeometry*>(oIt->get()->geometry().get())->point();
		PointCollectionPtr pc = new PointCollection();
		pc->add(p1); pc->add(p2);
		LineGeometryPtr losLine = new LineGeometry(pc);

		bool intersected = false;

		// for each fog polygon in vector
		for (std::vector<FeaturePtr>::const_iterator fIt = fog.begin(); fIt < fog.end(); fIt++)
		{
			LineGeometryPtr fogLine =
				dynamic_cast<PolygonGeometry*>(
						fIt->get()->geometry().get())->rings()->get(0);

			PointCollectionPtr intersections = losLine->lineIntersections(fogLine);

			if (intersections->size() != 0)
			{
				intersected = true;
				break;
			}
		}

		if (!intersected)
			return true;
	}

	return false;
}

/**
* Helper method. Determines if a given point collides with any of the
* polygon obstacles in a given vector. All Z-coordinates are
* ignored, so operates on 2D geometries.
*
* @pf point feature. The point to check collisions for
* @obs The vector of obstacles that the feature can collide with
* @margin the margin around the point which must not touch any polygon
* @return true if we detect collision with a polygon. Otherwise false
*/
bool pointCollide(const FeaturePtr& pf, const std::vector<FeaturePtr>& obs, double margin)
{
	Point p = dynamic_cast<PointGeometry*>(pf->geometry().get())->point();

	for (std::vector<FeaturePtr>::const_iterator it = obs.begin(); it < obs.end(); it++)
	{
		
		PolygonGeometryPtr poly = dynamic_cast<PolygonGeometry*>(dynamic_cast<Feature*>(it->get()->clone().get())->geometry().get());
		MultiPolygonGeometryPtr polyBufColl = poly->bufferZone(margin, 4);
		PolygonGeometryPtr polyBuf = polyBufColl->polygons().get()->begin()->get();

		if (polyBuf->pointInside(p))
			return true;
	}
	return false;
}

/*
  rounds a given value to a certain decimal precision
  If precision is for example 0.1, then the number 1.167 will be 
  rounded to 1.2

  @value the value to round to
  @precision the precision to round to
  @return the rounded value
*/
double roundTo(double value, double precision)
{
	return std::round(value * precision) / precision;
}

/**
* Helper method. Determines if a given Rectangle collides with
* any of the polygon obstacles in a given vector. Returns different output
* depending on if the feature partly collides, or is fully inside an obstacle
*
* @r rectangle. What we want to check collisions for
* @obs obstacles. The vector of obstacles that the feature can collide with
* @return 0: no collision; 1: partly collides (one or more); 2: fully inside (one or more)
*/
int rectangleCollidePolygon(const Carmenta::Engine::Rectangle r, const std::vector<FeaturePtr> obs)
{
	int collisionType = 0;

	// create a std::vector of all bounding points for rectangle
	std::vector<Point> rfVec;

	rfVec.push_back(Point(r.xMin(), r.yMin()));
	rfVec.push_back(Point(r.xMin(), r.yMax()));
	rfVec.push_back(Point(r.xMax(), r.yMax()));
	rfVec.push_back(Point(r.xMax(), r.yMin()));


	// for each obstacle
	for (std::vector<FeaturePtr>::const_iterator oIt = obs.begin(); oIt < obs.end(); oIt++)
	{
		PolygonGeometryPtr obsGeom = dynamic_cast<PolygonGeometry*>(oIt->get()->geometry().get());
		int collisionCounter = 0;

		// check center points
		if (obsGeom->pointInside(r.center()))
			collisionType = 1;
		if (r.inside(obsGeom->center()))
			collisionType = 1;

		// for each point in rectangle
		for (std::vector<Point>::iterator pIt = rfVec.begin(); pIt < rfVec.end(); pIt++)
		{
			if (obsGeom->pointInside(*pIt))
			{
				collisionType = 1;
				collisionCounter++;
			}
		}


		if (collisionCounter == rfVec.size())
		{
			collisionType = 2;
			break;
		}
	}

	return collisionType;
}

/**
* Helper method to get the pitch and yaw bounds for an observer to
* have a specified target point within line of sight.
*
* @obs Position of the observer
* @target The point which the observer needs to have within line of sight
* @c the crs to calculate pitch and yaw
* @output The vector reference to store pitch&yaw bounds
*/
void getPitchAndYawBounds(const Point obs, const Point target, const CrsPtr& c, std::vector<int>& output)
{
	if (output.size() < 4)
		throw EngineException("Error in getPitchAndYawBounds. OutputVector has too small size");

	double yaw = c->azimuthFromLineSegment(obs, target, LineTypeGreatCircle);
	double pitch = c->pitchFromLineSegment(obs, target, LineTypeGreatCircle);

	output[0] = std::max<double>(int(pitch - verticalFOV(UAS_FOV_H, ASPECT_RATIO_H, ASPECT_RATIO_V) / 2), PITCH_MIN);
	output[1] = std::min<double>(int(pitch + verticalFOV(UAS_FOV_H, ASPECT_RATIO_H, ASPECT_RATIO_V) / 2), PITCH_MAX);

	int yawMin = int(yaw - UAS_FOV_H / 2.0);
	output[2] = yawMin < 0 ? 360 + yawMin : yawMin;
	output[3] = int(yaw + UAS_FOV_H / 2.0) % (YAW_MAX + 1);
	return;

}

/**
* Helper method. Determines the euclidean distance (in meters) between
* two points. Takes in coordinate reference system for the points.
*
* @p1 the first point
* @p2 the second point
* @c the crs of the points
* @return The euclidean distance between p1 and p2
*/
double pointDist(const Point& p1, const Point& p2, const CrsPtr& c)
{
	Point r = c->metersXY(p1);
	double dx = std::abs((p1.x() * p1.x() - p2.x() * p2.x()) * r.x());
	double dy = std::abs((p1.y() * p1.y() - p2.y() * p2.y()) * r.y());
	double dz = std::abs(p1.z() * p1.z() - p2.z() * p2.z());
	double d = std::sqrt(dx + dy + dz);

	return d;
}

/*
* Checks if two polygons have the same coordinates.
* @a feature 1. Assumes has polygonGeometry
* @b feature2. Assmumes has polygonGeometry
* @return true if the polygons have identical coordinates, otherwise false
*/
bool polygonEquality(FeaturePtr a, FeaturePtr b)
{
	PolygonGeometryPtr ag = dynamic_cast<PolygonGeometry*>(a->geometry().get());
	PolygonGeometryPtr bg = dynamic_cast<PolygonGeometry*>(b->geometry().get());

	if (ag->center() != bg->center())
		return false;
	else if (ag->area() != bg->area())
		return false;

	return true;
}

/*
* Checks if two points have the same coordinates. Round value of 0 results in no rounding
* being conducted. Rounding value of 10 results in the 0.1 decimal being kept.
* @a feature 1. Assumes has PointGeometry
* @b feature 2. Assumes has PointGeometry
* @round amount of decimals to round the comparison to.
* @return true if the points have identical coordinates, otherwise false
*/
bool pointEquality(FeaturePtr a, FeaturePtr b, double round)
{
	Point pa = dynamic_cast<PointGeometry*>(a->geometry().get())->point();
	Point pb = dynamic_cast<PointGeometry*>(b->geometry().get())->point();

	if (round != 0)
	{
		Point paRound = Point((int(pa.x() * round)) / round, (int(pa.y() * round)) / round, (int(pa.z() * round)) / round);
		Point pbRound = Point((int(pb.x() * round)) / round, (int(pb.y() * round)) / round, (int(pb.z() * round)) / round);

		if (!(paRound.x() - pbRound.x() < 1/round && paRound.y() - pbRound.y() < 1/round && paRound.z() - pbRound.z() < 1/round))
			return false;
	}
	else
	{
		if (pa != pb)
			return false;
	}

	return true;
}

/**
* Helper method. shuffles a referenced std::vector. Mutates input.
* uses merseene twister to generate randomness.
*
* @v the vector to shuffle
* @rndEngine random number engine
*/
void shuffle(std::vector<int>& v, std::mt19937& rndEngine)
{
	std::vector<int> tmp;
	uint16_t size = v.size();
	std::uniform_real_distribution<> distrib(0.0, 1.0);
	for (uint32_t i = 0; i < size; i++)
	{
		int index = (int)(distrib(rndEngine) * v.size());
		tmp.push_back(v[index]);
		v.erase(v.begin() + index);
	}

	v = std::move(tmp);
}

/*
	Calculates the vertical field of view of a camera, 
	from its horisontal field of view and aspect ratio

	@fovH the horisontal field of view, in degrees
	@aspectRatioH The horisontal number of the aspect ratio
	@aspectRatioV The vertical number of the aspect ratio
	@return the vertical field of view
*/
double verticalFOV(double fovH, int aspectRatioH, int aspectRatioV)
{
	double fovV = 2 * atan(tan(fovH / 2 * DEG_TO_RAD) / aspectRatioH * aspectRatioV) / DEG_TO_RAD;
	return fovV;
}

#endif