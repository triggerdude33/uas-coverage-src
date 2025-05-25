#ifndef losWaypointOperator_CPP
#define losWaypointOperator_CPP

#include "losWaypointOperator.hpp"

/*
@author Matteus Berg
@date 2024-12-28
*/

using namespace Carmenta::Engine;

losWaypointOperator::losWaypointOperator(
	CrsPtr c,
	SplitOperatorPtr aoiIn,
	SplitOperatorPtr nfzIn,
	ReadOperatorPtr wpOut,
	ReadOperatorPtr targetIn,
	ReadOperatorPtr observerIn,
	SplitOperatorPtr targetLosOut,
	ReadOperatorPtr viewshedIn,
	SplitOperatorPtr viewshedOut,
	ImageDataSetPtr elevationIn) :
	AoiPointsId(-1), ViewshedsId(-1), NfzId(-1),
	waypointIdCounter(0), rndEngine(RND_SEEDS[0]),
	seedCounter(0), AoiCellUpdateCounter(0), ElevDataCoordRef(c),
	redoCalc(false)
{
	this->aoiIn = aoiIn;
	this->nfzIn = nfzIn;
	this->wpOut = wpOut;
	this->targetIn = targetIn;
	this->observerIn = observerIn;
	this->targetLosOut = targetLosOut;
	this->viewshedIn = viewshedIn;
	this->viewshedOut = viewshedOut;
	this->elevationIn = elevationIn;
}

/*
	increments the waypoint generation seed for randomly placing waypoints

	@return true if seed was successfully incremented, otherwise false
*/
bool losWaypointOperator::incrementSeed()
{
	if (seedCounter == NUM_SEEDS - 1)
		return false;
	seedCounter++;
	this->rndEngine.seed(RND_SEEDS[seedCounter]);
	redoCalc = true;
	return true;
}


/*
	@return the seed for randomly placing waypoints
*/
int losWaypointOperator::getSeedNum()
{
	return seedCounter;
}

/*
	decrements the waypoint generation seed for randomly placing waypoints

	@return true if seed was successfully decremented, otherwise false
*/
bool losWaypointOperator::decrementSeed()
{
	if (seedCounter == 0)
		return false;
	seedCounter--;
	this->rndEngine.seed(RND_SEEDS[seedCounter]);
	redoCalc = true;
	return true;
}

/*
* Function which updates the waypoint features in the output, if
* the input data has been mutated.
* 
* @return true if new features have been updated, otherwise false.
*/
bool losWaypointOperator::featureUpdate() 
{
	FeatureEnumeratorPtr aoiFeatures = aoiIn->getFeatures(Crs::wgs84LongLat());
	FeatureEnumeratorPtr nfzFeatures = nfzIn->getFeatures(Crs::wgs84LongLat());

	if (inputModified(this->aoiPoints, this->noFlyZones, aoiFeatures, nfzFeatures) || redoCalc)
	{
		redoCalc = false;
		waypoints.clear();
		waypointsV.clear();
		waypointIdCounter = 0;
		aoiFeatures = aoiIn->getFeatures(Crs::wgs84LongLat());

		computeWaypoints(aoiFeatures);

		MemoryDataSetPtr wpDataSet = dynamic_cast<MemoryDataSet*>(wpOut->dataSet().get());
		wpDataSet->clear();
		for (std::vector<FeaturePtr>::iterator it = waypoints.begin(); it < waypoints.end(); it++)
			wpDataSet->insert(*it);

		return true;
	}
	else
		return false;
}

/**
* For each no fly zone (Nfz), creates a waypoint for each edge point.
* Waypoint is placed a distance away from edge point, perpendicular from Nfz center.
*/
void losWaypointOperator::createNavpoints()
{
	// for each Nfz
	for (std::vector<FeaturePtr>::const_iterator it = noFlyZones.begin(); it < noFlyZones.end(); it++)
	{
		PolygonGeometryPtr polyG = dynamic_cast<PolygonGeometry*>(
			it->get()->geometry().get());
		LineGeometryPtr lineG =
			dynamic_cast<LineGeometry*>(polyG->rings()->begin()->get());
		PointCollectionPtr epc = lineG->points();
		CrsPtr c = it->get()->crs();

		// for each edge point in Nfz
		for (int i = 0; i < epc->size(); i++)
		{
			Point cp = epc->get(i);
			Point ratio = c->metersXY(cp);
			int prevPIndex = i > 0 ? i - 1 : epc->size() - 1;
			int nextPIndex = i < epc->size() - 1 ? i + 1 : 0;

			// get orientation from previous and next point in the line
			int yawPrev = c->azimuthFromLineSegment(cp, epc->get(prevPIndex), LineTypeGreatCircle);
			int yawNext = c->azimuthFromLineSegment(cp, epc->get(nextPIndex), LineTypeGreatCircle);
			
			// two different yaws for the navpoint is possible
			int yawNavP1 = (yawPrev + yawNext) / 2;
			int yawNavP2 = (yawNavP1 + 180) % 360;
			
			// get x, y coordinates for point
			Point navPoint1XY = Point(cp.x() + M_MARGIN_NAVPOINT/ratio.x() * std::sin(DEG_TO_RAD*yawNavP1), cp.y() + M_MARGIN_NAVPOINT/ratio.y() * std::cos(DEG_TO_RAD*yawNavP1));
			Point navPoint2XY = Point(cp.x() + M_MARGIN_NAVPOINT/ratio.x() * std::sin(DEG_TO_RAD*yawNavP2), cp.y() + M_MARGIN_NAVPOINT/ratio.y() * std::cos(DEG_TO_RAD*yawNavP2));
			Point xyp;

			// check which navpoint is outside the NFZ
			if (!polyG->pointInside(navPoint1XY))
				xyp = navPoint1XY;
			else if (!polyG->pointInside(navPoint2XY))
				xyp = navPoint2XY;
			else  //throw error if both navpoint candidates are inside the NFZ
				throw EngineException("Error in losWaypointOperator::createNavpoints(). Both Navpoint candidates are inside NFZ");

			// get z coordinates for point
			PointGeometryPtr pg = new PointGeometry(xyp);
			FeaturePtr fWgs = new Feature(pg, Crs::wgs84LongLat());
			FeaturePtr fLoc = fWgs->projectTo(ElevDataCoordRef)->get(0);

			// get the z-value in the crs of the elevation data
			double z = elevationIn->getValueAt(dynamic_cast<PointGeometry*>(fLoc->geometry().get())->point()) + WP_PLACE_HEIGHT_ABOVE_Z_MIN;

			Point wpCoord = Point(xyp.x(), xyp.y(), z);

			// create the new waypoint
			FeaturePtr waypoint = new Feature(
				new PointGeometry(wpCoord),
				Crs::wgs84LongLat()
			);

			waypoint->attributes()->set(Atom(AOI_CELL_NAME), -1);
			waypoint->attributes()->set(Atom(MAX_DISTANCE_NAME), -1);
			waypoint->attributes()->set(Atom(WAYPOINT_NAME), waypointIdCounter++);
			waypoint->attributes()->set(Atom(PITCH_LOW_NAME), PITCH_MIN);
			waypoint->attributes()->set(Atom(PITCH_HIGH_NAME), PITCH_MAX);
			waypoint->attributes()->set(Atom(YAW_LOW_NAME), YAW_MIN);
			waypoint->attributes()->set(Atom(YAW_HIGH_NAME), YAW_MAX);
			waypoint->attributes()->set(Atom(IS_LOS_WAYPOINT_NAME), false);
			waypointsV.push_back(waypoint);
			waypoints.push_back(waypoint);
		}
	}
}

void losWaypointOperator::computeWaypoints(FeatureEnumeratorPtr aoiIn)
{
	if (CREATE_NAVPOINTS)
		createNavpoints();
	
	// randomize the order in which aoiPoints are iterated
	std::vector<int> aoiPointsOrder = std::vector<int>(aoiPoints.size());
	int counter = 0;
	for (int i = 0; i < ((int)aoiPointsOrder.size()); i++)
		aoiPointsOrder[i] = counter++;
	shuffle(aoiPointsOrder, this->rndEngine);

	// do new waypoint calculations
	for (int aoiNum = 0; aoiNum < ((int)aoiPoints.size()); aoiNum++)
	{
		FeaturePtr AoiPoint = aoiPoints[aoiPointsOrder[aoiNum]];

		Point AoiPointCoord = dynamic_cast<PointGeometry*>(AoiPoint->geometry().get())->point();

		// this is vital if any AoiPoints are placed inside terrain
		// the AoiPoint will then be placed above the surface
		if (SET_AOI_POINTS_TO_GROUND_LEVEL)
		{
			FeaturePtr fLocCrs = AoiPoint->projectTo(ElevDataCoordRef)->get(0);
			double groundElev = elevationIn->getValueAt(dynamic_cast<PointGeometry*>(fLocCrs->geometry().get())->point());
			Point groundElevP = Point(AoiPointCoord.x(), AoiPointCoord.y(), groundElev + AOI_POINT_PLACE_ABOVE_GROUND_ELEV);
			
			AoiPoint  = new Feature(
				new PointGeometry(groundElevP),
				AoiPoint->crs(),
				dynamic_cast<AttributeSet*>(AoiPoint->attributes()->clone().get())
			);
			
		}

		FeaturePtr AoiPointWp = FeaturePtr();

		// Check if AoiCell can be seen from any already existing waypoints
		for (int i = 0; i < waypointsV.size(); i++)
		{
			// if the waypoint is not a LOS waypoint, continue.
			if (waypointsV[i]->attributes()->get(IS_LOS_WAYPOINT_NAME) == false)
				continue;

			MemoryDataSetPtr targetLosPoint = dynamic_cast<MemoryDataSet*>(targetIn->dataSet().get());
			MemoryDataSetPtr obsLosPoint = dynamic_cast<MemoryDataSet*>(observerIn->dataSet().get());

			{
				Guard guard(targetLosPoint);
				Guard guard2(obsLosPoint);
				targetLosPoint->insert(dynamic_cast<Feature*>(AoiPoint->clone().get()));
				obsLosPoint->insert(dynamic_cast<Feature*>(waypointsV[i]->clone().get()));
			}
			FeatureEnumeratorPtr resultEnum = targetLosOut->getFeatures(AoiPoint->crs());
			resultEnum->moveNext();
			FeaturePtr result = resultEnum->current();
			AttributeValue observerView;
			int observerViewInt;
			if (result->attributes()->tryGetValue("visibleBy", observerView))
				observerView.tryGetValue(observerViewInt);
			else
				throw Carmenta::Engine::EngineException("Error in losWaypointOperator. Cannot find 'visibleBy' attribute from TargetLosOperator output");
			{
				Guard guard(targetLosPoint);
				Guard guard2(obsLosPoint);
				targetLosPoint->clear();
				obsLosPoint->clear();
			}

			if (observerViewInt == 1 && joinCameraLosBounds(waypointsV[i], AoiPointCoord))
			{
				Point wpPoint = dynamic_cast<PointGeometry*>(waypointsV[i]->geometry().get())->point();
				AoiPointWp = new Feature(
					new PointGeometry(wpPoint),
					AoiPoint->crs()
				);
				AoiPointWp->attributes()->set(Atom(AOI_CELL_NAME), AoiPoint->attributes()->get(AOI_CELL_NAME));
				AoiPointWp->attributes()->set(Atom(MAX_DISTANCE_NAME), AoiPoint->attributes()->get(MAX_DISTANCE_NAME));
				AoiPointWp->attributes()->set(Atom(WAYPOINT_NAME), waypointsV[i]->attributes()->get(Atom(WAYPOINT_NAME)));
				AoiPointWp->attributes()->set(Atom(PITCH_LOW_NAME), waypointsV[i]->attributes()->get(PITCH_LOW_NAME));
				AoiPointWp->attributes()->set(Atom(PITCH_HIGH_NAME), waypointsV[i]->attributes()->get(PITCH_HIGH_NAME));
				AoiPointWp->attributes()->set(Atom(YAW_LOW_NAME), waypointsV[i]->attributes()->get(YAW_LOW_NAME));
				AoiPointWp->attributes()->set(Atom(YAW_HIGH_NAME), waypointsV[i]->attributes()->get(YAW_HIGH_NAME));
				AoiPointWp->attributes()->set(Atom(IS_LOS_WAYPOINT_NAME), true);
				break;
			}
		
		}

		// If AoiCellPoint could not get allocated to an existing waypoint, create a new waypoint
		if (AoiPointWp.get() == nullptr)
		{
			std::vector<FeaturePtr> viewshed;
			MemoryDataSetPtr viewshedPoint = dynamic_cast<MemoryDataSet*>(viewshedIn->dataSet().get());

			// do viewshed calculations
			{
				Guard guard(viewshedPoint);
				viewshedPoint->clear();
				viewshedPoint->insert(dynamic_cast<Feature*>(AoiPoint->clone().get()));
			}
			FeatureEnumeratorPtr resultEnum = viewshedOut->getFeatures(AoiPoint->crs());

			while (resultEnum->moveNext())
			{
				FeaturePtr newRaster = resultEnum->current();
				if (newRaster->attributes()->get(AOI_CELL_NAME) == AoiPoint->attributes()->get(AOI_CELL_NAME))
					viewshed.push_back(newRaster);
				else
					throw EngineException("Error in losWaypointOperator. viewshedAoi ID and AoiPoint ID differ");
			}

			// create a waypoint inside the calculated viewshed
			Point p;
			bool foundWaypoint = createWaypointPos(AoiPoint, viewshed, p);
			if (!foundWaypoint && WAYPOINT_GEN_FAILED_BEHAVIOR == 1)
				p = AoiPointCoord;
			else if (!foundWaypoint && WAYPOINT_GEN_FAILED_BEHAVIOR == 2)
				while (!createWaypointPos(AoiPoint, viewshed, p)); // force finding a waypoint

			AoiPointWp = new Feature(
				new PointGeometry(p),
				AoiPoint->crs()
			);
			AoiPointWp->attributes()->set(Atom(AOI_CELL_NAME), AoiPoint->attributes()->get(AOI_CELL_NAME));
			AoiPointWp->attributes()->set(Atom(MAX_DISTANCE_NAME), AoiPoint->attributes()->get(MAX_DISTANCE_NAME));
			AoiPointWp->attributes()->set(Atom(WAYPOINT_NAME), waypointIdCounter++);
			AoiPointWp->attributes()->set(Atom(PITCH_LOW_NAME), PITCH_MIN);
			AoiPointWp->attributes()->set(Atom(PITCH_HIGH_NAME), PITCH_MAX);
			AoiPointWp->attributes()->set(Atom(YAW_LOW_NAME), YAW_MIN);
			AoiPointWp->attributes()->set(Atom(YAW_HIGH_NAME), YAW_MAX);
			AoiPointWp->attributes()->set(Atom(IS_LOS_WAYPOINT_NAME), true);
			joinCameraLosBounds(AoiPointWp, AoiPointCoord);
			if(foundWaypoint) waypointsV.push_back(AoiPointWp);
		}

		// store waypoint in vector
		waypoints.push_back(AoiPointWp);
	}
}

/**
* Method which creates a waypoint for a given AoiCellPoint under the constraints of passed viewshed
* and the No Fly Zone (NFZ) list member variable
*
* @AoiPoint The Area Of Interest (AOI) point to create a waypoint for
* @viewshed the viewshed of the AoiPoint. Given as a volume encoding raster
* @result the point to mutate the result to
* @return True if a waypoint was created. Otherwise false
*/
bool losWaypointOperator::createWaypointPos(const FeaturePtr& AoiPoint, const std::vector<FeaturePtr>& viewshed, Point& result)
{
	std::vector<int> candidateViewshedCells = std::vector<int>(0);

	Point AoiCoord = dynamic_cast<PointGeometry*>(AoiPoint->geometry().get())->point();
	Point crsRatio = AoiPoint->crs()->metersXY(
		Point(AoiCoord));

	// iterate through all viewshed rasters and select only the ones which
	// are partly covered by the NFZ
	for (int i = 0; i < viewshed.size(); i++)
	{
		RasterGeometryPtr geom = dynamic_cast<RasterGeometry*>(viewshed[i]->geometry().get());
		switch (rectangleCollidePolygon(geom->bounds(), this->noFlyZones))
		{
		case 0:
			break;
		case 1:
			candidateViewshedCells.push_back(i);
			break;
		case 2:
			break;
		default:
			throw EngineException("Error in losWaypointOperator. Illegal output received from rectangleCollidePolygon");
		}
	}

	// If we can't find any candidate viewshed cells, throw error
	if (candidateViewshedCells.empty())
	{
		if (WAYPOINT_GEN_FAILED_BEHAVIOR == 0)
			throw EngineException("Error in losWaypointOperator. AoiPoint viewshed does not touch any NFZ edge");
		else
			return false;
	}


	// shuffle the candidate viewshedCells
	shuffle(candidateViewshedCells, rndEngine);

	// potential waypoints
	std::vector<Point> potWp = std::vector<Point>(0);

	// Iterate through all candidate viewshedCells
	for (std::vector<int>::iterator vsc = candidateViewshedCells.begin(); vsc < candidateViewshedCells.end(); vsc++)
	{
		RasterGeometryPtr geom = dynamic_cast<RasterGeometry*>(viewshed[*vsc]->geometry().get());

		// get the elevation data as pixels (short)
		int viewshedCellSideLenX = std::round((crsRatio.x() * (geom->bounds().xMax() - geom->bounds().xMin())));
		int viewshedCellSideLenY = std::round((crsRatio.y() * (geom->bounds().yMax() - geom->bounds().yMin())));
		int numPixels =
			(viewshedCellSideLenX * PIXEL_SIZE) * (viewshedCellSideLenY * PIXEL_SIZE);
		int numSidePixelsX = viewshedCellSideLenX * PIXEL_SIZE;
		int numSidePixelsY = viewshedCellSideLenY * PIXEL_SIZE;
		const unsigned char* data = geom->raster()->readOnlyData();

		// determine how many pixels to inspect
		int numPixInspect;
		if (numPixels < PIXEL_THRESHOLD_LINEAR_INSPECTION)
			numPixInspect = numPixels;
		else numPixInspect = int(std::log2(numPixels));

		// randomize the order in which the pixels are inspected
		std::vector<int> pixelOrdering = std::vector<int>(numPixInspect);
		std::uniform_int_distribution<> distrib(0, numPixels - 1);
		for (int i = 0; i < pixelOrdering.size(); i++)
			pixelOrdering[i] = distrib(rndEngine);

		// Iterate through candidate viewshedCell pixels
		for (int i = 0; i < pixelOrdering.size(); i++)
		{
			// viewshed cell starts from (lowX, lowY) and then
			// ascends in X-direction
			int pixelCol = pixelOrdering[i] % numSidePixelsX;
			int pixelRow = pixelOrdering[i] / numSidePixelsX;

			double pixelXMeter = pixelCol * PIXEL_SIZE;
			double pixelYMeter = pixelRow * PIXEL_SIZE;

			double pixelX = geom->bounds().xMin() + pixelXMeter / crsRatio.x();
			double pixelY = geom->bounds().yMin() + pixelYMeter / crsRatio.y();
			// get the low short-type value and set it as the Z-coordinate
			// multiply by two two times because a short datatype is two bytes
			// and each pixel contains two shorts (high and low elev value)
			unsigned short pixelZLow = *(data + pixelOrdering[i] * 2 * 2);
			unsigned short pixelZHigh = *(data + pixelOrdering[i] * 2 * 2 + 2);

			unsigned short waypointZ = pixelZLow + WP_PLACE_HEIGHT_ABOVE_Z_MIN <= pixelZHigh ?
				pixelZLow + WP_PLACE_HEIGHT_ABOVE_Z_MIN : (pixelZLow + pixelZHigh) / 2;

			// Create and add waypoint if satisfaction of constraints
			Point waypoint = Point(pixelX, pixelY, waypointZ);

			if (pixelZHigh == 0 || pixelZLow > pixelZHigh || pixelZLow == pixelZHigh) // 0 or nonsensical elevation = not within LOS
				continue;

			FeaturePtr pf = new Feature(new PointGeometry(waypoint), AoiPoint->crs());
			// must not collide with any NFZ
			if (pointCollide(pf, noFlyZones, 0))
				continue;
			// is able to bee-line to any other waypoint without going through NFZ
			// is exempted from this requirement if this is first waypoint being placed
			if(ONLY_CREATE_WAYPOINTS_WITH_BEELINE)
				if (waypointsV.size() != 0 && !pointLOSObserversFog(pf, waypointsV, noFlyZones))
					continue;
			potWp.push_back(waypoint);
		}
	}

	// If we can't find any candidate pixels, we have a problem
	if (potWp.empty()) {
		if (WAYPOINT_GEN_FAILED_BEHAVIOR == 0)
			throw EngineException("Error in losWaypointOperator. No potential pixels found in candidate viewshed cells");
		else
			return false;
	}

	Point chosenP = *potWp.begin();

	// check all potentialWaypoints. Choose shortest distance to AoiPoint
	for (std::vector<Point>::iterator it = potWp.begin(); it < potWp.end(); it++)
	{
		if (pointDist(AoiCoord, *it, AoiPoint->crs()) <
			pointDist(AoiCoord, chosenP, AoiPoint->crs()))
			chosenP = Point(*it);

	}

	result = chosenP;

	return true;
}

/**
* Method which attempts to further constrain the camera LOS bounds for an observation
* point with regards to an additional target point. If the function returns true,
* it will also have modified the camera LOS bound attributes for the observer.
* Using FOV:s larger than 180 degrees is not supported.
*
* @observer The observer
* @target the point which the function will attempt to constrain the observer to
* @return false if target camera constraints conflict with existing observer camera
* constraints. Otherwise return true.
*/
bool losWaypointOperator::joinCameraLosBounds(FeaturePtr& observer, const Point target)
{
	Point obsPoint = dynamic_cast<PointGeometry*>(observer->geometry().get())->point();
	// boundsAoiPoint
	std::vector<int> ba = std::vector<int>(4);
	// boundsObserverPoint
	std::vector<int> bo = std::vector<int>(4);
	std::vector<int> BoundsJoin = std::vector<int>(4);
	getPitchAndYawBounds(obsPoint, target, observer->crs(), ba);
	observer->attributes()->get(PITCH_LOW_NAME).tryGetValue(bo[0]);
	observer->attributes()->get(PITCH_HIGH_NAME).tryGetValue(bo[1]);
	observer->attributes()->get(YAW_LOW_NAME).tryGetValue(bo[2]);
	observer->attributes()->get(YAW_HIGH_NAME).tryGetValue(bo[3]);

	if (!pitchYawIntervalJoiner(ba, bo, BoundsJoin))
		return false;
	else
	{
		observer->attributes()->set(PITCH_LOW_NAME, BoundsJoin[0]);
		observer->attributes()->set(PITCH_HIGH_NAME, BoundsJoin[1]);
		observer->attributes()->set(YAW_LOW_NAME, BoundsJoin[2]);
		observer->attributes()->set(YAW_HIGH_NAME, BoundsJoin[3]);
		return true;
	}
}

/**
* Method used to determine if a certain point is inside of a
* vector of volume encoding rasters
*
* @ver The volume encoding raster
* @f The point feature to check if it is inside of the ver
* @returns true if p is inside of ver, otherwise false
*/
bool losWaypointOperator::LineOfSight(const std::vector<FeaturePtr>& ver, const FeaturePtr& f)
{
	const Point p = dynamic_cast<PointGeometry*>(f->geometry().get())->point();

	// find which viewshed cell the point is inside of
	int vscIndex = findPointInVerV(ver, p);

	// return false if the point isn't inside any viewshedCell
	if (vscIndex == -1)
		return false;

	FeaturePtr vsc = ver[vscIndex];

	// determine if the pixel in viewshedCell covers p
	bool pixelCovered = isPointInVer(vsc, p);

	return pixelCovered;
}

#endif