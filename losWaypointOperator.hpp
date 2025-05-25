#ifndef losWaypointOperator_HPP
#define losWaypointOperator_HPP

#include "constants.hpp"
#include "helpers.hpp"
#include <Carmenta/Engine/Core.hpp>
#include <Carmenta/Engine/DataSets.hpp>
#include <vector>
#include <chrono>

using namespace Carmenta::Engine;

// Author: Matteus Berg
// Date: 2024-11-19
/* Custom operator that generates a waypoint for LOS surveillance of a certain AOI point
*
*/
class losWaypointOperator
{
public:
	// Initializes a new instance of the losWaypointOperator class.
	losWaypointOperator(
		CrsPtr coordRef,
		SplitOperatorPtr aoiIn,
		SplitOperatorPtr nfzIn,
		ReadOperatorPtr wpOut,
		ReadOperatorPtr targetIn,
		ReadOperatorPtr observerIn,
		SplitOperatorPtr targetLosOut,
		ReadOperatorPtr viewshedIn,
		SplitOperatorPtr viewshedOut,
		ImageDataSetPtr elevationIn);

	bool featureUpdate();
	bool decrementSeed();
	bool incrementSeed();
	int getSeedNum();

private:
	// methods
	void computeWaypoints(FeatureEnumeratorPtr aoiIn);

	bool createWaypointPos(
		const Carmenta::Engine::FeaturePtr& AoiPoint,
		const std::vector<FeaturePtr>& viewshed, Point& result);
	bool joinCameraLosBounds(FeaturePtr& observer, const Point target);
	bool LineOfSight(const std::vector<FeaturePtr>& ver, const FeaturePtr& f);
	void createNavpoints();

	SplitOperatorPtr aoiIn;
	SplitOperatorPtr nfzIn;
	ReadOperatorPtr wpOut;

	ReadOperatorPtr targetIn;
	ReadOperatorPtr observerIn;
	SplitOperatorPtr targetLosOut;

	ReadOperatorPtr viewshedIn;
	SplitOperatorPtr viewshedOut;
		
	ImageDataSetPtr elevationIn;

	// the crs for the elevation data
	const CrsPtr ElevDataCoordRef;

	// all the no fly zones for the mission
	std::vector<Carmenta::Engine::FeaturePtr> noFlyZones;
	// all the area of interest points for the mission
	std::vector<Carmenta::Engine::FeaturePtr> aoiPoints;
	// all waypoints for the mission
	std::vector<Carmenta::Engine::FeaturePtr> waypoints;
	// all valid waypoints for the mission
	std::vector<Carmenta::Engine::FeaturePtr> waypointsV;
	// used for wayPoint location randomization
	std::mt19937 rndEngine;
	int seedCounter;

	int NfzId;
	int AoiPointsId;
	int ViewshedsId;

	int waypointIdCounter;
	int AoiCellUpdateCounter;

	bool redoCalc;

	const Carmenta::Engine::String AoiPointsInputName = "AOICenterPointSplit";
	const Carmenta::Engine::String ViewshedsInputName = "ViewshedSplit";
	const Carmenta::Engine::String NfzInputName = "NFZSplit";
};










#endif
