#ifndef PATH_OPERATOR_HPP
#define PATH_OPERATOR_HPP

#include "constants.hpp"
#include <windows.h>
#include "helpers.hpp"

#include <nlohmann/json.hpp>
#include <Carmenta/Engine/Core.hpp>
#include <unordered_map>
#include <list>
#include <cmath>

using namespace Carmenta::Engine;
using namespace std;

/**
	@author Matteus Berg
	@date 2025-01-15
*/
class pathOperator
{
public:
	explicit pathOperator(
		std::string& baseUrl, 
		SplitOperatorPtr wpIn, 
		SplitOperatorPtr nfzIn,
		ReadOperatorPtr legsOut);

	bool featureUpdate();

private:
	void computePaths();
	bool getLegs(const nlohmann::json& body, nlohmann::json& answer);
	nlohmann::json post(const std::string& endpoint, const nlohmann::json& body);

	std::string baseUrl;
	SplitOperatorPtr wpIn;
	SplitOperatorPtr nfzIn;
	ReadOperatorPtr legsOut;
	std::vector<Carmenta::Engine::FeaturePtr> noFlyZones;

	// vector containing all waypoints generated by the losWaypointOperator
	vector<FeaturePtr> waypoints;

	// No fly zone JSON body to send as a geofence along each REST request
	nlohmann::json nfzBody;

};


#endif