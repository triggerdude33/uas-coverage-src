#ifndef HELPERS_HPP
#define HELPERS_HPP

#include "constants.hpp"
#include <Carmenta/Engine/Core.hpp>
#include <vector>
#include <random>

using namespace Carmenta::Engine;

// helper methods
bool inputModified(std::vector<FeaturePtr>& points, std::vector<FeaturePtr>& noFlyZones, FeatureEnumeratorPtr aoiIn, FeatureEnumeratorPtr nfzIn);
bool polygonEquality(FeaturePtr a, FeaturePtr b);
bool pointEquality(FeaturePtr a, FeaturePtr b, double round);
bool pitchYawIntervalJoiner(const std::vector<int>& a, const std::vector<int>& b, std::vector<int>& answer);
void shuffle(std::vector<int>& v, std::mt19937& rndEngine);
bool pointLOSObserversFog(const FeaturePtr& pf,
	const std::vector<FeaturePtr> observers, const std::vector<FeaturePtr> fog);
bool pointCollide(const FeaturePtr& pf, const std::vector<FeaturePtr>& obs, double margin);
double roundTo(double value, double precision);
int rectangleCollidePolygon(const Carmenta::Engine::Rectangle r, const std::vector<FeaturePtr> obs);
double pointDist(const Point& p1, const Point& p2, const CrsPtr& c);
void getPitchAndYawBounds(const Point observer, const Point target, const CrsPtr& c, std::vector<int>& output);
bool isPointInVer(const FeaturePtr vsc, const Point p);
int findPointInVerV(const std::vector<FeaturePtr>& ver, const Point p);
double verticalFOV(double fovH, int aspectRatioH, int aspectRatioV);

#endif