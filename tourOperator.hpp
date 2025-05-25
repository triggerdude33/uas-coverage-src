#ifndef TOUR_OPERATOR_HPP
#define TOUR_OPERATOR_HPP

#include "constants.hpp"
#include "helpers.hpp"
#include <Carmenta/Engine/Core.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

#define K 10
#define OPTIMISE_ITERATIONS 3
#define MAX_N 1000

using namespace std;

using namespace Carmenta::Engine;

class tVertex {
private:
    int m_name; // The 0-based index from the input
    double m_x; // x-coordinate
    double m_y; // y-coordinate
    double m_z; // z-coordinate
    Point m_ratio; // the ratio between crs coordinates and meters

public:
    tVertex(int name, double x, double y, double z, Point ratio);

    tVertex();

    double distance(tVertex* tVertex2);

    int name() const;
    double x() const;
    double y() const;
    double z() const;
};

class tEdge {
private:
    tVertex* m_v1;
    tVertex* m_v2;
    double m_length;

public:
    tEdge(tVertex* vertex1, tVertex* vertex2);
    tEdge(tVertex* vertex1, tVertex* vertex2, FeaturePtr nfz);

    double length() const;
    tVertex* v1() const;
    tVertex* v2() const;

    bool operator<(const tEdge& other) const;

    bool operator>(const tEdge& other) const;
};

class tourOperator
{
public:
	explicit tourOperator(SplitOperatorPtr wpIn, SplitOperatorPtr nfzIn,
        ReadOperatorPtr tourOut);

    bool featureUpdate();

private:
	Carmenta::Engine::SplitOperatorPtr wpIn;
    SplitOperatorPtr nfzIn;

    tVertex vertices[MAX_N];
    // used in initial tour construction. All edges which don't pass through no fly zones
    priority_queue<tEdge, vector<tEdge>, greater<tEdge>> edgesTour;

    // used in initial tour construction. All edges which pass through no fly zones
    priority_queue<tEdge, vector<tEdge>, greater<tEdge>> edgesTourNfz;

    // used in k nearest neighbours for each vertex mapping
    // only has edges which don't pass through no fly zones
    priority_queue<tEdge, vector<tEdge>, greater<tEdge>> edgesneighbours;

    // feature vectors
    vector<FeaturePtr> waypoints;
    std::vector<FeaturePtr> noFlyZones;
    
    ReadOperatorPtr tourOut;

    double nfzDistance(const tVertex* v1, const tVertex* v2);

    int N;
    int tour[MAX_N][2]; // the tour that will be constructed and then optimised
    int neighbours[MAX_N][K]; // the k nearest neighbours for each vertex

    void readInput();
    void initialiseTour();
    void initialiseneighbours();
    void constructGraph();
    void writeGraphToOutput();
    void constructTour();
    int Dfs(int v, int parent, int count);
    int DfsAux(int v, int parent, int count, bool* visited);
    
    void directTour(int start, int predecessor);
    void determineClosestNeighbours();
    void optimiseTour();
    void twoOpt(int v1, int v2, int v3, int v4);
    void produceOutput();

    void produceDebugOutput();
    void printTourCompact();

    // used for debugging
    int graphNum;
};



#endif