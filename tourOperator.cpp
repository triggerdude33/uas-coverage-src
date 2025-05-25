#ifndef TOUR_OPERATOR_CPP
#define TOUR_OPERATOR_CPP

#include "tourOperator.hpp"

tourOperator::tourOperator(SplitOperatorPtr wpIn, SplitOperatorPtr nfzIn,
    ReadOperatorPtr tourOut)
    : wpIn(wpIn), nfzIn(nfzIn), tourOut(tourOut), N(0), graphNum(1)
{
    if (this->wpIn.get() == nullptr)
        throw EngineException("Error in tourOperator::tourOperator. wpIn shared_ptr does not own an object");
}

/**
  @date 2023-11-08
  @author Matteus Berg, Jakob Alfredsson, Erik Hedlund, Gabriel Staifo

  2024-12-28 rewritten for thesis work by Matteus Berg
*/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

using namespace std;

// tVertex function definitions
tVertex::tVertex(int name, double x, double y, double z, Point ratio) : m_name(name), m_x(x), m_y(y), m_z(z), m_ratio(ratio) {}

tVertex::tVertex() : m_name(0), m_x(0), m_y(0), m_z(0) {}

double tVertex::distance(tVertex* vertex2) {
    return sqrt(
        (pow(m_ratio.x() * (this->x() - vertex2->x()), 2.0)) +
        (pow(m_ratio.y() * (this->y() - vertex2->y()), 2.0)) +
        pow(this->z() - vertex2->z(), 2.0)
    );
}

// assumes that there is only 1 nfz in member vector and that this
// nfz only has 1 ring
double nfzDistance(const tVertex* va, const tVertex* vb, FeaturePtr nfz)
{
    Point vPointA = Point(va->x(), va->y(), va->z());
    Point vPointB = Point(vb->x(), vb->y(), vb->z());
    LineGeometryPtr nfzRing = 
        dynamic_cast<LineGeometry*>(
        dynamic_cast<PolygonGeometry*>(
            nfz->geometry().get())->rings()->begin()->get());

    Point ringPointA = nfzRing->nearestPoint(vPointA);
    Point ringPointB = nfzRing->nearestPoint(vPointB);
    int segmentA = nfzRing->segmentNearPoint(ringPointA);
    int segmentB = nfzRing->segmentNearPoint(ringPointB);

    // same segment
    if (segmentA == segmentB)
        return pointDist(ringPointA, ringPointB, Crs::wgs84LongLat());

    // different segment
    // ascending order
    double distanceAsc = 0;
    Point Point1 = ringPointA;
    Point Point2 = nfzRing->points()->get(segmentA + 1);
    while (segmentA != segmentB)
    {
        distanceAsc += pointDist(Point1, Point2, Crs::wgs84LongLat());

        segmentA = segmentA + 1 < nfzRing->points()->size() ? segmentA + 1 : 0;
        Point1 = Point2;
        Point2 = nfzRing->points()->get(segmentA + 1);
    }
    Point2 = ringPointB;
    distanceAsc += pointDist(Point1, Point2, Crs::wgs84LongLat());

    // descending order
    segmentA = nfzRing->segmentNearPoint(ringPointA);
    double distanceDesc = 0;
    Point1 = ringPointA;
    Point2 = nfzRing->points()->get(segmentA);
    while (segmentA != segmentB)
    {
        distanceDesc += pointDist(Point1, Point2, Crs::wgs84LongLat());

        segmentA = segmentA - 1 > -1 ? segmentA - 1 : nfzRing->points()->size() - 1;
        Point1 = Point2;
        Point2 = nfzRing->points()->get(segmentA);
    }
    Point2 = ringPointB;
    distanceDesc += pointDist(Point1, Point2, Crs::wgs84LongLat());

    return min(distanceAsc, distanceDesc);
}

int tVertex::name() const { return m_name; }
double tVertex::x() const { return m_x; }
double tVertex::y() const { return m_y; }
double tVertex::z() const { return m_z; }

// tEdge function definitions
tEdge::tEdge(tVertex* vertex1, tVertex* vertex2)
    : m_v1(vertex1), m_v2(vertex2), m_length(vertex1->distance(vertex2)) {}

tEdge::tEdge(tVertex* vertex1, tVertex* vertex2, FeaturePtr nfz)
    : m_v1(vertex1), m_v2(vertex2), m_length(nfzDistance(vertex1, vertex2, nfz)) {}

double tEdge::length() const { return m_length; }
tVertex* tEdge::v1() const { return m_v1; }
tVertex* tEdge::v2() const { return m_v2; }

bool tEdge::operator<(const tEdge& other) const {
    return (this->length() < other.length());
}

bool tEdge::operator>(const tEdge& other) const {
    return (this->length() > other.length());
}

// Tour operator function definitions

/* Initialise the K nearest neighbours for each vertex to -1 */
void tourOperator::initialiseneighbours() 
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < K; j++) {
            neighbours[i][j] = -1;
        }
    }
}

/* Initialise all connections for all vertices to -1 */
void tourOperator::initialiseTour() 
{
    for (int i = 0; i < N; i++) {
        tour[i][0] = -1;
        tour[i][1] = -1;
    }
}

/* Reads the input */
void tourOperator::readInput() 
{
    for (int i = 0; i < (int)(waypoints.size()); i++) {

        Carmenta::Engine::FeaturePtr wpFeat = waypoints[i];

        // assert that waypoint attribute ID comes in incrementing order
        Carmenta::Engine::AttributeValue wpId;

        if (!wpFeat->attributes()->tryGetValue(WAYPOINT_NAME, wpId))
            throw Carmenta::Engine::EngineException("Error in tourOperator::readInput. No Id for input waypoint found");
        int wpIdVal;
        if (!wpId.tryGetValue(wpIdVal))
            throw Carmenta::Engine::EngineException("Error in tourOperator::readInput. Waypoint Id is in non-integer format");

        if (wpIdVal != i)
            throw Carmenta::Engine::EngineException("Error in tourOperator::readInput. Waypoints are not given in incrementing Waypoint ID order");

        // add waypoint coordinates to the vertices array
        Carmenta::Engine::Point p = dynamic_cast<Carmenta::Engine::PointGeometry*>(wpFeat->geometry().get())->point();
        Point ratio = wpFeat->crs()->metersXY(p);
        vertices[i] = tVertex(i, p.x(), p.y(), p.z(), ratio);
    }
}

/* Produces the output to tourOut ReadOperator */
void tourOperator::produceOutput() {
    MemoryDataSetPtr tourOutDataSet = dynamic_cast<MemoryDataSet*>(this->tourOut->dataSet().get());

    {
        Guard guard(tourOutDataSet);
        tourOutDataSet->clear();

        // Base cases
        switch (N) {
        case 1:
            tourOutDataSet->insert(dynamic_cast<Feature*>(waypoints[0]->clone().get()));
            return;
        case 2:
            tourOutDataSet->insert(dynamic_cast<Feature*>(waypoints[0]->clone().get()));
            tourOutDataSet->insert(dynamic_cast<Feature*>(waypoints[1]->clone().get()));
            return;
        }

        // Cases for N > 2
        int currentIndex = 0;
        int parent = -2;
        int tourOrder = 0;
        do {
            FeaturePtr tourWp = dynamic_cast<Feature*>(waypoints[currentIndex]->clone().get());
            tourWp->attributes()->set(Atom(TOUR_ORDER_NAME), tourOrder);
            tourOrder++;
            tourOutDataSet->insert(tourWp);
            
            int old_parent = parent;
            parent = currentIndex;
            currentIndex = (tour[currentIndex][1] != old_parent)
                ? tour[currentIndex][1]
                : tour[currentIndex][0];
        } while (currentIndex != 0);
    }

    
}

/* Used for debugging, ignore this */
void tourOperator::produceDebugOutput() {

    // Base cases
    switch (N) {
    case 1:
        cout << 0 << endl;
        return;
    case 2:
        cout << 0 << endl;
        cout << 1 << endl;
        return;
    }

    int currentIndex = 0;
    int parent = -2;
    do {
        cout << currentIndex << " " << flush;
        int old_parent = parent;
        parent = currentIndex;
        currentIndex = (tour[currentIndex][0] != old_parent)
            ? tour[currentIndex][0]
            : tour[currentIndex][1];
    } while (currentIndex != 0);
    cout << endl;
}

/* Used for debugging, ignore this */
void tourOperator::printTourCompact() {
    int node = 0;
    cout << "Tour: " << flush;
    for (int i = 0; i < N; i++) {
        cout << node << "->" << flush;
        node = tour[node][1];
    }
    cout << endl;
}

/* Create edges between all vertices and add them to three priority queues. edgesTour
and edgesTourNfz are used to construct the tour. edgesneighbours are used to create
the closest neighbours array */
void tourOperator::constructGraph() {
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            vector<FeaturePtr> other;
            other.push_back(waypoints[j]);

            // if bee-line does not intersect with Nfz, add edge to edgesTour
            if (pointLOSObserversFog(waypoints[i], other, noFlyZones))
            {
                // in order for the greedy tour construction to not give a tour going through the NFZ,
                // we need to reformulate the construction as a one dimensional problem
                edgesTour.push(tEdge(&vertices[i], &vertices[j], noFlyZones.begin()->get()));
                edgesneighbours.push(tEdge(&vertices[i], &vertices[j]));
            }
            else
            {
                edgesTourNfz.push(tEdge(&vertices[i], &vertices[j]));
            }
            
        }
    }
}

void tourOperator::writeGraphToOutput()
{
    string output = "";
    output.append(std::to_string(N));
    output.append("\n");

    tVertex* v1;
    tVertex* v2;
    double dist;

    while (!edgesTour.empty())
    {
        v1 = edgesTour.top().v1();
        v2 = edgesTour.top().v2();
        dist = edgesTour.top().length();
        edgesTour.pop();

        output.append(std::to_string(v1->name()));
        output.append(" ");
        output.append(std::to_string(v2->name()));
        output.append(" ");
        output.append(std::to_string(dist));
        output.append("\n");
    }

    string graphFileName = "graph";
    graphFileName.append(std::to_string(graphNum));
    graphFileName.append(".txt");
    graphNum++;

    std::ofstream(graphFileName, std::ios_base::app) << output;

    throw Carmenta::Engine::EngineException("Error. You are running the tourOperator::writeGraphToOutput function. Delete its call in tourOperator::featureUpdate()");
}

/* An auxiliary function for the DFS. Modified to only check
a maximum of two edges per vertex */
int tourOperator::DfsAux(int v, int parent, int count, bool* visited) {
    visited[v] = true;

    // Determine which connected node is the parent
    if (tour[v][0] == parent) {
        if (tour[v][1] == -1)
            return -1;
        else if (visited[tour[v][1]]) {
            return ++count;
        }
        else
            return DfsAux(tour[v][1], v, ++count, visited);
    }

    else if (tour[v][1] == parent) {
        if (tour[v][0] == -1)
            return -1;
        else if (visited[tour[v][0]] == true) {
            return ++count;
        }
        else
            return DfsAux(tour[v][0], v, ++count, visited);
    }
    // true only first time entering the function
    else if (-2 == parent) {
        int result1 = -1;
        int result2 = -1;
        if (tour[v][0] != -1) {
            result1 = DfsAux(tour[v][0], v, ++count, visited);
        }
        if (tour[v][1] != -1) {
            result2 = DfsAux(tour[v][1], v, ++count, visited);
        }

        return max(result1, result2);
    }

    cerr << "parent: " << parent << endl;
    throw runtime_error("invalid parent value inserted into DFS");
}

/* Depth First Search */
int tourOperator::Dfs(int v, int parent, int count) {
    bool visited[MAX_N];
    for (int i = 0; i < N; i++)
        visited[i] = false;
    return DfsAux(v, parent, count, visited);
}

/* Constructs tour in the greedy approach
adds edge to route iff it does not create cycle
smaller than N and both verticies have fewer
than two neighbours */
void tourOperator::constructTour() {
    // special case for N < 3
    if (N < 3)
        return;

    bool quit = false;

    // quit when we reach cycle of size N
    while (!quit) {

        tVertex* v1;
        tVertex* v2;

        // if we have run out of edges in edgesTour and still don't have a tour, we must
        // iterate through the edges which go through No Fly Zones (NFZ)
        if (!edgesTour.empty())
        {
            v1 = edgesTour.top().v1();
            v2 = edgesTour.top().v2();
            edgesTour.pop();
        }
        else
        {
            v1 = edgesTourNfz.top().v1();
            v2 = edgesTourNfz.top().v2();
            edgesTourNfz.pop();
        }
        
        int emptyV1 = -1;
        int emptyV2 = -1;

        // if v1's first connection slot is empty
        if (tour[v1->name()][0] == -1)
            emptyV1 = 0;
        // else check if v1's second connection slot is empty
        else if (tour[v1->name()][1] == -1)
            emptyV1 = 1;
        // same for v2
        if (tour[v2->name()][0] == -1)
            emptyV2 = 0;

        else if (tour[v2->name()][1] == -1)
            emptyV2 = 1;

        // if any vertex has two connected vertices or more
        if (emptyV1 == -1 || emptyV2 == -1)
            continue;

        // assign to empty slots
        tour[v1->name()][emptyV1] = v2->name();
        tour[v2->name()][emptyV2] = v1->name();

        // run DFS
        int DFSResult = Dfs(v1->name(), -2, 0);

        /* if found cycle of size N, quit. If a cycle shorter than that is found,
        remove connection */
        if (DFSResult == N) {
            quit = true;
        }
        else if (DFSResult != -1) {
            tour[v1->name()][emptyV1] = -1;
            tour[v2->name()][emptyV2] = -1;
        }
    }


}

/* Rearranges connection slots so that
the tour becomes a directed cycle of size N */
void tourOperator::directTour(int start, int predecessor) {

    int node = start;
    int temp;

    do {
        if (tour[node][0] != predecessor) {
            /* Swap the order */
            temp = tour[node][0];
            tour[node][0] = tour[node][1];
            tour[node][1] = temp;
        }

        predecessor = node;
        node = tour[node][1];
    } while (node != start);
}

/* map the k nearest neighbours for each vertex */
void tourOperator::determineClosestNeighbours() {

    /* For each neighbour slot, assign neighbours in
      ascending distance order. Order guaranteed by
      priority queue */
    for (tEdge e = edgesneighbours.top(); !edgesneighbours.empty();
        edgesneighbours.pop()) {
        e = edgesneighbours.top();
        tVertex* v1 = e.v1();
        tVertex* v2 = e.v2();
        bool assignedV1 = false;
        bool assignedV2 = false;

        // find (if possible) empty neighbour slot to insert into
        for (int i = 0; i < K; i++) {
            if (neighbours[v1->name()][i] == -1 && !assignedV1) {
                neighbours[v1->name()][i] = v2->name();
                assignedV1 = true;
            }
            if (neighbours[v2->name()][i] == -1 && !assignedV2) {
                neighbours[v2->name()][i] = v1->name();
                assignedV2 = true;
            }
            if (assignedV1 && assignedV2)
                break;
        }
    }
}

/* Take two edges and swap their vertices if
swapped distance is shorter */
void tourOperator::twoOpt(int v1, int v2, int v3, int v4) {
    // determine distances
    double distv1v2 = vertices[v1].distance(&vertices[v2]);
    double distv3v4 = vertices[v3].distance(&vertices[v4]);
    double distv1v3 = vertices[v1].distance(&vertices[v3]);
    double distv2v4 = vertices[v2].distance(&vertices[v4]);

    // see to that the switch does not cause the tour to go through NFZ
    FeaturePtr v1p = new Feature(new PointGeometry(Point(vertices[v1].x(), vertices[v1].y())), Crs::wgs84LongLat());
    FeaturePtr v2p = new Feature(new PointGeometry(Point(vertices[v2].x(), vertices[v2].y())), Crs::wgs84LongLat());
    FeaturePtr v3p = new Feature(new PointGeometry(Point(vertices[v3].x(), vertices[v3].y())), Crs::wgs84LongLat());
    FeaturePtr v4p = new Feature(new PointGeometry(Point(vertices[v4].x(), vertices[v4].y())), Crs::wgs84LongLat());
    vector<FeaturePtr> v3pVec = vector<FeaturePtr>(); v3pVec.push_back(v3p);
    vector<FeaturePtr> v4pVec = vector<FeaturePtr>(); v4pVec.push_back(v4p);
    
    if(!pointLOSObserversFog(v1p, v3pVec, noFlyZones)) return;
    if(!pointLOSObserversFog(v2p, v4pVec, noFlyZones)) return;

    // if new distance is smaller than old distance
    if (distv1v3 + distv2v4 < distv1v2 + distv3v4) {

        tour[v1][1] = v3;
        tour[v3][1] = v1;
        tour[v2][0] = v4;
        tour[v4][0] = v2;

        // redirect tour after edge switch
        directTour(v1, v3);
    }
}

/* Iteratively calls 2-Opt on each vertex */
void tourOperator::optimiseTour() {
    for (int m = 0; m < OPTIMISE_ITERATIONS; m++) {
        // for each vertex v1
        for (int v1 = 0; v1 < N; v1++) {
            // iterate over the K nearest neighbours of v1
            for (int i = 0; i < min(K, N - 1); i++) {
                // v2 is v1's i:th nearest neighbour
                int v2 = neighbours[v1][i];
                int v1Other;
                int v2Other;

                /* case 1. v1's and v2's connections are to their predecessors */
                v1Other = tour[v1][0];
                v2Other = tour[v2][0];
                if (v1Other != v2 && v2Other != v1 && v1Other != v2Other)
                    twoOpt(v1Other, v1, v2Other, v2);

                /* case 2. v1's connection is to it's successor.
                v2's connection is to its predecessor */
                v1Other = tour[v1][1];
                v2Other = tour[v2][0];
                if (v1Other != v2 && v1Other != v2Other && v1 != v1Other)
                    twoOpt(v1, v1Other, v2Other, v2);

                /* case 3. v1's and v2's connections are to their successors */
                v1Other = tour[v1][1];
                v2Other = tour[v2][1];
                if (v1Other != v2 && v1Other != v2Other && v1 != v1Other)
                    twoOpt(v1, v1Other, v2, v2Other);
            }
        }
    }
}

/**
* Updates the tour if the input to this operator has been modified.
* 
* @return true if tour has been updated. Otherwise false
*/
bool tourOperator::featureUpdate() {
    FeatureEnumeratorPtr waypointFeatures = wpIn->getFeatures(Crs::wgs84LongLat());
    FeatureEnumeratorPtr nfzFeatures = nfzIn->getFeatures(Crs::wgs84LongLat());

    if (inputModified(this->waypoints, this->noFlyZones, waypointFeatures, nfzFeatures))
    {
        this->N = (int)(waypoints.size());

        // calculate new tour
        readInput();
        initialiseTour();
        initialiseneighbours();
        constructGraph();
        constructTour();
        directTour(0, tour[0][0]);
        determineClosestNeighbours();
        optimiseTour();
        produceOutput();

        return true;
    }
    else
        return false;
}

#endif