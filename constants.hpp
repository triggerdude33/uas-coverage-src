#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <httplib.h>
#include <unordered_map>
#include <list>

                /* Common config */
#define MAX_DISTANCE_NAME "MaxDistance"

      /* qtapplication config */
#define LOS_OP_AOI_IN "AOICenterPointSplit"
#define LOS_OP_NFZ_IN "NFZSplit"
#define LOS_OP_ELEV_IN "ElevationDataSet"
#define LOS_OP_WP_OUT "WaypointRead"
#define LOS_OP_TARGET_IN "TargetRead"
#define LOS_OP_OBSERVER_IN "ObserverRead"
#define LOS_OP_TARGET_LOS_OUT "TlosSplit"
#define LOS_OP_VIEWSHED_IN "ViewshedRead"
#define LOS_OP_VIEWSHED_OUT "ViewshedSplit"

#define TOUR_OP_WP_IN "WaypointSplit"
#define TOUR_OP_TOUR_OUT "WaypointTourRead"

#define PATH_OP_WP_TOUR_IN "WaypointTourSplit"
#define PATH_OP_ROUTE_OUT "RouteRead"

#define BENCHMARK_LOS_POINT_IN "LosPointSplit"
#define BENCHMARK_TOUR_POINT_IN "WaypointTourSplit"
#define BENCHMARK_ROUTE_IN "RouteSplit"

#define VIEW_WAYPOINT_LOOK_LINE "WaypointLookLineSwitch"
#define VIEW_WAYPOINT_YAW_LOS_LINE "WaypointYawLosLineSwitch"
#define VIEW_TOUR "Tour"
#define VIEW_ROUTE "Route"

            /* losWaypointOperator config */
// on 0 => throw EngineException in function
// on 1 => return false in function, place placeholder waypoint on AOI cell coordinates
// on 2 => return false in function, call createWaypointPos again in the hopes of eventually finding a point
#define WAYPOINT_GEN_FAILED_BEHAVIOR 2

// on 0 => On waypoint generation, waypoints will be placed with no consideration of placement of other waypoints
// on 1 => On waypoint generation, a new waypoint will only be placed if it has LOS of another waypoint which does not intersect with a NFZ
#define ONLY_CREATE_WAYPOINTS_WITH_BEELINE false

// on 0 => No corner waypoints will be placed
// on 1 => 1 waypoint will be placed just outside each corner of every NFZ polygon
#define CREATE_NAVPOINTS true

// on 0 => AOI points are placed on the same Z-value as its corresponding AOI cell
// on 1 => all AOI points are placed just above the elevation surface
#define SET_AOI_POINTS_TO_GROUND_LEVEL true

constexpr int NUM_SEEDS = 20;
constexpr int RND_SEEDS[] = {81525, 68377, 93123, 17921, 3414, 83168, 89149, 21223, 76292, 87537,
                             91912, 3320, 28985, 12020, 7162, 9891, 15735, 56480, 90916, 94004 };

// amount of geodata pixels per meter (m)
constexpr auto PIXEL_SIZE = 1;
constexpr auto PIXEL_THRESHOLD_LINEAR_INSPECTION = 100;
constexpr auto WP_PLACE_HEIGHT_ABOVE_Z_MIN = 10;
constexpr auto AOI_POINT_PLACE_ABOVE_GROUND_ELEV = 1;
constexpr auto POINT_COLLISION_MARGIN = 0.00005;
constexpr auto GEOFENCE_BUFFER_ROUTING = 0.00001;
constexpr auto M_MARGIN_NAVPOINT = 10;
constexpr auto RASTER_ELEV_DATA_POINT_SIZE = 5;
// Observe. FOV:s larger than 180 degrees not supported
constexpr int UAS_FOV_H = 120;
constexpr int ASPECT_RATIO_H = 16;
constexpr int ASPECT_RATIO_V = 9;
constexpr auto PITCH_MIN = -90;
constexpr auto PITCH_MAX = 90;
constexpr auto YAW_MIN = 0;
constexpr auto YAW_MAX = 359;
constexpr auto PITCH_LOW_NAME = "PitchLow";
constexpr auto PITCH_HIGH_NAME = "PitchHigh";
constexpr auto YAW_LOW_NAME = "YawLow";
constexpr auto YAW_HIGH_NAME = "YawHigh";
constexpr auto AOI_CELL_NAME = "AoiCellId";
constexpr auto WAYPOINT_NAME = "WaypointId";
// If set to false, then the waypoint does not contribute to any LOS and its only purpose is to assist with pathfinding
constexpr auto IS_LOS_WAYPOINT_NAME = "IsLosWaypoint";
constexpr auto WAYPOINT_CALC_TIME_NAME = "WaypointCalculationTime";

            /* waypointCamDirOp config */
constexpr auto LINE_LENGTH_PROPERTY_NAME = "lineLength";
constexpr auto CAM_FEAT_TYPE = "CameraFeatureType";
constexpr auto CAM_LINE_YAW_MARG_1 = 0;
constexpr auto CAM_LINE_YAW_MARG_2 = 1;
constexpr auto CAM_LINE_YAW_LOS_1 = 2;
constexpr auto CAM_LINE_YAW_LOS_2 = 3;
constexpr auto CAM_LINE_PITCH = 4;
constexpr auto CAM_POLY = 5;
constexpr auto DEG_TO_RAD = 0.0174533;
constexpr auto LOS_YAW_MAX_NAME = "YawLowLos";
constexpr auto LOS_YAW_MIN_NAME = "YawHighLos";

        /* tourOperator config  */
constexpr auto TOUR_ORDER_NAME = "tourOrderId";
constexpr auto DEBUG_WP_ID = 0;

        /* pathOperator config */
constexpr auto REST_API_URL = "http://localhost:5000";
constexpr auto REST_ROUTING_ENDPOINT = "/routing/simple-bvlos";
constexpr auto ZONE_BOTTOM_ALTITUDE = 0;
constexpr auto ZONE_TOP_ALTITUDE = 1000000;
constexpr auto ROUTE_FOUND_NAME = "routeFound";
constexpr auto POINT_EQUALITY_PRECISION = 10000;
constexpr auto LEG_WAYPOINT_1_NAME = "firstWaypointId";
constexpr auto LEG_WAYPOINT_2_NAME = "secondWaypointId";
constexpr auto LEG_DISTANCE_NAME = "legDistance";

/* benchmarkModule config */
constexpr auto BENCHMARK_DIR_OUTPUT = "benchmark.csv";
//constexpr auto BENCHMARK_MODE = true;

#endif