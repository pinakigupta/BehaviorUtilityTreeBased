// shared_enums.h   --  enumerations for shared data entities in the Beacon/Corona systems
// 
// Copyright (c) 2016 General Motors Company
//
// Developed by Michael A. Losh (GM)
//
// Last modification: 2016-09-28 by M. Losh

#ifndef __SHARED_ENUMS__H__
#define __SHARED_ENUMS__H__

// ///
// Enumerations related to Advanced Navigation Spatial FOG Dual GPS+IMU unit
// ///

// GPS "Fix" type -- how the position was determined, esp. what kind of external data was applied (if any)
typedef enum {
	GPS_FIX_NONE            = 0,    // no position
	GPS_FIX_2D              = 1,    // single-point horizontal position only, alt may be very inacurate, no external reference data used
	GPS_FIX_3D              = 2,    // single-point position, no external reference data used
	GPS_FIX_SBAS            = 3,    // corrected by SBAS data, e.g. using WAAS data from satellite
	GPS_FIX_DIFFERENTIAL    = 4,    // local basestation corrections applied (e.g. RTCM?)
	GPS_FIX_OMNISTAR        = 5,    // Omnistar or Starfire commercial reference data applied to position fix
	GPS_FIX_RTK_FLOAT       = 6,    // RTK "float" fix (e.g. with Trimble RTX?)
	GPS_FIX_RTK_FIXED       = 8     // Basestation mode???
} gps_fix_type_enum;


// GPS Health Alert/Error flag bit positions; if any bit is set, it indicates a problem
typedef enum {
	GPS_ALARM_FLAG_BIT_SYSTEM_FAILURE               = 0x0001,
	GPS_ALARM_FLAG_BIT_ACCELEROMETER_SENSOR_FAILURE = 0x0002,
	GPS_ALARM_FLAG_BIT_GYROSCOPE_SENSOR_FAILURE     = 0x0004,
	GPS_ALARM_FLAG_BIT_MAGNETOMETER_SENSOR_FAILURE  = 0x0008,
	GPS_ALARM_FLAG_BIT_PRESSURE_SENSOR_FAILURE      = 0x0010,
	GPS_ALARM_FLAG_BIT_GNSS_FAILURE                 = 0x0020,
	GPS_ALARM_FLAG_BIT_ACCELEROMETER_OVER_RANGE     = 0x0040,
	GPS_ALARM_FLAG_BIT_GYROSCOPE_OVER_RANGE         = 0x0080,
	GPS_ALARM_FLAG_BIT_MAGNETOMETER_OVER_RANGE      = 0x0100,
	GPS_ALARM_FLAG_BIT_PRESSURE_OVER_RANGE          = 0x0200,
	GPS_ALARM_FLAG_BIT_MINIMUM_TEMPERATURE_ALARM    = 0x0400,
	GPS_ALARM_FLAG_BIT_MAXIMUM_TEMPERATURE_ALARM    = 0x0800,
	GPS_ALARM_FLAG_BIT_LOW_VOLTAGE_ALARM            = 0x1000,
	GPS_ALARM_FLAG_BIT_HIGH_VOLTAGE_ALARM           = 0x2000,
	GPS_ALARM_FLAG_BIT_GNSS_ANTENNA_DISCONNECTED    = 0x4000,
	GPS_ALARM_FLAG_BIT_SERIAL_PORT_OVERFLOW_ALARM   = 0x8000,
} gps_alarm_flag_bits;


// GPS filter status flag bit positions; for single-bit fields, bit is set if the condition is true 
// note: the GPS fix type status is a 3-bit field that can be extracted by bitfield in data structure union
typedef enum {
	GPS_FILTER_FLAG_BIT_ORIENTATION_FILTER_INITIALISED  = 0x0001,
	GPS_FILTER_FLAG_BIT_INS_FILTER_INITIALISED          = 0x0002,
	GPS_FILTER_FLAG_BIT_HEADING_INITIALISED             = 0x0004,
	GPS_FILTER_FLAG_BIT_UTC_TIME_INITIALISED            = 0x0008,
	GPS_FILTER_FLAG_BIT_EVENT1_FLAG                     = 0x0080,
	GPS_FILTER_FLAG_BIT_EVENT2_FLAG                     = 0x0100,
	GPS_FILTER_FLAG_BIT_INTERNAL_GNSS_ENABLED           = 0x0200,
	GPS_FILTER_FLAG_BIT_DUAL_ANTENNA_HEADING_ACTIVE     = 0x0400,
	GPS_FILTER_FLAG_BIT_VELOCITY_HEADING_ENABLED        = 0x0800,
	GPS_FILTER_FLAG_BIT_ATMOSPHERIC_ALTITUDE_ENABLED    = 0x1000,
	GPS_FILTER_FLAG_BIT_EXTERNAL_POSITION_ACTIVE        = 0x2000,
	GPS_FILTER_FLAG_BIT_EXTERNAL_VELOCITY_ACTIVE        = 0x4000,
	GPS_FILTER_FLAG_BIT_EXTERNAL_HEADING_ACTIVE         = 0x8000,
} gps_filter_flag_bits;

/***** Enumerations from Data dictionary - start **********/
typedef enum UID_entity_hint_enum{
	UNCLASSIFIED_ENTITY 				= 0,	/*UID for an object entity that is not classified*/
	SPECIAL_AREA_ENTITY 				= 1,	/*UID for place that is specially inserted into map but may not represent a physical object*/
	POLE_TYPE_OBJECT_ENTITY 			= 2,	/*UID for object is fairly compact in footprint (e.g. < 1.0 m^2) but has signficant height*/
	FENCE_TYPE_OBJECT_ENTITY 			= 3,	/*UID for object that is extended but no significant thickness (treat boundary as open polyline)*/
	VEGETATION_TYPE_OBJECT_ENTITY 		= 4,	/*UID for area of non-drivable vegetation, may be irregular in shape*/
	STRUCTURE_TYPE_OBJECT_ENTITY 		= 5,	/*UID for any structure with significant footprint*/
	BRIDGE_TYPE_OBJECT_ENTITY 			= 6,	/*UID for any structure that extends over the roadway, height is vertical clearance*/
	GATE_TYPE_OBJECT_ENTITY 			= 7,	/*UID for an object that may, at certain times at least, block the roadway*/
	HOLE_TYPE_OBJECT_ENTITY	 			= 8,	/*UID for a localized spot that is measurably lower than normal roadway surface*/
	//(reserved) = 9,
	SENSED_OBSTACLE_ENTITY 				= 0xA,	/*UID for an object/obstacle sensed by onboard equipment or known via V2V*/
	//(reserved) = 0xB,
	//(reserved) = 0xC,
	INTERSECTION_ENTITY 				= 0xD,	/*UID for a mapped intersection*/
	EDGE_ENTITY 						= 0xE,	/*UID for an Edge entity, typically a lane marking/road edge*/
	LANE_SEGMENT_ENTITY 				= 0xF	/*UID for a mapped lane segment*/
}eEntityHint;

typedef enum route_step_task_enum{
	SHUTDOWN_TASK 		= 0,	/*Park at (or near) segment end and shut down vehicle*/
	STANDBY_TASK 		= 1,	/*Park at (or near) segment end, idle*/
	PROCEED_TASK 		= 2,	/*Move toward target segment after completing current segment*/
	CHANGE_LANES_TASK 	= 3,	/*Maneuver into an adjacent lane (when safe)*/
	MERGE_TASK			= 4,	/*Blend into traffic near some mid-point of lane segment*/
	EXIT_TASK			= 5,	/*Leave some-mid-point of a lane segment to take fork or turn*/
	TURN_TASK 			= 6,	/*Make a smooth turning maneuver from the end of one segment to the start of another*/
	REVERSE_TASK		= 7,	/*Back up a to a specified distance along the segment*/
	REVERSE_TURN_TASK	= 8,		/*Back up and turn to align to a target segment*/
	PROCEED_AFTER_LANE_CHANGE_TASK = 9,
	PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK = 10,
	PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK = 11,
	PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK = 12,
    PROCEED_THROUGH_PARKING_AISLE_TASK = 13
}eRouteStepTask;

typedef enum reroute_type_enum{
	NO_REROUTE                  = 0,
	OPTIMAL_REROUTE            = 1, // Most optimal reroute to destination from the A* planner. So if we are in our original planned path and request optimal path it should again give back the original path (assuming it was optimal to begin with).
	SAFE_REROUTE            = 2, // Reroute to a safe state ( may not be destination )
	STRAIGHT_ROUTE = 3, //  Straight ahead (using primary segments) to the current segment group for at least x m( =100) and optimal path there after.
	LEFT_LANE_CHANGE_ROUTE          = 4, // immediate left lanes and its primary segments for at least x m(=100) and optimal path there after.
	RIGHT_LANE_CHANGE_ROUTE   = 5, // immediate left lanes and its primary segments for at least x m(=100) and optimal path there after.   ORIGINAL_ROUTE_REQUEST = 6 // If we are currently in a reroute and we want to get back to our original route. We may not be able to just issue another optimal path as our original path may not be optimal.
	LANE_CHANGE_ROUTE   = 6, // immediate opportunistic lane change. Whatever PathPlanner thinks is the best. With double lane change NOT allowed.
	LANE_CHANGE_ROUTE_DLCX_ALLWD   = 7 , // immediate opportunistic lane change. Whatever PathPlanner thinks is the best. With double lane change allowed.
	LANE_CHANGE_ROUTE_DLCX_RQRD   = 8 , // immediate opportunistic lane change. Whatever With double lane change back to the same lane required/forced.
	OPPOSING_LANE_CHANGE_ROUTE   = 9 // immediate opportunistic lane change. Whatever PathPlanner thinks is the best. With double lane change allowed.

}ePathPlanRerouteType;

typedef enum right_of_way_rule_enum{
	ASSUME_RIGHT_OF_WAY_RULE 				= 0,	/*Only yield for collision avoidance*/
	YIELD_UNTIL_CLEAR_RULE 					= 1,	/*Yield to cross-traffic and conflicting oncoming traffic*/
	STOP_AND_YIELD_UNTIL_CLEAR_RULE 		= 2,	/*Yield to cross-traffic and conflicting oncoming traffic, after making stop*/
	HANDLE_MULTIWAY_STOP_RULE 				= 3,	/*Stop and proceed after earlier arriving vehicles, following multiway-stop etiquette*/
	ASSUME_RIGHT_OF_WAY_AFTER_SIGNAL_RULE	= 4,	/*Stop if signal is or soon red; after green, only yield for collision avoidance*/
	YIELD_UNTIL_CLEAR_AFTER_SIGNAL_RULE		= 5,	/*Stop if signal is or soon red; after green, yield to conflicting traffic*/
	RIGHT_ON_RED_RULE			 			= 6,	/*Yield to cross-traffic and conflicting oncoming traffic after making stop at red light; on green, assume right-of-way*/
	MERGE_AT_TRAFFIC_SPEED_RULE			 	= 7,    /*Merge into exisiting traffic flow, matching speed if slower than intended*/
	EXIT_AT_TRAFFIC_SPEED_RULE 				= 8,
	ASSUME_RIGHT_OF_WAY_AFTER_RAIL_CROSSING_CLEAR_RULE = 9, /*Stop if rail crossing has a train present, or crossing barrier present, or flashing red light */
	CHANGE_LANES_RULE = 10,
	OBEY_POSTED_SIGN_RULE = 11,
	YIELD_UNTIL_CREEP_DISTANCE_REACHED = 12, 		/*Creep forward, however, stop (to yield) anytime a vehicle is seen in the way*/
	FOLLOW_INTERSECTION_RULE = 13,
	YIELD_FOR_PEDESTRIAN_RULE = 14,
	OBEY_SIGN_IF_POSTED_RULE = 15
}eRightOfWayRule;

typedef enum direction_enum{
	STRAIGHT_DIRECTION 	     = 0,	/*Proceed straight ahead, following lane curvature*/
	BEAR_LEFT 			     = 1,	/*Take the left-going branch at a fork in the road*/
	TURN_LEFT 			     = 2,	/*Turn onto another road, drive, or parking area*/
	BEAR_RIGHT 			     = 3,	/*Take the left-going branch at a fork in the road*/
	TURN_RIGHT 			     = 4,	/*Turn onto another road, drive, or parking area*/
	REVERSE_STRAIGHT	     = 5,	/*Back up along segment*/
	REVERSE_LEFT		     = 6,	/*Turn steering to left during reverse*/
	REVERSE_RIGHT		     = 7,	/*Turn steering to right during reverse*/
	BEAR_LEFT_OPPOSING_LANE	 = 8,	/*Go to an oopposing lane on left*/
	BEAR_RIGHT_OPPOSING_LANE = 9,	/*Go to an oopposing lane on left*/
	STRAIGHT_OPPOSING_DIRECTION =10 /*When going opposite direction on a straight lane, applicable transiently */
}eDirection;

typedef enum lane_flow_restriction_type_enum{
	NO_RESTRICTION                        = 0,
	STOP_SIGN_RESTRICTION                 = 1,
	YIELD_SIGN_RESTRICTION                = 2,
	PEDESTRIAN_CROSSING_RESTRICTION       = 3,
	NO_STOPPING_RESTRICTION               = 4,
	CONSTRUCTION_ZONE_RESTRICTION         = 5,
	SHARED_LANE_RESTRICTION               = 6
}eLaneFlowRestrictionType;

typedef enum master_state_enum{
	OFF_MASTER_STATE 				= 0,	/*Base vehicle is not started*/
	IDLE_MASTER_STATE 				= 1,	/*Base vehicle is started, but in park and not moving*/
	MANUAL_MASTER_STATE 			= 2,	/*Base vehicle is started, but in a non-park gear*/
	AUTONOMOUS_MASTER_STATE 		= 3,	/*Vehicle is controlled by autonomous systems*/
	AUTONOMOUS_ESTOP_MASTER_STATE 	= 4		/*Vehicle is autonomous system but attempting to handle emergency by stopping*/
}eMasterState;

typedef enum state_goal_enum{
	NO_STATE_GOAL 				= 0,	/*No goal is defined*/
	GO_MANUAL_STATE_GOAL 		= 1,	/*Hand over control to manual driver*/
	AUTON_PARK_STATE_GOAL		= 2,	/*Vehicle should autonomously maneuver to parking space and prepare to shut down*/
	AUTON_RETRIEVE_STATE_GOAL	= 3,	/*Vehicle should autonomously start up and maneuver to user pickup location*/
	AUTON_ESTOP_STATE_GOAL 		= 4,	/*Vehicle should slow down promptly and hold brakes; after pause, shift to park and then shutdown*/
	AUTON_DRIVE_STATE_GOAL 		= 5		/*Vehicle should autonomously drive to destination, stop, and hold brakes; after short pause, shift to park and idle*/
}eStateGoal;


typedef enum active_maneuver_enum{
	NO_MANEUVER 							= 0,	/*No autonomous maneuver is active*/
	LANE_KEEPING_MANEUVER 					= 1,	/*Follow lanes as defined by route*/
	LANE_FOLLOWING_AND_CHANGING_MANEUVER 	= 2,	/*Change lanes along route opportunistically*/
	INTERSECTION_DRIVING_MANEUVER 			= 3,	/*Proceed through intersection according to rules of the road*/
	MERGE_MANEUVER 							= 4,	/*Merge into a lane matching speed to traffic in a sufficient gap*/
	MANEUVER_EXIT 							= 5,	/*Pull out of lane and manage deceleration*/
	U_TURN_MANEUVER 						= 6		/*Perform a u-turn maneuver (usually a left turn through a median)*/
}eActiveManeuver;

typedef enum lane_change_feedback_enum{
	REJECTED 		= 1,  /* trajectory planner rejected the lane change command from behavior before commiting to it */
	ABORTED 		= 2,  /* trajectory planner aborted the lane change command from behavior after commiting to it previously */
	INACTIVE 		= 3,  /* lane change is inactive */
        INITIATED 		= 4,   /* trajectory planner initiated the lane change without any lane change command from behavior*/
        COMMITED_TO_LEFT	= 5,  /* trajectory planner is following the left lane change command from behavior */
        COMMITED_TO_RIGHT	= 6   /* trajectory planner is following the right lane change command from behavior */
}eLaneChangeFeedback;

typedef enum additional_space_request_enum{
	NONE 			= 0,  /* no addtional data for adjacent lanes is requested */
	ALL 			= 1,  /* additional data from both left and right is requested  */
	LEFT			= 2,  /* additional data from left is requested  */
	RIGHT			= 3   /* additional data from right is requested */
}eAdditionalSpaceRequest;


typedef enum lane_intention_enum{
        LANE_CHANGE_IS_PROHIBITED       = 0,	/*No autonomous maneuver is active*/
        LANE_CHANGE_FOR_EMERGENCY_ONLY 	= 1,	/*Only use lane/shoulder in emergency*/
        LANE_CHANGE_IS_ALLOWED 		= 2,	/*OK to change lanes opportunistically*/
        LANE_CHANGE_IS_DESIRED 		= 3,	/*Behavior planner sees good opportunity to change lanes*/
        LANE_CHANGE_IS_RECOMMENDED      = 4,	/*Route requires vehicle to be in a different lane within some moderate priority threshold time/distance*/
        LANE_CHANGE_IS_URGENT 		= 5,	/*Route requires vehicle to be in a different lane within some urgent priority threshold time/distance*/
        LANE_CHANGE_IS_NOT_DESIRED      = 6,	/*Behavior planner think lane change is not desired due to static or dynamic reasons*/
        LANE_CHANGE_IN_PROGRESS		= 7,	/*Behavior planner relinquishes control to trajectory planner at this point, gave it the correct gap, and not in a good position to analyze lane gaps*/
     	LANE_CHANGE_NEAR_COMPLETION     = 8 ,    /*Lane change in progress +  localization has already decided it crossed the lane boundary towards the target state */
     	LANE_CHANGE_OPPOSING     = 9     /*Lane change (not yey in progress) to the opposing lane */
}eLaneIntention;


typedef enum road_class_enum{
	MARKED_SUBURBAN_ROADTYPE 	= 0,	/*Low to moderate speeds, should have lanemarks at least for road center*/
	INTERSTATE_FREEWAY_ROADTYPE	= 1,	/*High speed, controlled access road, usually two or more lanes each way*/
	OTHER_FREEWAY_ROADTYPE		= 2,	/*High speed, controlled access road, usually two or more lanes each way*/
	UNMARKED_SUBURBAN_ROADTYPE 	= 3,	/*Low to moderate speeds, lanemarkings mostly absent*/
	HIGHWAY_ONRAMP_ROADTYPE		= 4,	/*Onramp to enter controlled-access freeway*/
	HIGHWAY_OFFRAMP_ROADTYPE	= 5,	/*Offramp from a controlled-access freeway*/
	MARKED_URBAN_ROADTYPE		= 6,	/*Low to moderate speeds in densely-build environment, lanemarkings should be present*/
	UNMARKED_URBAN_ROADTYPE		= 7,	/*Low speed, perhaps-one way streets or alleys*/
	ARTERIAL_ROADTYPE			= 8,	/*Moderate speed multi-lane road, lanes markings should exist*/
	RURAL_HIGHWAY_ROADTYPE		= 9,	/*Moderate to higher speed road, usually single lane each way, pass in opposing lane*/
	PARKING_AISLE_ROADTYPE		= 10,	/*Lane through a parking lot*/
	GRAVEL_ROADTYPE			 	= 11,	/*Road is paved with gravel*/
	UNPAVED_ROADTYPE			= 12,	/*Road may be dirt path or lawn*/
	INVALID_ROADTYPE 			= 255	/*Data is not valid or trustworthy for some reason*/
}eRoadClass;


typedef enum flow_restriction_enum{
	NO_FLOW_RESTRICTION 						= 0,	/*Shouldn't need to stop or yield at end of this segment if continuing on*/
	YIELD_FLOW_RESTRICTION 						= 1,	/*Segment ends at intersection with a posted yield sign*/
	PEDESTRIAN_CROSSING_FLOW_RESTRICTION        = 2,
	SIMPLE_STOP_FLOW_RESTRICTION 				= 3,	/*Stop for a pedestrian crosswalk or intersection with non-stopping traffic*/
	MULTI_WAY_STOP_FLOW_RESTRICTION 			= 4,	/*Three-way or Four-way stop sign intersection*/
	SIGNALIZED_STOP_FLOW_RESTRICTION			= 5,	/*Traffic light controlled intersection*/
	RAILROAD_CROSSING_BARRIER_FLOW_RESTRICTION 	= 6,	/*Intersection with railroad, with a physical crossing gate barrier*/
	RAILROAD_CROSSING_LIGHTS_FLOW_RESTRICTION 	= 7,	/*Intersection with railroad, lights only (no barrier)*/
	I2V_SIGNALIZED_STOP_FLOW_RESTRICTION		= 8,	/*Traffic light controlled intersection that broadcasts signal phase and timing via I2V*/
	NO_RIGHT_ON_RED_SIGNAL_FLOW_RESTRICTION		= 9,		/*Traffic light controlled intersection with no right on red sign*/
	CONSTRUCTION_ZONE_FLOW_RESTRICTION          = 10,
	MUST_MERGE_FLOW_RESTRICTION                 = 11,
	BUS_STOP_FLOW_RESTRICTION                   = 12,
	SCHOOL_ZONE_FLOW_RESTRICTION                = 13,
	SPEED_BUMP_FLOW_RESTRICTION                 = 14,
	DO_NOT_STOP_HERE_FLOW_RESTRICTION           = 15,
	SHARED_LANE_FLOW_RESTRICTION                = 16,   /* Two-way traffic without center line */
	RAILROAD_CROSSING_SIGN_FLOW_RESTRICTION     = 17,    /* No barrier, no flashing lights */
    POSSIBLE_UNKNOWN_FLOW_RESTRICTION                = 18,
    POSSIBLE_YIELD_OR_STOP_FLOW_RESTRICTION          = 19,
    POSSIBLE_STOP_OR_TRAFFIC_SIGNAL_FLOW_RESTRICTION = 20,
    POSSIBLE_NO_FLOW_RESTRICTION                     = 21,
	CREEP_AFTER_STOP_OR_YIELD_RESTRICTION      = 22
    
}eFlowRestriction;

typedef enum laneseg_attribute_mask{
	ENDS_AT_FULL_RIGHT_OF_WAY 				= 0x00000001,	/*Shouldn't need to stop or yield at end of this segment if continuing on*/
	ENTERS_INTERSECTION 					= 0x00000100,	/*Lane segment ends at entrance to intersection, may have cross-traffic just beyond this segment*/
	TRAVERSES_INTERSECTION 					= 0x00000200,	/*Segment begins at a stop bar or equivalent and traverses into/through the intersection (might be useful if this is a pre-mapped lane through intersection, some/most? intersections may not have these explicitly mapped out)*/
	ENDS_AT_DESTINATION 					= 0x00000400,	/*Possible destination of autonomous driving, e.g. parking spot or valet drop-off/pick-up location*/
	NO_TURNS 								= 0x00000800,	/*Lane that enters intersection is marked for NO turns (straight only)*/
	LEFT_TURN_ONLY 							= 0x00001000,	/*Lane is marked to allow left turns only*/
	RIGHT_TURN_ONLY 						= 0x00002000,	/*Ditto for right*/
	ALTITUDE_MATTERS			 			= 0x00004000,	/*Other segments may pass under or over this segment*/
	OPPO_LANE_MAY_USE 						= 0x00008000,	/*Traffic from opposing direction may use for passing, etc. (dashed yellow line)*/
	IGNORE_LANEMARKINGS 					= 0x00010000,	/*System should not trust lane markings for localization, steering, or situation awareness*/
	HAS_ENTRIES 							= 0x00020000,	/*driveways, alleys, etc. enter directly to this lane or an adjacent lane parallel lane*/
	TURNED_IN_PARKED_CARS_EXPECTED_LEFT 	= 0x00040000,	/*The lane/aisle has 90-degree or angled parking along the left edge*/
	TURNED_IN_PARKED_CARS_EXPECTED_RIGHT 	= 0x00080000,	/*Ditto for right*/
	PARALLEL_PARKED_CARS_EXPECTED_LEFT 		= 0x00100000,	/*The lane/aisle has parking spots or is likely to have parked cars along the left edge – Ditto for right*/
	PARALLEL_PARKED_CARS_EXPECTED_RIGHT 	= 0x00200000,	/*Ditto for right*/
	DEPARTS_INTERSECTION 					= 0x00400000,	/*Segment that is completely past the intersection*/
	SUCCESSOR_IS_INTERSECTION 				= 0x00800000,	/*True if there is an intersection after this lane segment (Redundant to ENTERS_INTERSECTION?)*/
	PATH_CURVE_VALID 						= 0x01000000,	/*True if the curve model coefficients are accurate for nominal path (no more than 0.1 meters divergence over length of lane segment??)*/
	NO_LEFT_ALLOWED 						= 0x02000000,	/*Only straight or right turns allowed*/
	NO_RIGHT_ALLOWED 						= 0x04000000,	/*Only straight or left turns allowed*/
	NO_RIGHT_ON_RED			 				= 0x08000000,	/*Cannot turn right at intersection during a red light signal phase*/
	//(reserved) = 0x10000000,
	//(reserved) = 0x20000000,
	//(reserved) = 0x40000000,
	//(reserved) = 0x80000000
}eLaneSegAttributeMask;


typedef enum cross_traffic_type_enum{
	UNKNOWN_CROSS_TRAFFIC 			= 0,	/*Don't know yet, or data is conflicting*/
	NO_CROSS_TRAFFIC 				= 1,
	CROSS_TRAFFIC_FROM_LEFT 		= 2,
	CROSS_TRAFFIC_FROM_RIGHT 		= 3,
	CROSS_TRAFFIC_FROM_BOTH_SIDES 	= 4,
	QUICK_ENTRY_FROM_LEFT 			= 5,	/*Merge/onramp with relatively high approach angle (not parallel)*/
	QUICK_ENTRY_FROM_RIGHT 			= 6
}eCrossTrafficType;

typedef enum connection_type_enum{
	UNKNOWN_CONNECTION 					= 0,	/*Don't know yet, or data is conflicting*/
	DOWNSTREAM_CONNECTION 				= 1,
	UPSTREAM_CONNECTION 				= 2,
	PARALLEL_LEFT_ADJ_CONNECTION 		= 3,
	PARALLEL_RIGHT_ADJ_CONNECTION 		= 4,
	PARALLEL_NON_ADJ_CONNECTION 		= 5,
	PARALLEL_OPPOSING_CONNECTION 		= 6,
	INTERPOLATED_JUNCTION_CONNECTION 	= 7,	/*OBSOLETE? There is gap between given segment and "other", which may be defined by explicit point list*/
    OPPOSING_SHARED_LANE_CONNECTION     = 8,
    CENTER_SHARED_TURN_LANE_CONNECTION  = 9     // DO WE WANT THIS?
}eConnectionType;

typedef enum scenario_lane_edge_type_enum{
    DASHED_LANE_EDGE_TYPE               = 0,
    SHARED_LANE_EDGE_TYPE               = 1,
    SOLID_LANE_EDGE_TYPE                = 2
}eScenarioLaneEdgeType;

typedef enum edge_type_enum{
	UNKNOWN_EDGETYPE 					= 0,	/*Don't know yet, or data is conflicting*/
	DASHED_WHITE_EDGETYPE 				= 1,	/*Painted stripe/tape*/
	SOLID_WHITE_EDGETYPE 				= 2,
	SOLID_YELLOW_EDGETYPE 				= 3,
	DOUBLE_SOLID_YELLOW_EDGETYPE 		= 4,
	DOTS_EDGETYPE 						= 5,	/*Botts dots or similar marking*/
	UNPAVED_SHOULDER_EDGETYPE 			= 6,	/*Gravel/dirt at edge of roadway (paint maybe missing or hard to see)*/
	CURB_EDGETYPE 						= 7,	/*Raised curb > 2.5 cm high*/
	GUARD_RAIL_EDGETYPE 				= 8,
	WALL_EDGETYPE 						= 9,	/*Wall or barrier at least 30 cm high*/
	CONSTRUCTION_BARRICADE 				= 10,	/*Contiguous baricades or barrels for a construction zone*/
	DOUBLE_MY_SIDE_PASSING_EDGETYPE 	= 11,	/*Yellow stripes that allows host to pass by using opposing traffic lane*/
	DOUBLE_THEIR_SIDE_PASSING_EDGETYPE 	= 12,	/*Yellow stripes that allows opposing traffic to pass using host's lane*/
	DOUBLE_FOR_CENTER_TURN 				= 13,	/*Looks like DOUBLE_THEIR_SIDE_PASSING, but marks edge of shared (AKA "suicide") center turn lane*/
	BLUE_EDGETYPE 						= 14,	/*Painted stripe*/
	PARKING_EDGETYPE                    = 15,
	DRIVEWAY_EDGETYPE                   = 16,
	DASHED_YELLOW_EDGETYPE              = 17,
	DOUBLE_DASHED_YELLOW_EDGETYPE       = 18,
	DOUBLE_SOLID_WHITE_EDGETYPE         = 19,
	DOUBLE_DASHED_WHITE_EDGETYPE        = 20,
	IMPLIED_LANE_BOUNDARY_EDGETYPE      = 21,    /* Defined by center of road or by average width of on-street parking */
    NO_PARKING_EDGETYPE                 = 22
}eEdgeType;

typedef enum laneboundary_type_enum{
	UNDECIDED_LANEBOUNDARY_TYPE         = 0,
	SOLID_LANEBOUNDARY_TYPE             = 1,
	ROAD_EDGE_LANEBOUNDARY_TYPE         = 2,
	DASHED_LANEBOUNDARY_TYPE            = 3,
	DOUBLE_LINE_LANEBOUNDARY_TYPE       = 4
}eLaneBoundaryType;

typedef enum laneboundary_color_enum{
	WHITE_LANEBOUNDARY_COLOR            = 0,
	YELLOW_LANEBOUNDARY_COLOR           = 1,
	BLUE_LANEBOUNDARY_COLOR             = 2,
	NO_LANEBOUNDARY_COLOR               = 3
}eLaneBoundaryColor;

typedef enum intersection_type_enum{
	SIDE_STREET_INTERSECTION_TYPE 		= 0,	/*Side street or major driveway connects with another street but does not continue on itself*/
	CROSSROADS_INTERSECTION_TYPE 		= 1,	/*Two roadways cross each other where turns from one to the other are expected*/
	MERGE_INTERSECTION_TYPE 			= 2,	/*A branch brings in traffic that needs to merge at speed*/
	EXIT_INTERSECTION_TYPE 				= 3,	/*A new branch forks off main segment sequence with usually with reduced speed*/
	FORK_INTERSECTION_TYPE 				= 4,	/*Similar to exit, but the branches of fork do not change speeds from prior segment*/
	TRAFFIC_CIRCLE_INTERSECTION_TYPE 	= 5		/*I.E. roundabout*/
}eIntersectionType;


typedef enum related_object_class_enum{
	UNKNOWN_OBJECT_CLASS 			= 0,	/*Object is not classified*/
	SPECIAL_AREA_OBJECT_CLASS 		= 1,	/*Place that is specially inserted into map but may not represent a physical object*/
	POLE_TYPE_OBJECT_CLASS 			= 2,	/*Object is fairly compact in footprint (e.g. < 1.0 m^2) but has signficant height*/
	FENCE_TYPE_OBJECT_CLASS 		= 3,	/*Object is extended but no significant thickness (treat boundary as open polyline)*/
	VEGETATION_TYPE_OBJECT_CLASS 	= 4,	/*Object is area of non-drivable vegetation, may be irregular in shape*/
	STRUCTURE_TYPE_OBJECT_CLASS 	= 5,	/*Any structure with significant footprint*/
	BRIDGE_TYPE_OBJECT_CLASS 		= 6,	/*Any structure that extends over the roadway, height is vertical clearance*/
	GATE_TYPE_OBJECT_CLASS 			= 7,	/*An object that may, at certain times at least, block the roadway*/
	HOLE_TYPE_OBJECT_CLASS 			= 8,	/*A localized spot that is measurably lower than normal roadway surface*/
	TRAFFIC_CONTROL_DEVICE_CLASS    = 9     /*A non-sign device for traffic control such as traffic light, RR active light, etc.*/
}eRelatedObjectClass;

typedef enum special_area_enum{
	UNDEFINED_AREA_OBJECT 			= 0,	/*Area that is not further classified*/
	UNMARKED_PARKING_SPACE_OBJECT 	= 1,	/*Mapped space for parking that does not have guidelines*/
	MARKED_PARKING_SPACE_OBJECT 	= 2,	/*Mapped space for parking that does with guidelines*/
	PASSENGER_DROPOFF_AREA_OBJECT 	= 3,	/*Mapped location where vehicle will stop to drop off passengers*/
	PASSENGER_PICKUP_AREA_OBJECT 	= 4,	/*Mapped location where vehicle will stop to pick up passengers*/
	VEHICLE_STANDBY_AREA_OBJECT 	= 5,	/*Mapped location where vehicle will idle when not busy*/
	ROUTE_DESTINATION_AREA_OBJECT 	= 6		/*Mapped location that the navigation/route planner function may use as a destination*/
}eSpecialArea;

typedef enum traffic_control_device_subtype_enum{
	UNDEFINED_TCD_OBJECT 			        = 0,	/*not further classified*/
	STD_TRAF_SIG_TCD_OBJECT 	            = 7,	/*Standard Red Yellow Green traffic light*/
	FLASHING_RED_TRAF_SIG_TCD_OBJECT        = 9,    /*Red flashing light only stop signal*/
	LEFT_GREEN_ARROW_TRAF_SIG_TCD_OBJECT    = 36,   /*Green Arrow only toward left*/
	RIGHT_GREEN_ARROW_TRAF_SIG_TCD_OBJECT   = 68    /*Green Arrow only toward right*/
	//...
}eTrafficControlDeviceType;


typedef enum pole_subtype_enum{
	GENERAL_POLE_OBJECT 		= 0,	/*Object is not classified*/
	TREE_TRUNK_OBJECT 			= 1,
	UTILITY_POLE_OBJECT 		= 2,
	CONSTRUCTION_BARREL_OBJECT 	= 3,

	STOP_SIGN_OBJECT 						= 32,
	YIELD_SIGN_OBJECT 						= 33,
	PEDESTRIAN_CROSSING_SIGN_OBJECT 		= 34,
	RAILROAD_CROSSING_SIGN_OBJECT 			= 35,
	NO_PASSING_SIGN_OBJECT 					= 36,
	PASSING_ALLOWED_SIGN_OBJECT 			= 37,
	CURVE_AREAD_SIGN_OBJECT 				= 38,
	CURVE_SPEED_ADVISORY_SIGN_OBJECT 		= 39,
	MERGE_FROM_LEFT_AHEAD_SIGN_OBJECT 		= 40,
	MERGE_FROM_RIGHT_AHEAD_SIGN_OBJECT 		= 41,
	LANE_ENDS_MERGE_LEFT_AHEAD_SIGN_OBJECT 	= 42,
	LANE_ENDS_MERGE_RIGHT_AHEAD_SIGN_OBJECT = 43,
	NO_PARKING_SIGN_OBJECT 					= 44,
	NO_RIGHT_ON_RED_SIGN_OBJECT 			= 45,
	CONSTRUCTION_AHEAD_SIGN_OBJECT 			= 46,
	CONSTRUCTION_BEGINS_SIGN_OBJECT 		= 47,
	CONSTRUCTION_ENDS_SIGN_OBJECT 			= 48,


	SPEED_LIMIT_15_SIGN_OBJECT = 64,
	SPEED_LIMIT_20_SIGN_OBJECT = 65,
	SPEED_LIMIT_25_SIGN_OBJECT = 66,
	SPEED_LIMIT_30_SIGN_OBJECT = 67,
	SPEED_LIMIT_35_SIGN_OBJECT = 68,
	SPEED_LIMIT_40_SIGN_OBJECT = 69,
	SPEED_LIMIT_45_SIGN_OBJECT = 70,
	SPEED_LIMIT_50_SIGN_OBJECT = 71,
	SPEED_LIMIT_55_SIGN_OBJECT = 72,
	SPEED_LIMIT_60_SIGN_OBJECT = 73,
	SPEED_LIMIT_65_SIGN_OBJECT = 74,
	SPEED_LIMIT_70_SIGN_OBJECT = 75,
    
    NO_SIGN_OBJECT_PRESENT = 255
}ePoleSubType;

typedef enum fence_subtype_enum{
	GENERAL_FENCE_OBJECT 			= 0,	/*Object is not classified*/
	BORDER_FENCE_OBJECT 			= 1,
	HEDGE_OBJECT 					= 2,
	BERM_OBJECT 					= 3,
	WALL_OBJECT 					= 4,
	CONSTRUCTION_BARRICADE_OBJECT 	= 5
}eFenceSubType;


typedef enum vegetation_subtype_enum{
	GENERAL_VEGETATION_OBJECT 	= 0,	/*Object is not classified*/
	LOW_VEGETATION_OBJECT 		= 1,
	WOODS_OBJECT			 	= 2
}eVegetationSubType;

typedef enum structure_subtype_enum{
	UNDEFINED_STRUCTURE_OBJECT 				= 0,	/*Object is not classified*/
	BOOTH_STRUCTURE_OBJECT 					= 1,	/*Structure is small and may be close to roadway*/
	SMALL_BUILDING_OBJECT			 		= 2,	/*Structure is moderate in size*/
	LARGE_BUILDING_OBJECT 					= 3,	/*Structure is larger*/
	GARAGE_STRUCTURE_OBJECT			 		= 4,	/*Single-level structure where a driveway may enter*/
	MULTI_DECK_PARKING_STRUCTURE_OBJECT		= 5		/*Multi-level parking deck/garage structure*/
}eStructureSubType;

typedef enum bridge_subtype_enum{
	BRIDGE_OVERHEAD_OBJECT 	= 0,	/*Bridge that passes overhead, height is the vertical clearance.*/
	BRIDGE_OBJECT 			= 1,	/*Bridge used by a roadway to cross a low area, it may have some structure overhead, height is the vertical clearance.*/
	TUNNEL_OBJECT 			= 2,	/*Tunner or underpass that a roadway must use, height is the vertical clearance.*/
	DRAWBRIDGE_OBJECT 		= 3		/*Bridge over shipping lane where extra ship clearance is needed. Might be combined with a gate object.*/
}eBridgeSubType;

typedef enum gate_subtype_enum{
	GENERAL_GATE_OBJECT 	= 0,	/*Some kind of gate that might block a roadway or driveway*/
	AUTOMATED_GATE_OBJECT	= 1,	/*Gate that will open if approached slowly and a stop maneuver performed*/
	RAILROAD_GATE_OBJECT 	= 2
}eGateSubType;

typedef enum hole_subtype_enum{
	GENERAL_HOLE_OBJECT 	= 0,	/*Object is not classified*/
	POTHOLE_OBJECT 			= 1,	/*Low area of rough/missing pavement in roadway surface*/
	UTILITY_HOLE_OBJECT 	= 2		/*Hole usually closed by a metal cover in roadway surface*/
}eHoleSubType;

typedef enum connection_maneuver_enum{
	STRAIGHT_CONNECTION_MANEUVER 			= 0,
	LEFT_TURN_CONNECTION_MANEUVER 			= 1,
	RIGHT_TURN_CONNECTION_MANEUVER 			= 2,
	LEFT_MERGE_CONNECTION_MANEUVER			= 3,
	RIGHT_MERGE_CONNECTION_MANEUVER			= 4,
	LEFT_EXIT_CONNECTION_MANEUVER			= 5,
	RIGHT_EXIT_CONNECTION_MANEUVER			= 6,
	LEFT_FORK_CONNECTION_MANEUVER 			= 7,
	RIGHT_FORK_CONNECTION_MANEUVER 			= 8,
	TRAFFIC_CIRCLE_CONNECTION_MANEUVER 		= 9,
	LEFT_LANE_CHANGE_CONNECTION_MANEUVER 	= 10,
	RIGHT_LANE_CHANGE_CONNECTION_MANEUVER 	= 11,
	NO_ALLOWED_CONNECTION_MANEUVER 			= 12,
	LEFT_U_TURN_CONNECTION_MANEUVER         = 13,
	RIGHT_U_TURN_CONNECTION_MANEUVER        = 14,    //not likely
	EVASIVE_CONNECTION_MANEUVER             = 15,
	UNDEFINED_CONNECTION_MANEUVER           = 16
}eConnectionManeuver;

typedef enum connection_priority_enum{
	UNDEFINED_CONNECTION_PRIORITY 			= 0,
	UNIMPEDED_CONNECTION_PRIORITY 			= 1,    // direct connection that would not ever be expected to stop/yield at that point
	MAJOR_DOES_NOT_STOP_CONNECTION_PRIORITY = 2,    // no sign, this connection has full right-of-way 
	MAJOR_YIELD_ONLY_FOR_ONCOMMING_CONNECTION_PRIORITY	
	= 3,    // only yield to oncoming traffic (e.g. left turn where there is no stop/yield)
	MAJOR_CONTROLLED_CONNECTION_PRIORITY	= 4,    // multi-lane road, controlled by traffic light or all-way sign
	MINOR_YIELD_ASSUMED_CONNECTION_PRIORITY	= 5,    // no sign, but nature of intersection implies users of connection should yield
	MINOR_CONTROLLED_CONNECTION_PRIORITY	= 6,    // traffic light controlled minor road (single lane approach, other approaches multi-lane)
	MINOR_MUST_YIELD_CONNECTION_PRIORITY	= 7,    
	MINOR_MUST_STOP_CONNECTION_PRIORITY	    = 8    
}eConnectionPriority;

typedef enum behavior_type{
	ACC_BEHAVIOR,
	LANE_CHANGE_BEHAVIOR,
	LANE_KEEPING_BEHAVIOR,
	TURN_LEFT_BEHAVIOR,
	TURN_RIGHT_BEHAVIOR,
	TURN_ALONG_CURVY_ROAD_BEHAVIOR,
	YIELD_TO_PEDESTRIAN_BEHAVIOR,
	YIELD_TO_CROSS_TRAFFIC_BEHAVIOR,
	HANDLE_INTERSECTION_BEHAVIOR,
	STOP_AT_STOP_BAR_BEHAVIOR,
	ENTERING_ROUNDABOUT_BEHAVIOR,
	EXITING_ROUNDABOUT_BEHAVIOR,
	MERGING_BEHAVIOR,
	EXITING_FLOW_BEHAVIOR,
	STOP_AT_TRAFFIC_LIGHT_BEHAVIOR,
	UNKNOWN_BEHAVIOR,
	SAFETY_FIRST_BEHAVIOR,
	COLLISION_IMMINENT_BEHAVIOR,
	EVASIVE_LANE_CHANGE_BEHAVIOR

}eBehaviorType;

// Accel and Decel accelration threshold should be same
typedef enum behavior_accel_request{
	COMFORT_ACCEL_REQUEST_BEHAVIOR,
	AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR,
	SUPER_AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR,
	SLOW_ACCEL_REQUEST_BEHAVIOR,
	SUPER_SLOW_ACCEL_REQUEST_BEHAVIOR

}eBehaviorAccelRequest;

typedef enum scenario_enum{
	STANDBY_SCENARIO 							= 0,	/*System is waiting for further input (HMI command)*/
	MANUAL_SCENARIO 							= 1,	/*Vehicle is being driven manually*/
	AUTON_URBAN_STREET_SCENARIO 				= 2,
	AUTON_SUBURBAN_STREET_SCENARIO 				= 3,
	AUTON_RURAL_ROAD_SCENARIO			 		= 4,
	AUTON_FREEWAY_ROAD_SCENARIO			 		= 5,
	AUTON_FREEWAY_MERGE_SCENARIO			 	= 6,	/*Merge into faster road safely*/
	AUTON_FREEWAY_EXIT_SCENARIO			 		= 7,	/*Exit from faster road to lower-speed road*/
	AUTON_YIELD_INTERSECTION_SCENARIO 			= 8,
	AUTON_SIMPLE_STOP_INTERSECTION_SCENARIO 	= 9,
	AUTON_MULTIWAY_STOP_INTERSECTION_SCENARIO 	= 10,
	AUTON_TRAFFIC_SIGNAL_INTERSECTION_SCENARIO	= 11,
	AUTON_U_TURN_INTERSECTION_SCNARIO 			= 12,
	AUTON_ROUNDABOUT_INTERSECTION_SCNARIO 		= 13,
	AUTON_HEAVY_CONGESTION_SCENARIO			 	= 14,
	AUTON_CONSTRUCTION_ZONE_SCENARIO			= 15
}eScenario;

typedef enum traffic_cond_enum{
	UNKNOWN_TRAFFIC_COND 		= 0,	/*Traffic conditions unknown*/
	MINIMAL_TRAFFIC_COND 		= 1,	/*No or very few vehicles, no vehicle impede progress below posted limits*/
	LIGHT_TRAFFIC_COND 			= 0,	/*Some vehicle detected, average progress impeded less than 10%*/
	MODERATE_TRAFFIC_COND 		= 0,	/*Multiple vehicles detected concurently, average progress impeded less than 25%*/
	CONGESTED_TRAFFIC_COND 		= 0,	/*Multiple vehicles detected concurently, average progress impeded more than 25% but less than 75%*/
	JAMMED_TRAFFIC_COND			= 0		/*Multiple vehicles detected concurently, average progress impeded more than 75%*/
}eTrafficCond;

typedef enum temperature_cond_enum{
	UNKNOWN_TEMPERATURE_COND 		= 0,	/*Temperature conditions unknown*/
	NORMAL_TEMPERATURE_COND 		= 1,	/*Temperature 35F to 90F*/
	HOT_TEMPERATURE_COND 			= 2,	/*Temperature above 90F*/
	NEAR_FREEZING_TEMPERATURE_COND 	= 3,	/*Temperature 31F to 35F*/
	FREEZING_TEMPERATURE_COND 		= 4		/*Temperature below 31F*/
}eTemperatureCond;


typedef enum precipitation_cond{
	UNKNOWN_PRECIPITATION_COND 	= 0,
	NO_PRECIPITATION_COND 		= 1,
	LIGHT_PRECIPITATION_COND 	= 2,
	HEAVY_PRECIPITATION_COND	= 3
}ePrecipitationCond;

typedef enum visibility_cond{
	UNKNOWN_VISIBILITY_COND 		= 0,
	CLEAR_VISIBILITY_COND 			= 1,
	LIMITED_VISIBILITY_COND 		= 2,	/*LIGHT FOG/RAIN/SNOW*/
	VERY_LIMITED_VISIBILITY_COND	= 3		/*HEAVY FOG/RAIN/SNOW*/
}eVisibilityCond;


typedef enum daylight_cond{
	UNKNOWN_DAYLIGHT_COND 			= 0,
	NORMAL_DAYLIGHT_COND			= 1,
	SUNRISE_SUNSET_DAYLIGHT_COND 	= 2,	/*Could have glare from sun near horizon*/
	DIM_DAYLIGHT_COND 				= 3,	/*Pre-dawn, dusk, very dark cloudcover*/
	NO_DAYLIGHT_COND_dark			= 4		/*Full darkness*/
}eDaylightCond;


typedef enum view_cond{
	UNKNOWN_VIEW_COND 			= 0,
	OPEN_VIEW_COND 				= 1,	/*Sightlines not blocked*/
	PARTIALLY_BLOCKED_VIEW_COND = 2,	/*Sightlines somewhat blocked by vegetation or traffic*/
	BLOCKED_VIEW_COND 			= 3		/*Sightlines heavily blocked by vegetation or traffic*/
}eViewCond;


typedef enum driving_surface_cond{
	UNKNOWN_SURFACE_COND 	= 0,
	GOOD_SURFACE_COND 		= 1,	/*Smooth pavement*/
	ROUGH_SURFACE_COND 		= 2,	/*Pavement broken in many places/many potholes*/
	UNPAVED_SURFACE_COND	= 3		/*Dirt/grass/gravel*/
}eDrivingSurfaceCond;


typedef enum traction_cond{
	UNKNOWN_TRACTION_COND 		= 0,
	GOOD_TRACTION_COND 			= 1,
	MARGINAL_TRACTION_COND 		= 2,	/*Slight or intermittant tire slip detected*/
	POOR_TRACTION_COND			= 3		/*Significant and/or frequent tire slip detected*/
}eTractionCond;


typedef enum terrain_cond{
	UNKNOWN_TERRAIN_COND 	= 0,
	FLAT_TERRAIN_COND 		= 1,	/*Temperature 35F to 90F*/
	MODERATE_TERRAIN_COND	= 2,	/*Temperature above 90F*/
	SEVERE_TERRAIN_COND		= 4		/*Temperature below 31F*/
}eTerrainCond;


typedef enum location_occupancy_enum{
	UNKNOWN_LOCATION_OCCUPANCY 					= 0,
	LOCATION_UNOCCUPIED 						= 1,	/*No object is in the surrounding/intersection location and none expected within time window*/
	LOCATION_OCCUPATION_EXPECTED_IMMINENTLY		= 2,	/*Surrounding/intersection location likely occupied within the "imminently" threshold (e.g. 0.5 seconds)*/
	LOCATION_OCCUPATION_EXPECTED_SOON			= 3,	/*Surrounding/intersection location likely occupied within the "imminently" threshold (e.g. 2.0  seconds for surrounding lane, 4 seconds for intersection cross traffic?))*/
	LOCATION_OCCUPIED_AND_STATIONARY 			= 4,	/*Surrounding/intersection location occupied by stationary object*/
	LOCATION_OCCUPIED_AND_CONVERGING 			= 5,	/*Surrounding/intersection location occupied by object that is currently approaching host*/
	LOCATION_OCCUPIED_AND_DIVERGING 			= 6,	/*Surrounding/intersection location occupied by object that is currently separating from host*/
	LOCATION_OCCUPIED_AND_SIMILAR 				= 7		/*Surrounding/intersection location occupied by object that is not moving significantly towards/away from host*/
}eLocationOccupancy;


typedef enum sa_obj_location_enum{
	SA_NOT_NEAR 				= 0,	/*Not close to host vehicle at this time*/
	SA_AHEAD 					= 1,	/*Ahead, in lane*/
	SA_BEHIND 					= 2,	/*Behind, in lane*/
	SA_LEFT 					= 3,	/*Left, in adjacent lane (+/- 3 meters)*/
	SA_RIGHT 					= 4,	/*Right, in adjacent lane (+/- 3 meters)*/
	SA_AHEAD_LEFT 				= 5,	/*Ahead (more than 3 meters) in adjacent lane to left*/
	SA_AHEAD_RIGHT 				= 6,	/*Ahead (more than 3 meters) in adjacent lane to right*/
	SA_BEHIND_LEFT 				= 7,	/*Behind (more than 3 meters to rear) in adjacent lane to left*/
	SA_BEHIND_RIGHT 			= 8,	/*Behind (more than 3 meters to rear) in adjacent lane to rigjt*/
	SA_AHEAD_FAR_LEFT 			= 9,	/*Ahead, second lane over to left*/
	SA_AHEAD_FAR_RIGHT 			= 10,	/*Ahead, second lane over to right*/
	SA_BEHIND_FAR_LEFT 			= 11,	/*Behind, second lane over to left*/
	SA_BEHIND_FAR_RIGHT 		= 12,	/*Behind, second lane over to right*/
	SA_ONCOMING 				= 13,	/*Oncoming in host lane*/
	SA_ONCOMING_LEFT 			= 14,	/*Oncoming in lane segement to left of host*/
	SA_ONCOMING_RIGHT 			= 15,	/*Oncoming in lane segement to right of host*/
	SA_ONCOMING_FAR_LEFT 		= 16,	/*Oncoming but more than one lane segement to left of host*/
	SA_ONCOMING_FAR_RIGHT 		= 17,	/*Oncoming but more than one lane segement to right of host*/
	SA_INTERSECTING_FROM_LEFT 	= 18,	/*Intersecting direction, coming from host's left side*/
	SA_INTERSECTING_FROM_RIGHT 	= 19,	/*Intersecting direction, coming from host's right side*/
	SA_MERGING_FROM_LEFT 		= 20,	/*Merging into host lane, coming from host's left side*/
	SA_MERGING_FROM_RIGHT 		= 21,	/*Merging into host lane, coming from host's right side*/
	SA_AHEAD_RIGHT_EDGE 		= 22,	/*Ahead, in lane but on the right edge*/
	SA_AHEAD_LEFT_EDGE 			= 23,	/*Ahead, in lane but on the right edge*/
	SA_BEHIND_RIGHT_EDGE 		= 24,	/*Behind, in lane but on the right edge*/
	SA_BEHIND_LEFT_EDGE 		= 25,	/*Behind, in lane but on the right edge*/
	SA_INTERSECTING_FROM_LEFT_AHEAD 	= 26,	/*Intersecting direction, coming from host's left side. One lane ahead*/
	SA_INTERSECTING_FROM_RIGHT_AHEAD 	= 27,	/*Intersecting direction, coming from host's right side. One lane ahead*/
	SA_MERGING_FROM_LEFT_AHEAD 		= 28,	/*Merging into host lane, coming from host's left side. One lane ahead*/
	SA_MERGING_FROM_RIGHT_AHEAD 		= 29	/*Merging into host lane, coming from host's right side. One lane ahead*/


}eSAObjLocation;

// Lane Situation Indices
typedef enum lane_situation_enum {
	HV_LANE_SITUATION = 0,
	LEFT_ADJACENT_LANE_SITUATION = 1,
	RIGHT_ADJACENT_LANE_SITUATION = 2,
	LEFT_OPPOSING_LANE_SITUATION = 3,
	RIGHT_OPPOSING_LANE_SITUATION = 4,
    OPPOSING_SHARED_LANE_SITUATION  = 5,
    STRIAGHT_LANE_SITUATION = 6 // If HV lane is turning then the straight lane SA that connect from the current laneseg to the next primary lane seg and beyond.
}eLaneSituation;

typedef enum sa_obj_yield_priority_enum{
	YIELD_DUE_TO_STATIC_INFERIORITY				=0, // (ex obj in minor , host in major)
	DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY		=1, //  (ex obj in major , host in minor)
	YIELD_DUE_TO_ARRIVAL_INFERIORITY			=2, // ( ex both in minor but host arrived first)
	DO_NOT_YIELD_DUE_TO_ARRIVAL_SUPERIORITY		=3,  //  ( ex both in minor but obj arrived first)
	YIELD_SITUATION_AMBIGUOUS 					=4,// ( ex both in minor and arrived around the same time)
	DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY_RV_AT_INT = 5, // This is when the RV is at the connection/intersection even if it was originally on a minor lane segment and gets yield priority
	YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT    =6,   // This is when the HV is at the connection/intersection. even if RV was originally on a major lane segment  gets a lower yield priority
	DO_NOT_YIELD_DUE_TO_PERCEIVED_ARRIVAL_SUPERIORITY		=7,  //  According the SA/Fusion data HV appeared earlier. However the RV shows intentions of proceeding.
	DO_NOT_YIELD_IN_VIOLATION_OF_TRAFFIC_RULES = 8
}eSAObjYieldPriority;

typedef enum sa_lane_yield_priority_enum{
    LN_YIELD_DUE_TO_STATIC_INFERIORITY =0, // (ex obj in minor , host in major)
    LN_DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY =1, //  (ex obj in major , host in minor)
    LN_YIELD_ACCORDING_TO_ARRIVAL_ORDER =2, // ( ex both in minor/both in major but who arrived first)
    LN_YIELD_UNKNOWN=3
}eSLnYieldPriority;

typedef enum sa_obj_stationary_status_enum{
    STATIONARY_STATUS_MOVING=0,
    STATIONARY_STATUS_STATIONARY = 1,
    STATIONARY_STATUS_PARKED_ROADSIDE = 2,
    STATIONARY_STATUS_PARKED_PARKING_AISLE = 3,
    STATIONARY_STATUS_OFF_ROAD = 4
}eSAStationaryStatus;

typedef enum traffic_signal_state{
	/*	UNKNOWN_SIGNAL_STATE 				= 0,
	SOLID_RED_SIGNAL_STATE			 	= 1,	//For my current lane (and plan turned direction, if relevant), corresponding signal is red
	FLASHING_RED_SIGNAL_STATE 			= 2,
	YELLOW_SIGNAL_STATE			 		= 3,
	FLASHING_YELLOW_SIGNAL_STATE 		= 4,
	GREEN_SIGNAL_STATE			 		= 5,
	GREEN_ARROW_SIGNAL_STATE			= 6,
	RED_WITH_GREEN_ARROW_SIGNAL_STATE	= 7,	There is a simultaneous red light with a green arrow (for a specific turn direction)
	YELLOW_ARROW_SIGNAL_STATE			= 8,
	RED_ARROW_SIGNAL_STATE			 	= 9,
	STOP_SIGN_PRESENT_STATE 			= 10,
	YIELD_SIGN_PRESENT_STATE 			= 11
	 */
	TRAF_SIG_STATE_NO_DETECTION         = 0,
	TRAF_SIG_STATE_RED                  = 1,
	TRAF_SIG_STATE_YELLOW               = 2,
	TRAF_SIG_STATE_RED_YELLOW           = 3,
	TRAF_SIG_STATE_GREEN                = 4,
	TRAF_SIG_STATE_GREEN_YELLOW         = 6,
	TRAF_SIG_STATE_FLASHING_BIT         = 8, // (flashing modifier, not valid by itself)
	TRAF_SIG_STATE_FLASHING_RED         = 9,
	TRAF_SIG_STATE_FLASHING_YELLOW      = 10,
	TRAF_SIG_STATE_FLASHING_GREEN       = 12,
	TRAF_SIG_STATE_UP_DOWN_ARROW_BIT    = 16, // (up/down arrow modifier, not valid by itself)
	TRAF_SIG_STATE_STRAIGHT_ARROW_RED   = 17,
	TRAF_SIG_STATE_STRAIGHT_ARROW_YELLOW = 18,
	TRAF_SIG_STATE_STRAIGHT_ARROW_GREEN = 20,
	TRAF_SIG_STATE_LEFT_ARROW_BIT       = 32, // (left arrow modifier, not valid by itself)
	TRAF_SIG_STATE_LEFT_ARROW_RED       = 33,
	TRAF_SIG_STATE_LEFT_ARROW_YELLOW    = 34,
	TRAF_SIG_STATE_LEFT_ARROW_GREEN     = 36,
	TRAF_SIG_STATE_FLASHING_LEFT_ARROW_RED      = 41,
	TRAF_SIG_STATE_FLASHING_LEFT_ARROW_YELLOW   = 42,
	TRAF_SIG_STATE_FLASHING_LEFT_ARROW_GREEN    = 44,
	TRAF_SIG_STATE_RIGHT_ARROW_BIT      = 64, // (right arrow modifier, not valid by itself)
	TRAF_SIG_STATE_RIGHT_ARROW_RED      = 65,
	TRAF_SIG_STATE_RIGHT_ARROW_YELLOW   = 66,
	TRAF_SIG_STATE_RIGHT_ARROW_GREEN    = 68,
	TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_RED     = 73,
	TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_YELLOW  = 74,
	TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_GREEN   = 76,
	TRAF_SIG_RAILROAD_CROSSING_UNKNOWN  = 96,
	TRAF_SIG_RAILROAD_CROSSING_RED      = 97,
	TRAF_SIG_RAILROAD_CROSSING_CLEAR    = 100,
	TRAF_SIG_RAILROAD_CROSSING_BLOCKED  = 104,
	TRAF_SIG_PEDISTRIAN_SAME_DIR_UNKNOWN    = 108,
	TRAF_SIG_PEDISTRIAN_SAME_DIR_STOPPED    = 109,
	TRAF_SIG_PEDISTRIAN_SAME_DIR_WARNING    = 110,
	TRAF_SIG_PEDISTRIAN_SAME_DIR_CROSS      = 112,
	TRAF_SIG_PEDISTRIAN_PERP_DIR_UNKNOWN    = 116,
	TRAF_SIG_PEDISTRIAN_PERP_DIR_STOPPED    = 117,
	TRAF_SIG_PEDISTRIAN_PERP_DIR_WARNING    = 118,
	TRAF_SIG_PEDISTRIAN_PERP_DIR_CROSS      = 120,
	TRAF_SIG_UNKNOWN                    = 121,
	TRAF_SIG_OFF                        = 122

}eTrafficSignalState;


typedef enum object_rel_maneuver_enum{
	UNKNOWN_REL_MANUV 							= 0,
	FROM_RIGHT_TURNING_LEFT_REL_MANUV 			= 1,	/*Approaching from perpendicular (roughly) lane from host's right*/
	FROM_RIGHT_PROCEEDING_STRAIGHT_REL_MANUV 	= 2,
	FROM_RIGHT_TURNING_RIGHT_REL_MANUV 			= 3,
	ONCOMING_TURNING_LEFT_REL_MANUV 			= 4,	/*Approaching in an oncoming lane*/
	ONCOMING_PROCEEDING_STRAIGHT_REL_MANUV 		= 5,
	ONCOMING_TURNING_RIGHT_REL_MANUV 			= 6,
	FROM_LEFT_TURNING_LEFT_REL_MANUV 			= 7,	/*Approaching from perpendicular (roughly) lane from host's left*/
	FROM_LEFT_PROCEEDING_STRAIGHT_REL_MANUV 	= 8,
	FROM_LEFT_TURNING_RIGHT_REL_MANUV 			= 9,
	SAME_DIR_TURNING_LEFT_REL_MANUV 			= 10,	/*Driving in same direction (same or parallel lane)*/
	SAME_DIR_PROCEEDING_STRAIGHT_REL_MANUV 		= 11,
	SAME_DIR_TURNING_RIGHT_REL_MANUV 			= 12,
	STATIONARY_MANUV                            = 13
}eObjectRelManeuver;


typedef enum vehicle_running_state_enum{
	UNKNOWN_RUNNING_STATE 			= 0,
	NOT_RUNNING_STATE 				= 1,	/*Engine and/or electric propulsion system is off (key-off state)*/
	STARTING_RUNNING_STATE 			= 2,	/*Engine and/or electric propulsion system is in process of starting up (transition)*/
	STOPPING_RUNNING_STATE 			= 3,	/*Engine and/or electric propulsion system is in process of starting up (transition)*/
	NORMAL_RUNNING_STATE 			= 4,	/*Engine is running and/or electric propulsion in ready/active state (key-on state)*/
	ACCESSORY_ONLY_RUNNING_STATE 	= 5		/*Accessory circuits switched on but engine/propulsion is NOT ready/active state (key at accessory state)*/
}eVehicleRunningState;


typedef enum vehicle_gear_enum{
	UNKNOWN_GEAR_STATE 	= 0,
	PARK_GEAR 			= 1,
	REVERSE_GEAR 		= 2,
	NEUTRAL_GEAR 		= 3,
	DRIVE_GEAR 			= 4,
	LOW_GEAR 			= 5
}eVehicleGear;


typedef enum turn_indicator_state_enum{
	UNKNOWN_TURN_INDICATOR_STATE 		= 0,
	LEFT_TURN_INDICATOR_STATE 			= 1,
	RIGHT_TURN_INDICATOR_STATE 			= 2,
	EMERGENCY_FLASH_INDICATOR_STATE 	= 3,
	NO_TURN_INDICATOR_STATE 			= 4
}eTurnIndicatorState;

typedef enum position_pose_source_enum{
	POSITION_POSE_UNKNOWN 								= 0,	/*Position and pose cannot be determined*/
	POSITION_POSE_SOURCE_GPS_ONLY 						= 1,	/*Position and pose determined from GPS+IMU device output only*/
	POSITION_POSE_SOURCE_WH_ENC_RECKONING_ONLY 			= 2,	/*Position and pose determined by dead reckoning from wheel encoder values from previously-trusted position & pose*/
	POSITION_POSE_SOURCE_GPS_PLUS_WH_ENC 				= 3,	/*Position and pose determined from (Kalman?) filtered combination of GPS+IMU and wheel encoder data*/
	POSITION_POSE_SOURCE_VIS_ODO_RECKONING_ONLY 		= 4,	/*Position and pose determined by dead reckoning from visual odometry only from previously-trusted position & pose*/
	POSITION_POSE_SOURCE_GPS_PLUS_VIS_ODO 				= 5,	/*Position and pose determined from (Kalman?) filtered combination of GPS+IMU and visual odometry data*/
	POSITION_POSE_SOURCE_WH_ENC_PLUS_VIS_ODO_RECKONING 	= 6,	/*Position and pose determined by dead reckoning from (Kalman?) filtered combination of wheel encoder data and visual odometry data*/
	POSITION_POSE_SOURCE_GPS_PLUS_WH_ENC_PLUS_VIS_ODO 	= 7,	/*Position and pose determined from (Kalman?) filtered combination of GPS+IMU and wheel encoder data and visual odometry*/
	POSITION_POSE_SOURCE_ADVANCED_LOCALIZATION 			= 8		/*Position and pose determined from advanced localization methods (any of above methods with non-VO landmark triangulation, lanemarking/road edge analysis, etc)*/
}ePositionPoseSource;

/* FusionObjectList Obstacle Enums */
typedef enum fusion_obstacle_quality_enum {
	INVALID_OBSTACLE_QUALITY = 0,
	LOWEST_OBSTACLE_QUALITY = 1,
	L1_OBSTACLE_QUALITY = 1,
	L2_OBSTACLE_QUALITY = 2,
	L3_OBSTACLE_QUALITY = 3,
	LOW_OBSTACLE_QUALITY = 3,
	L4_OBSTACLE_QUALITY = 4,
	L5_OBSTACLE_QUALITY = 5,
	L6_OBSTACLE_QUALITY = 6,
	MODERATE_OBSTACLE_QUALITY = 6,
	L7_OBSTACLE_QUALITY = 7,
	L8_OBSTACLE_QUALITY = 8,
	HIGH_OBSTACLE_QUALITY = 8,
	L9_OBSTACLE_QUALITY = 9,
	L10_OBSTACLE_QUALITY = 10,
	HIGHEST_OBSTACLE_QUALITY = 10

} eFusionObstacleQuality;

typedef enum fusion_obstacle_type_enum {
	UNKNOWN_OBSTACLE_TYPE = 0,
	VEHICLE_OBSTACLE_TYPE = 1,
	TRUCK_OBSTACLE_TYPE   = 2,
	PEDESTRIAN_OBSTACLE_TYPE = 3,
	CYCLE_OBSTACLE_TYPE = 4,
	CONE_OBSTACLE_TYPE = 5,
	BARREL_OBSTACLE_TYPE = 6,

} eFusionObstacleType;

typedef enum road_object_type_enum {
	// Special types for visualizing roads, start higher than highest eFusionObstacleType

	ROAD_OBJECT_SOLID_YELLOW_STRIPE_PATCH = 21,
	ROAD_OBJECT_DASHED_YELLOW_STRIPE_PATCH = 22,
	ROAD_OBJECT_DASHED_WHITE_STRIPE_PATCH = 23,
	ROAD_OBJECT_PLAIN_ROAD_PATCH = 24
} eRoadObjectType;

typedef enum road_object_quality_provenance_enum {
	// Provenance/Quality hint

	ROAD_OBJECT_PROVENANCE_UNKNOWN = 0,
	ROAD_OBJECT_PROVENANCE_SD_MAP = 1,  // std. definition (e.g. navigation) map
	ROAD_OBJECT_PROVENANCE_DERIVED_MAP = 2,  // e.g. from logs of driven path
	ROAD_OBJECT_PROVENANCE_HD_MAP_BY_HAND = 3,  // hand-coded somehow
	ROAD_OBJECT_PROVENANCE_HD_MAP_FROM_CRUISE = 4,  // Extracted from Cruise Automation semantic database

	ROAD_OBJECT_PROVENANCE_RT_SENSED_MAP = 9,  // e.g. from real-time sensing scene interpretation

} eRoadObjectProvenance;

typedef enum fusion_sensor_source_flags_enum {
	SENSOR_SOURCE_UNKNOWN   = 0x000,
	SENSOR_SOURCE_LRRFC     = 0x001,
	SENSOR_SOURCE_LRRFL     = 0x002,
	SENSOR_SOURCE_LRRFR     = 0x004,
	SENSOR_SOURCE_LRRR      = 0x008,
	SENSOR_SOURCE_SRRFL     = 0x010,
	SENSOR_SOURCE_SRRFR     = 0x020,
	SENSOR_SOURCE_SRRRR     = 0x040,
	SENSOR_SOURCE_SRRRL     = 0x080,
	SENSOR_SOURCE_VLPL      = 0x100,
	SENSOR_SOURCE_VLPR      = 0x200,
	SENSOR_SOURCE_VISF      = 0x400,
	SENSOR_SOURCE_VISFL     = 0x800,
	SENSOR_SOURCE_VISFR     = 0x1000

} eFusionSensorSourceFlags;

typedef enum fusion_dynamic_property_enum {
	DYNAPROPER_UNKNOWN          = 0,
	DYNAPROPER_NEVER_MOVED      = 1,
	DYNAPROPER_MOVED_STOP       = 2,
	DYNAPROPER_MOVING           = 3,
	DYNAPROPER_MOVING_OPPOSITE  = 4

} eFusionDynamicProperty;

typedef enum fusion_rel_lane_enum {
	FUSION_REL_LANE_UNKNOWN     = 0,
	FUSION_REL_LANE_HOST        = 1,
	FUSION_REL_LANE_RIGHT       = 2,
	FUSION_REL_LANE_LEFT        = 3

} eFusionRelativeLane;

typedef enum module_status_enum {
	NOMINAL_MODULE_STATUS       = 0,    // Things are running OK, in normal full operations (for some modules, only when vehicle is autonomous)
	STANDBY_MODULE_STATUS       = 1,    // Ready to fully run, but not in a fully running state (sleeping/waiting for autonomous mode, etc.)
	UNKNOWN_MODULE_STATUS       = 2,    // Module status not known (probably not sent by module itself, but, for example, by MABXProxy when a module seems missing)
	INITIALIZING_MODULE_STATUS  = 3,    // Module is in normal start-up process, but not yet in a ready or fully-running state
	FAULTED_MODULE_STATUS       = 4,    // Module cannot function properly due to possibly-temporary condition (data gap, sensor inhibited by weather, etc.), may revert to INITIALIZING, STANDBY, or NOMINAL if condition clears
	FAILED_MODULE_STATUS        = 5     // Serious issue that the module should not continue, and would normally require human intervention to clear (fix hardware, reboot system, recalibrate/configure settings, etc.); once entered, the module would stay in this state until restarted
} eModuleStatus;

typedef enum module_issue_enum {
	NO_ISSUE                    = 0,
	MISSING_INPUT_DATA_ISSUE    = 1,
	LATE_INPUT_DATA_ISSUE       = 2,
	INVALID_INPUT_DATA_ISSUE    = 3,
	MISSING_SENSOR_ISSUE        = 4,
	FAILED_SENSOR_ISSUE         = 5,
	IMPARED_SENSOR_ISSUE        = 6,
	MISSING_ACTUATOR_ISSUE      = 7,
	FAILED_ACTUATOR_ISSUE       = 8,
	IMPARED_ACTUATOR_ISSUE      = 9,
	MEMORY_DEPLETED_ISSUE       = 10,
	STORAGE_DEPLETED_ISSUE      = 11,
	POWER_DEPLETED_ISSUE        = 12,
	COMM_NETWORK_OVERLOAD_ISSUE = 13,
	COMM_NETWORK_DATA_CORRUPTION_ISSUE = 14,
	COMM_NETWORK_ACCESS_ISSUE   = 15,
	DEADLOCK_ISSUE              = 16,
	MISSED_DEADLINE_ISSUE       = 17,

	ETHERNET_SOCKET_INIT_ISSUE  = 30,
	ETHERNET_SOCKET_RX_ISSUE    = 31,
	ETHERNET_SOCKET_TX_ISSUE    = 32,

	CANBUS_INIT_ISSUE           = 35,
	CANBUS_RX_ISSUE             = 36,
	CANBUS_Tx_ISSUE             = 37,

	SERIAL_COMM_INIT_ISSUE      = 40,
	SERIAL_COMM_RX_ISSUE        = 41,
	SERIAL_COMM_Tx_ISSUE        = 42,

	SHARED_DATA_INIT_ISSUE      = 50,
	SHARED_DATA_SUBSCRIBE_ISSUE = 51,
	SHARED_DATA_HANDLER_ISSUE   = 52,
	SHARED_DATA_FORMAT_ISSUE    = 53,
	SHARED_DATA_OFFER_PUB_ISSUE = 54,
	SHARED_DATA_PUBLISHING_ISSUE= 55,

    LOCAL_LOGGING_INIT_ISSUE    = 60,


	// ...
	// (First module ID 1 specific issue) = 100
	// ( additional module ID 1 issues from 101 to 199...)
	// ...
	// (First module ID 2 specific issue) = 200
	// ( additional module ID 2 issues from 201 to 299...)
	// ...
	//   etc., etc.

	// TRAJECTORY PLAN ISSUES
	MISSING_POSITIONPOSE_DATA_ISSUE = 1900,
	MISSING_KINEMATICS_DATA_ISSUE   = 1901,
	MISSING_OBSTACLE_DATA_ISSUE     = 1902,
	MISSING_SCENARIO_DATA_ISSUE     = 1903,
	MISSING_BEHAVIOR_DATA_ISSUE     = 1904,
	FAILED_GRAPH_GENERATION_ISSUE   = 1905,
	FAILED_OPTIMAL_TRAJECTORY_ISSUE = 1906,
	FAILED_LOCAL_TRAJECTORY_ISSUE   = 1907,
	MISSING_SITUATION_DATA_ISSUE    = 1908,
	MISSING_OBJECT_SITUATION_DATA_ISSUE    = 1909,

	// Scenario Planner issues
	INVALID_LOCALIZATION_DATA_ISSUE = 1800,
	MISSING_ROUTESEGMENTLIST_DATA_ISSUE = 1801,
	MISSING_LANESEGMENT_DATA_ISSUE = 1802,



	// KINEMATIC ISSUES
	MISSING_RAWGPSIMU_DATA_ISSUE = 2900,

    // MAPLET ISSUES
    MAPLET_MISSING_POSITIONPOSE_DATA_ISSUE = 3300,
    MAPLET_INVALID_POSITIONPOSE_DATA_ISSUE = 3301,
    MAPLET_MISSING_MAP_FILE_ISSUE          = 3302,
    
    LAST_ISSUE = 9999

} eModuleIssue;

typedef enum module_id_enum {
	UNKNOWN_MODULE_ID = 0,
	SRR_LEFT_FRONT_MODULE_ID = 1,
	SRR_RIGHT_FRONT_MODULE_ID = 2,
	SRR_LEFT_BACK_MODULE_ID = 3,
	SRR_RIGHT_BACK_MODULE_ID = 4,
	LRR_LEFT_FRONT_MODULE_ID = 5,
	LRR_RIGHT_FRONT_MODULE_ID = 6,
	LRR_LEFT_BACK_MODULE_ID = 7,
	LRR_RIGHT_BACK_MODULE_ID = 8,
	LIDAR_LEFT_A_PILLAR_MODULE_ID = 9,
	LIDAR_RIGHT_A_PILLAR_MODULE_ID = 10,
	LIDAR_TOP_MODULE_ID = 11,
	SURROUND_CAM_LEFT_MODULE_ID = 12,
	SURROUND_CAM_RIGHT_MODULE_ID = 13,
	SURROUND_CAM_FRONT_MODULE_ID = 14,
	SURROUND_CAM_BACK_MODULE_ID = 15,
	LR_CAM_LEFT_MODULE_ID = 16,
	LR_CAM_RIGHT_MODULE_ID = 17,
	LR_CAM_FRONT_MODULE_ID = 18,
	LR_CAM_BACK_MODULE_ID = 19,
	GPS_IMU_RECEIVE_MODULE_ID = 20,
	KINEMATICS_MODULE_ID = 21,
	MOBILEYE_MODULE_ID = 22,
	LANE_FUSION_MODULE_ID = 23,
	SURROUND_VIEW_MODULE_ID = 24,
	CROSS_TRAFFIC_FRONT_MODULE_ID = 25,
	SENSOR_FUSION_MODULE_ID = 26,
	MAP_FUSION_MODULE_ID = 27,
	TRAFFIC_SIGNAL_INTERPRETATION_MODULE_ID = 28,
	POSITION_POSE_MODULE_ID = 29,
	VO_MODULE_ID = 30,
	MAP_DB_SERVICE_MODULE_ID = 31,
	MAP_DB_PROXY_MODULE_ID = 32,
	MAPLET_PROVIDER_MODULE_ID = 33,
	PATH_PLANNER_MODULE_ID = 34,
	SCENARIO_PLANNER_MODULE_ID = 35,
	SITUATION_AWARENESS_MODULE_ID = 36,
	BEHAVIOR_CONTROL_MODULE_ID = 37,
	TRAJECTORY_PLANNER_MODULE_ID = 38,
	HMI_MODULE_ID = 39,
	SAFETY_MONITOR_MODULE_ID = 40,
	VIDEO_LOGGING_MODULE_ID = 41,
	CAN_LOGGING_MODULE_ID = 42,
	ETHERNET_LOGGING_MODULE_ID = 43,
	LOGGING_CONTROL_MODULE_ID = 44,
	MABXPROXY_MODULE_ID = 45,
	EPS_MODULE_ID = 46,
	EBCM_MODULE_ID = 47,
	PATH_PREDICTION_MODULE_ID = 48,
	MOTION_CONTROLLER_MODULE_ID = 49,
	CROSS_TRAFFIC_BACK_MODULE_ID = 50,
	CROSS_TRAFFIC_LEFT_MODULE_ID = 51,
	CROSS_TRAFFIC_RIGHT_MODULE_ID = 52,
	MAX_MODULE_IDS
} eModuleID;

typedef enum vehicle_autonomous_state_enum {
	OFF_AUTONOMOUS_STATE = 0,
	READY_TO_ASSIST_AUTONOMOUS_STATE = 1,
	FAIL_STEERING_AUTONOMOUS_STATE = 2,
	FAIL_AUTONOMOUS_STATE = 3,
	ABORT_AUTONOMOUS_STATE = 4,
	DRIVER_OVERRIDE_AUTONOMOUS_STATE = 5,
	WARNING_AUTONOMOUS_STATE = 6,
	NOT_READY_TO_ASSIST_AUTONOMOUS_STATE = 7,
	ACTIVE_AUTONONOMOUS_STATE = 8
} eVehicleAutonomousState;


typedef enum localization_source_type_enum {
	WHEEL_ODOMETRY = 0,
	VISUAL_ODOMETRY = 1,
}eLocalizationSourceType;


typedef enum mapping_providence {
	PRECISION_MAPPING,
	LIDAR_MAPPING,//e.g. Cruise mapping
	NEW_SELF_MAPPING,
	REPEATED_SELF_MAPPING
} eMappingProvidence;

typedef enum notification_emblem_code {
	NO_EMBLEM = 0,
	SYSTEM_INIT_EMBLEM = 1,
	STEERING_EMBLEM = 2,
	BRAKE_EMBLEM = 3,
	PROPULSION_EMBLEM = 4,
	GPS_EMBLEM = 5,
	IMU_EMBLEM = 6,
	WHEEL_SENSORS_EMBLEM = 7,
	CAMERA_EMBLEM = 8,
	RADAR_EMBLEM = 9,
	LIDAR_EMBLEM = 10,
	COMPUTER_EMBLEM = 11,
	NETWORKING_EMBLEM = 12,
	SOFTWARE_EMBLEM = 13,
	MAP_EMBLEM = 14,
	MANEUVER_EMBLEM = 15,
	ARRIVAL_EMBLEM = 16,
	APPROVAL_NEEDED_EMBLEM = 17,
	DRIVER_INTERVENTION_EMBLEM = 18,
	SYSTEM_UP_EMBLEM = 19,
	SYSTEM_DOWN_EMBLEM = 20,
	YIELD_SIGN_EMBLEM = 21,
	STOP_SIGN_EMBLEM = 22,
	PED_CROSSING_EMBLEM = 23,
	RAILROAD_CROSSING_EMBLEM = 24,
	PASSENGER_EMBLEM = 25,              // EMBARK/DISEMBARK
	STOPLIGHT_GENERIC_EMBLEM = 26,
	STOPLIGHT_RED_EMBLEM = 26,
	STOPLIGHT_YELLOW_EMBLEM = 27,
	STOPLIGHT_GREEN_EMBLEM = 28,
	STOPLIGHT_GREEN_LEFT_ARROW_EMBLEM = 29,
	STOPLIGHT_GREEN_RIGHT_ARROW_EMBLEM = 30,
	PLANNING_EMBLEM = 31,
	LEFT_OF_PATHLINE_EMBLEM = 32,
	RIGHT_OF_PATHLINE_EMBLEM = 33,
	MISALIGNMENT_EMBLEM = 34
} eNotificationEmblamCode;

typedef enum sensor_source_flags_enum {
    SENBMP_LRRF  = 0x00000001, //long range radars
    SENBMP_LRRB  = 0x00000002,
    SENBMP_LRRFL = 0x00000004,
    SENBMP_LRRFR = 0x00000008,
    SENBMP_LRRBL = 0x00000010,
    SENBMP_LRRBR = 0x00000020,
    SENBMP_SRRFL = 0x00000040, //short range radars
    SENBMP_SRRFR = 0x00000080,
    SENBMP_SRRBR = 0x00000100,
    SENBMP_SRRBL = 0x00000200,
    SENBMP_LONCAM_F = 0x00000400,
    SENBMP_LONCAM_L = 0x00000800,
    SENBMP_LONCAM_R = 0x00001000,
    SENBMP_LONCAM_B = 0x00002000,
    SENBMP_FCM = 0x00004000, //forward camera  module 
    SENBMP_SURROUNDVISION = 0x00008000, //surround view camera module 
    SENBMP_VELODYNE_L     = 0x00010000,
    SENBMP_VELODYNE_R     = 0x00020000,
    SENBMP_VELODYNE_B     = 0x00040000,
    SENBMP_COASTING       = 0x80000000 
} eSensorSourceFlags;

typedef enum map_fusion_hyp_maneuever_type_enum {
    HYP_MANEUVER_MOVING_ONROAD      = 0x00, // moving on a road
    HYP_MANEUVER_MOVING_OFFROAD     = 0x01, // moving off a road
    HYP_MANEUVER_STATIONARY_OFFROAD = 0x10, // stationary off road 
    HYP_MANEUVER_PARKED_ROAD_SIDE   = 0x11, // parked beside road
    HYP_MANEUVER_PARKED_PARKING_LOT = 0x12, // parked in parking lot
    HYP_MANEUVER_STATIONARY_ONROAD  = 0x13, // stationary on a road
    HYP_MANEUVER_LANE_CHANGE_LEFT   = 0x21, // changing to left lane
    HYP_MANEUVER_LANE_CHANGE_RIGHT  = 0x22  // changing to right lane
} eMapFusionHypManeuverType;


#endif
/***** Enumerations from Data dictionary -end **********/

