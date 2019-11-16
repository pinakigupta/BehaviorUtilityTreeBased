#pragma once
#include "../PCH/pch.hpp"

extern double RIGHT_ON_RED_ENABLE;
extern double DEBUG_OBJ_UID;
extern double DLCX_ALLWD_GLOBALLY;
extern double VEHICLE_LENGTH;
extern double INTERSECTION_FOV_TTC_THRESH;
extern double PRINT_CODE_RBG;
extern double PRINT_CODE_R;
extern double PRINT_CODE_B;
extern double PRINT_CODE_G;
extern double PRINT_CODE_Y;
extern double PRINT_CODE_P;
//extern std::string PRINT_CODE_RBG_FILES_CONTAINING;
extern double BEHAVIOR_EXEC_WAIT_TIME_S;
extern double TEMP_DISALLOW_ACC;
extern double SA_CLEANUP_TIME_THRSH;
extern double ALTPOLICIES_CLEANUP_TIME_THRSH;
extern double SAFE_FOLLOWING_OFFSET,SAFE_FOLLOWING_ADDOFFSET_STATIONARY;
extern double ENABLE_OPTIMAL_POLICY_SEARCH;
extern double DEFAULT_WAIT_TIME_BEHIND_PASSIVE_ROAD_BLOCK_s;
extern double ENABLE_PASSIVE_ROAD_BLOCK_HANDLING;
extern double  DEC_VEL_TO_YIELD_MPS;
extern double  TURN_TGT_DIST_OFFSET;
extern double  DEC_VEL_TO_TURN_FACTOR;
extern double  KAPPA_THRESHOLD_FOR_ROAD_BENDS;
extern double KAPPA_LOOK_AHEAD_HORIZON;
extern double KAPPA_IMMEDIATE_LOOK_AHEAD_HORIZON;
extern double KAPPA_LOWER_THRESHOLD_FOR_ROAD_BENDS;
extern double  SA_LOOK_AHEAD_HORIZON;
extern double  SAFE_PEDESTRIAN_YIELDING_OFFSET_m;
extern double  MAP_INCONSISTENCY_THRESHOLD_m;
extern double K_User_Global_Confirmed_1;
extern double SAFE_FOLLOWING_MIN_DIST;
extern double SAFE_FOLLOWING_CRITICAL_MIN_DIST;
extern double LOOK_AHEAD_HORIZON_WHEN_STATIONARY_m;
extern double LOOK_AHEAD_TIME_s;
extern double DIST_THRSH_FOR_FLOW_RESTRICTION_CONSIDERATION;
extern double SAFE_FOLLOWING_HV_MIN_SPEED;
extern double SAFE_FOLLOWING_MIN_SPEED;
extern double SAFE_FOLLOWING_MAX_SPEED;
extern double CONTROLLER_OVERSHOOT_LOWERBOUND_AT_STOPS_m;
extern double CONTROLLER_OVERSHOOT_LOWERBOUND_AT_SIGNALS_m;
extern double VEHICLE_VELOCITY_THRESHOLD_FOR_COMPLETE_STOP_mps;
extern double DIST_THRSH_FOR_FLOW_RESTRICTION_COST_INFLUENCE;
extern double DIST_THRESHOLD_FOR_COMPLETE_STOP;
extern double INTERSECTION_VICINITY_THRESHOLD;
extern double INTERSECTION_EXIT_VICINITY_THRESHOLD;
extern double BEHAVIOR_LOOK_AHEAD_HORIZON;
extern double BEHAVIOR_LOOK_AHEAD_TIME;
extern double CodeRed_TimeGap;
extern double CodeGreen_TimeGap;
extern double CodeBlue_TimeGap;
extern double LAT_ACCEL_LIMIT;
extern double ENABLE_REMEDIAL_ACTIONS;
extern double VISUALIZER_NOTIFICATION_ENABLED_FOR_CODERED;
extern double EXIT_OBJ_DETECTION_UNCERTAINTY_SAFETY_MARGIN;
extern double PED_YIELD_DISTANCE;
extern double PED_YIELD_TTC_THRESH;
extern double PED_SLOWDOWN_TTC_THRESH;
extern double COLLISION_IMMINENT_DISTANCE_THRESH;
extern double COLLISION_IMMINENT_TTC_THRESH;
extern double BEHAVIORPOLICY_EXEC_WAIT_TIME_S;
extern double SPD_THRSH_FOR_DISTBSD_COST;
extern double INTERSECTION_TM_THRESHOLD_FOR_FAR_LANE_THREAT_ASSESSMENT;


extern double DIAGNOSTICS_MAX_DTAR_SKIP_THRESHOLD;
extern double DIAGNOSTICS_SEGMENT_MISSING_TIME_THRESHOLD;
extern double BEHAVIORVISUALIZATION_PUB_ENABLED;
extern double ALTBEHAVIORVISUALIZATION_PUB_ENABLED;
extern double BEHAVIORPOLICY_CONT_PUB_ENABLED;
extern double USEROUTPUTS_PUB_ENABLED;
extern double BEHAVIORPLOT_WAIT_TIME_S;
extern double BEHAVIOR_PUB_WAIT_TIME_S;
extern double LANESEG_REQUEST_ENABLED;
extern double RAISE_INTERRUPT,RAISE_INTERRUPT_VALUE;
extern double PRINT_PATHPLAN;
extern double PRINT_ALTPATHPLAN;

extern double GAP_PRED_ACCEL_FACTOR;
extern double GAP_PRED_SPEED_FACTOR;
extern double MIN_GAP_COST_THRESH ;
extern double MIN_GAP_CREATION_COST_THRESH;
extern double GAP_CREATION_HV_SPD_THRSH;
extern double ABRT_MIN_GAP_COST_THRESH ;
extern double LCX_LOOKAHEAD_TIME_HORIZON;
extern double LCX_LOOKAHEAD_STEP;
extern double LCX_TM_LEFT_FOR_LCX_ABRT_s,LCX_TM_LEFT_FOR_LCX_ABRT_TRAJ_COMMITED_s ;
extern double LCX_TM_LEFT_FOR_RECOMMENDED_ZONE_s ;
extern double LCX_TM_LEFT_FOR_URGENT_ZONE_s;
extern double LCX_MIN_GAP_BEHIND;
extern double LCX_MIN_GAP_BEHIND_STATIC_OBJ;
extern double LCX_MIN_GAP_AHEAD;
extern double LCX_GAP_FILTER_PASS_THRESH;
extern double LCX_PRED_ACCEL_DEC;
extern double MAX_TARGET_LANE_CENTER_OFFSET_FOR_LCX_CMPLT;

extern double LCX_TM_LEFT_FOR_GAP_CREATION_ABRT_THRSH_s;
extern double LCX_TM_LEFT_FOR_GAP_CREATION_THRSH_s;
extern double GAP_CREATION_DECEL_LIMIT;
extern double GAP_CREATION_DECEL_STEP;
extern double GAP_CREATION_ACCEL_LIMIT;
extern double GAP_CREATION_ACCEL_STEP;
extern double GAP_CREATION_TGT_DIST;

extern double OPPORTUNISTIC_LCX_URGENCY_OVRD_DIST_THRSH;
extern double OPPORTUNISTIC_LCX_IMPATIENCE_TM_THRESHOLD;
extern double OPPORTUNISTIC_LCX_IMPATIENCE_SPD_THRESHOLD;
extern double OPPORTUNISTIC_LCX_IMPATIENCE_HV_SPD_THRESHOLD;
extern double OPPORTUNISTIC_LCX_IMPATIENCE_DIST_THRESHOLD;
extern double OPPORTUNISTIC_LCX_EOS_DIST_THRESHOLD;
extern double OPPORTUNISTIC_LCX_BEHIND_CLR_DIST_THRSH;
extern double OPPORTUNISTIC_LCX_AHEAD_CLR_DIST_THRSH;
extern double OPPORTUNISTIC_LCX_DLCX_ALLWD_DIST_THRSH;
extern double DO_NOT_USE_OPPORTUNISTIC_LCX_ENBL_OVRRD;

extern double TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGN_STATE;
extern double TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGNAL_STATE;

extern double SHARED_LANE_RV_BOTTLENECK_DIST_THRESH;
extern double SHARED_LANE_HV_BOTTLENECK_DIST_THRESH;

extern double LEFT_ADJACENT_LANE_POLICY_SWITCH_COST;
extern double RIGHT_ADJACENT_LANE_POLICY_SWITCH_COST;
extern double LEFT_OPPOSING_LANE_POLICY_SWITCH_COST;
extern double RIGHT_OPPOSING_LANE_POLICY_SWITCH_COST;
extern double ACTIVE_POLICY_CHANGE_COST_THRESH;

extern pthread_mutex_t mutex_global_calibration;


template<typename T>
inline void ExtractVar(std::string name, T  &value, const boost::property_tree::ptree &pt, pthread_mutex_t& mutex_local_calibration ){
	pthread_mutex_lock(&mutex_local_calibration);
	try{
		value = pt.get<T>(name);
	}catch(...){
		// Do Nothing
	}
	pthread_mutex_unlock(&mutex_local_calibration);
}

template<typename T>
inline void ExtractVar(std::string name, T  &value, const boost::property_tree::ptree &pt ){
	ExtractVar(name,value,pt,mutex_global_calibration);
}

#define CALL_ExtractVar(VAR,pt) \
	extern double VAR; \
	ExtractVar(#VAR, VAR,pt)


#define UpdateCal(VAR,CalFile)  \
  ( \
		    (boost::property_tree::ptree pt),\
			(pthread_mutex_lock(&mutex_global_calibration) ), \
			(boost::property_tree::json_parser::read_json(CalFile, pt)  ), \
			(CALL_ExtractVar(VAR,pt) ),\
			(pthread_mutex_unlock(&mutex_global_calibration) ), \
			(void)0 \
  ) \


void *json_handler(void *);
