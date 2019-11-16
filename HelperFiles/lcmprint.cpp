#include "../PCH/pch.hpp"
#include "../include/lcmprint.hpp"
#include "../include/BehaviorEnums.h"



std::ostream& lcmprint::operator<<(std::ostream& os,const exlcm::policycosts_t& MyPolicyCost){
	os<<" Total cost = "<<MyPolicyCost.TotalCost<<" Behavior related Cost = "<<MyPolicyCost.BehaviorPolicyCost.PolicyCost<<" TrafficEfficiencyCost "<<\
			MyPolicyCost.TrafficEfficiencyCost<<" routeCost= "<<MyPolicyCost.routeCost<<" PolicyTransitionCost "<<MyPolicyCost.PolicyTransitionCost<<std::endl;
	return os;
}



std::ostream& lcmprint::PrintEnum(std::ostream &os, const sa_obj_stationary_status_enum My_stationary_state){
	switch(My_stationary_state)
	{
	PrintBlue(STATIONARY_STATUS_MOVING);
	PrintBlue(STATIONARY_STATUS_OFF_ROAD);
	PrintBlue(STATIONARY_STATUS_STATIONARY);
	PrintBlue(STATIONARY_STATUS_PARKED_ROADSIDE);
	PrintBlue(STATIONARY_STATUS_PARKED_PARKING_AISLE);
	default: {os<<blue_on<<"UNKNOWN_STATIONARY_STATUS_IS_ACTIVE. value is "<<(int)My_stationary_state<<std::endl;break;}
	}
	return os;
}


std::ostream& lcmprint::PrintEnum(std::ostream &os, const TempRules My_rule){
	switch(My_rule)
	{
	PrintBlue(ASSUME_RIGHT_OF_WAY_RULE);
	PrintBlue(YIELD_UNTIL_CLEAR_RULE);
	PrintBlue(STOP_AND_YIELD_UNTIL_CLEAR_RULE);
	PrintBlue(HANDLE_MULTIWAY_STOP_RULE);
	PrintBlue(ASSUME_RIGHT_OF_WAY_AFTER_SIGNAL_RULE);
	PrintBlue(YIELD_UNTIL_CLEAR_AFTER_SIGNAL_RULE);
	PrintBlue(RIGHT_ON_RED_RULE);
	PrintBlue(MERGE_AT_TRAFFIC_SPEED_RULE);
	PrintBlue(EXIT_AT_TRAFFIC_SPEED_RULE);
	PrintBlue(CHANGE_LANES_RULE);
	PrintBlue(OBEY_POSTED_SIGN_RULE);
	PrintBlue(YIELD_UNTIL_CREEP_DISTANCE_REACHED);
	PrintBlue(FOLLOW_INTERSECTION_RULE);
	PrintBlue(ASSUME_RIGHT_OF_WAY_AFTER_RAIL_CROSSING_CLEAR_RULE);
	PrintBlue(NO_STOP_ZONE);
	PrintBlue(SLOWDOWN_ZONE);
	PrintBlue(RIGHT_OF_WAY_AFTER_SIGNAL_STOP_ZONE);
	PrintBlue(YIELD_TO_OBJ);
	PrintBlue(ACC);
	PrintBlue(LEFT_TURN);
	PrintBlue(RIGHT_TURN);
	PrintBlue(TURN_ALONG_CURVY_ROAD_);
	PrintBlue(YIELD_FOR_PEDESTRIAN_RULE);
	PrintBlue(STOP_AND_STAND_BY);
	PrintBlue(YIELD_AT_TRAFFIC_LIGHT);
	PrintBlue(STOP_AT_TRAFFIC_LIGHT);
	PrintBlue(SLOWDOWN_FOR_SAFETY);
	PrintBlue(COLLISION_IMMINENT);
	PrintBlue(CHANGE_LANES_OPPORTUNISTIC);
	PrintBlue(NOT_DEFINED_YET);
	PrintBlue(NO_RULES);
	PrintBlue(OBEY_SIGN_IF_POSTED_RULE);

	default: {os<<blue_on<<"UNKNOWN_RULE_IS_ACTIVE. Rule value is "<<(int)My_rule<<std::endl;break;}
	}
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const lane_situation_enum my_lane_situation ){
	switch(my_lane_situation)
	{
	PrintPurple(HV_LANE_SITUATION);
	PrintPurple(LEFT_ADJACENT_LANE_SITUATION);
	PrintPurple(RIGHT_ADJACENT_LANE_SITUATION);
	PrintPurple(LEFT_OPPOSING_LANE_SITUATION);
	PrintPurple(RIGHT_OPPOSING_LANE_SITUATION);
	PrintPurple(OPPOSING_SHARED_LANE_SITUATION);
	PrintPurple(STRIAGHT_LANE_SITUATION);
	default: {os<<purple_on<<"Not sure. lane_situation is "<<(int)my_lane_situation<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}


std::ostream& lcmprint::operator <<(std::ostream &os, const exlcm::pathstep_t &MyStep)
{
	int CurrentSegUID = MyStep.current_segment_group;
	int TargetSegUID = MyStep.target_segment_group;
	os << "step_index_n = " << MyStep.step_index_n <<std::endl;
	os<<"  current_segment_station_m = "<<MyStep.current_segment_station_m<<"  target_segment_station_m = "<<MyStep.target_segment_station_m;
	os<<"  posted_speed_lim_mps "<<MyStep.posted_speed_lim_mps<<std::endl;
	os<<" step_task = ";
	PrintEnum(os,(route_step_task_enum)MyStep.step_task);
	os<<" step_rule = ";
	PrintEnum(os,(TempRules)MyStep.step_rule);
	os<<" step_direction = ";
	PrintEnum(os,(direction_enum)MyStep.direction);
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const lane_intention_enum my_lane_intention){
	switch(my_lane_intention)
	{
	PrintPurple(LANE_CHANGE_IS_PROHIBITED);
	PrintPurple(LANE_CHANGE_FOR_EMERGENCY_ONLY);
	PrintPurple(LANE_CHANGE_IS_ALLOWED);
	PrintPurple(LANE_CHANGE_IS_DESIRED);
	PrintPurple(LANE_CHANGE_IS_RECOMMENDED);
	PrintPurple(LANE_CHANGE_IS_URGENT);
	PrintPurple(LANE_CHANGE_IS_NOT_DESIRED);
	PrintPurple(LANE_CHANGE_IN_PROGRESS);
	PrintPurple(LANE_CHANGE_NEAR_COMPLETION);
	PrintPurple(LANE_CHANGE_OPPOSING);
	default: {os<<purple_on<<"Not sure. lane_intention is "<<(int)my_lane_intention<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const behavior_accel_request my_behavior_accel){
	switch(my_behavior_accel)
	{
	PrintPurple(COMFORT_ACCEL_REQUEST_BEHAVIOR);
	PrintPurple(AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR);
	PrintPurple(SUPER_AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR);
	PrintPurple(SLOW_ACCEL_REQUEST_BEHAVIOR);
	PrintPurple(SUPER_SLOW_ACCEL_REQUEST_BEHAVIOR);
	default: {os<<purple_on<<"Not sure. behavior_accel_request is "<<(int)my_behavior_accel<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}


std::ostream& lcmprint::PrintEnum(std::ostream &os, const reroute_type_enum my_reroute){
	switch(my_reroute)
	{
	PrintPurple(NO_REROUTE);
	PrintPurple(OPTIMAL_REROUTE);
	PrintPurple(SAFE_REROUTE);
	PrintPurple(STRAIGHT_ROUTE);
	PrintPurple(LEFT_LANE_CHANGE_ROUTE);
	PrintPurple(RIGHT_LANE_CHANGE_ROUTE);
	PrintPurple(LANE_CHANGE_ROUTE);
	PrintPurple(LANE_CHANGE_ROUTE_DLCX_ALLWD);
	PrintPurple(LANE_CHANGE_ROUTE_DLCX_RQRD);
	PrintPurple(OPPOSING_LANE_CHANGE_ROUTE);
	default: {os<<purple_on<<"Not sure. reroute is "<<(int)my_reroute<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}


std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::useroutputs_t& my_useroutputs){
	os<<" user output: ";
	os<<green_on<<" timestamp_sec "<<color_off<<my_useroutputs.timestamp_sec<<std::endl;
	os<<" autonomous_mode_state "<<my_useroutputs.autonomous_mode_state<<" turn_indication_request "<<(int)my_useroutputs.turn_indication_request;
	os<<" horn_request "<<(int)my_useroutputs.horn_request<<std::endl;
	os<<" current_lane_segment_uid "<<my_useroutputs.current_lane_segment_uid<<" next_lane_segment_uid "<<my_useroutputs.next_lane_segment_uid<<std::endl;
	os<<" primary_target_object_id "<<my_useroutputs.primary_target_object_id<<" target_speed_mps "<<my_useroutputs.target_speed_mps<<std::endl;
	os<<" current_maneuver_task ";
	lcmprint::PrintEnum(os,(route_step_task_enum)my_useroutputs.current_maneuver_task);
	os<<" next_maneuver_task ";
	lcmprint::PrintEnum(os,(route_step_task_enum)my_useroutputs.next_maneuver_task);
	os<<" current task direction ";lcmprint::PrintEnum(os,(direction_enum)my_useroutputs.current_maneuver_direction);
	os<<" next task direction ";lcmprint::PrintEnum(os,(direction_enum)my_useroutputs.next_maneuver_direction);
	return os;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::localization_t & my_localization){
	os<<blue_on<<" localization info: "<<color_off<<std::endl;
	os<<"matched_seg_fr "<<my_localization.matched_seg_fr<<" matched_laneseg_uid "<<my_localization.matched_laneseg_uid<<" matched_lane_station_m "<<\
			my_localization.matched_lane_station_m<<" matched_conn_n "<<my_localization.matched_conn_n<<std::endl;
	os<<" localization_confidence_fr "<<my_localization.localization_confidence_fr<<" localization_sources "<<my_localization.localization_sources<<std::endl;
	os<<" inside intersection "<<(int)my_localization.inside_intersection_f<<" matched_intersection_uid "<<my_localization.matched_intersection_uid<<" dist_to_matched_intersection_m "<<\
			my_localization.dist_to_matched_intersection_m<<std::endl;
	return os;

}



std::ostream& lcmprint::PrintEnum(std::ostream &os, const lane_change_feedback_enum feedback ){
	lane_change_feedback_enum my_feedback = (lane_change_feedback_enum)feedback;
	switch(my_feedback)
	{
	PrintPurple(COMMITED_TO_LEFT);
	PrintPurple(COMMITED_TO_RIGHT);
	PrintPurple(REJECTED);
	PrintPurple(ABORTED);
	PrintPurple(INACTIVE);
	PrintPurple(INITIATED);
	default: {os<<purple_on<<"Not sure. Lane change feedback value is "<<(int)my_feedback<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const direction_enum my_direction ){

	switch(my_direction)
	{
	PrintPurple(STRAIGHT_DIRECTION);
	PrintPurple(BEAR_LEFT);
	PrintPurple(BEAR_LEFT_OPPOSING_LANE);
	PrintPurple(TURN_LEFT);
	PrintPurple(BEAR_RIGHT);
	PrintPurple(BEAR_RIGHT_OPPOSING_LANE);
	PrintPurple(TURN_RIGHT);
	PrintPurple(REVERSE_STRAIGHT);
	PrintPurple(REVERSE_LEFT);
	PrintPurple(REVERSE_RIGHT);
	default: {os<<purple_on<<"Not sure. Direction value is "<<(int)my_direction<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::pathplan_t& my_pathPlan){
	os<<" behaviorinput::pathPlan: ";
	os<<" valid_f = "<<my_pathPlan.valid_f<<" total_step_count = "<<my_pathPlan.total_step_count<<" included_step_count = "<<my_pathPlan.included_step_count<<std::endl;
	if(my_pathPlan.total_step_count>0){
		for(int i=0;i<=my_pathPlan.total_step_count;i++){
			os<<my_pathPlan.path_steps[i]<<std::endl<<std::endl;
		}
	}
	else{
		os<<red_on<<" Can't print behaviorinput::pathPlan as total step count = "<<color_off<<my_pathPlan.total_step_count<<std::endl;
	}
	return os;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::trafficsignalinterpretation_t& my_traffic_signa_interpretation){
	os<<" intersection_uid "<<my_traffic_signa_interpretation.intersection_uid<<" connection_count "<<my_traffic_signa_interpretation.connection_count<<std::endl;
	for(int i=0;i<my_traffic_signa_interpretation.connection_count;i++){
		os<<my_traffic_signa_interpretation.connection_traffic_signal_state[i]<<std::endl;
	}

	return os;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::trafficsigninterpretation_t& my_traffic_sign_interpretation){
	os<<" intersection_uid "<<my_traffic_sign_interpretation.intersection_uid<<" connection_count "<<my_traffic_sign_interpretation.connection_count<<std::endl;
	for(int i=0;i<my_traffic_sign_interpretation.connection_count;i++){
		os<<my_traffic_sign_interpretation.connection_traffic_sign[i]<<std::endl;
	}

	return os;
}


std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::connectiontrafficsignal_t& my_connection_traffic_signal){
	os<<" incoming_laneseg_uid "<<my_connection_traffic_signal.incoming_laneseg_uid<<" connection_n "<<my_connection_traffic_signal.connection_n<<" content_validity "<<(int)my_connection_traffic_signal.content_validity<<std::endl;
	os<<" signal_state ";
	lcmprint::PrintEnum(os,(traffic_signal_state)my_connection_traffic_signal.signal_state[0]);
	os<<" signal_probability "<<my_connection_traffic_signal.signal_probability[0]<<std::endl;
	os<<" time_in_phase_sec "<<my_connection_traffic_signal.time_in_phase_sec<<" remaining_time_in_phase_sec "<<my_connection_traffic_signal.remaining_time_in_phase_sec<<std::endl;
	return os;

}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::connectiontrafficsign_t& my_connection_traffic_sign){
	os<<" incoming_laneseg_uid "<<my_connection_traffic_sign.incoming_laneseg_uid<<" connection_n "<<my_connection_traffic_sign.connection_n<<std::endl;
	os<<" signal_state ";
	//lcmprint::PrintEnum(os,(traffic_sign_state)my_connection_traffic_sign.sign[0]);
	os<<" signal_probability "<<my_connection_traffic_sign.sign_probability[0]<<std::endl;
	os<<" station_m "<<my_connection_traffic_sign.station_m[0]<<std::endl;
	return os;

}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const traffic_signal_state my_traffic_signal_State){
	switch(my_traffic_signal_State)
	{
	PrintPurple(TRAF_SIG_STATE_NO_DETECTION);
	PrintPurple(TRAF_SIG_STATE_RED);
	PrintPurple(TRAF_SIG_STATE_YELLOW);
	PrintPurple(TRAF_SIG_STATE_RED_YELLOW);
	PrintPurple(TRAF_SIG_STATE_GREEN);
	PrintPurple(TRAF_SIG_STATE_GREEN_YELLOW);
	PrintPurple(TRAF_SIG_STATE_FLASHING_BIT);
	PrintPurple(TRAF_SIG_STATE_FLASHING_RED);
	PrintPurple(TRAF_SIG_STATE_FLASHING_YELLOW);
	PrintPurple(TRAF_SIG_STATE_FLASHING_GREEN);
	PrintPurple(TRAF_SIG_STATE_UP_DOWN_ARROW_BIT);
	PrintPurple(TRAF_SIG_STATE_STRAIGHT_ARROW_RED);
	PrintPurple(TRAF_SIG_STATE_STRAIGHT_ARROW_YELLOW);
	PrintPurple(TRAF_SIG_STATE_STRAIGHT_ARROW_GREEN);
	PrintPurple(TRAF_SIG_STATE_LEFT_ARROW_BIT);
	PrintPurple(TRAF_SIG_STATE_LEFT_ARROW_RED);
	PrintPurple(TRAF_SIG_STATE_LEFT_ARROW_YELLOW);
	PrintPurple(TRAF_SIG_STATE_LEFT_ARROW_GREEN);
	PrintPurple(TRAF_SIG_STATE_FLASHING_LEFT_ARROW_RED);
	PrintPurple(TRAF_SIG_STATE_FLASHING_LEFT_ARROW_YELLOW);
	PrintPurple(TRAF_SIG_STATE_FLASHING_LEFT_ARROW_GREEN);
	PrintPurple(TRAF_SIG_STATE_RIGHT_ARROW_GREEN);
	PrintPurple(TRAF_SIG_STATE_RIGHT_ARROW_BIT);
	PrintPurple(TRAF_SIG_STATE_RIGHT_ARROW_RED);
	PrintPurple(TRAF_SIG_STATE_RIGHT_ARROW_YELLOW);
	PrintPurple(TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_GREEN);
	PrintPurple(TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_RED);
	PrintPurple(TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_YELLOW);
	PrintPurple(TRAF_SIG_RAILROAD_CROSSING_UNKNOWN);
	PrintPurple(TRAF_SIG_RAILROAD_CROSSING_RED);
	PrintPurple(TRAF_SIG_RAILROAD_CROSSING_CLEAR);
	PrintPurple(TRAF_SIG_RAILROAD_CROSSING_BLOCKED);
	PrintPurple(TRAF_SIG_PEDISTRIAN_SAME_DIR_UNKNOWN);
	PrintPurple(TRAF_SIG_PEDISTRIAN_SAME_DIR_STOPPED);
	PrintPurple(TRAF_SIG_PEDISTRIAN_SAME_DIR_WARNING);
	PrintPurple(TRAF_SIG_PEDISTRIAN_SAME_DIR_CROSS);
	PrintPurple(TRAF_SIG_PEDISTRIAN_PERP_DIR_UNKNOWN);
	PrintPurple(TRAF_SIG_PEDISTRIAN_PERP_DIR_STOPPED);
	PrintPurple(TRAF_SIG_PEDISTRIAN_PERP_DIR_WARNING);
	PrintPurple(TRAF_SIG_PEDISTRIAN_PERP_DIR_CROSS);
	PrintPurple(TRAF_SIG_UNKNOWN);
	PrintPurple(TRAF_SIG_OFF);
	default: {os<<purple_on<<"Not sure. TRAF_SIG value is "<<(int)my_traffic_signal_State<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}


std::ostream& lcmprint::PrintEnum(std::ostream &os, const route_step_task_enum my_step_task){
	switch(my_step_task)
	{
	PrintBlue(SHUTDOWN_TASK);
	PrintBlue(STANDBY_TASK);
	PrintBlue(PROCEED_TASK);
	PrintBlue(CHANGE_LANES_TASK);
	PrintBlue(MERGE_TASK);
	PrintBlue(EXIT_TASK);
	PrintBlue(TURN_TASK);
	PrintBlue(REVERSE_TASK);
	PrintBlue(REVERSE_TURN_TASK);
	PrintBlue(PROCEED_AFTER_LANE_CHANGE_TASK);
	PrintBlue(PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK);
	PrintBlue(PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK);
	PrintBlue(PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK);
	PrintBlue(PROCEED_THROUGH_PARKING_AISLE_TASK);
	default: {os<<blue_on<<"Not sure. step_task Value is "<<(int)my_step_task<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os,  const right_of_way_rule_enum my_step_rule){

	switch(my_step_rule)
	{
	PrintPurple(ASSUME_RIGHT_OF_WAY_RULE);
	PrintPurple(YIELD_UNTIL_CLEAR_RULE);
	PrintPurple(STOP_AND_YIELD_UNTIL_CLEAR_RULE);
	PrintPurple(HANDLE_MULTIWAY_STOP_RULE);
	PrintPurple(ASSUME_RIGHT_OF_WAY_AFTER_SIGNAL_RULE);
	PrintPurple(YIELD_UNTIL_CLEAR_AFTER_SIGNAL_RULE);
	PrintPurple(RIGHT_ON_RED_RULE);
	PrintPurple(ASSUME_RIGHT_OF_WAY_AFTER_RAIL_CROSSING_CLEAR_RULE);
	PrintPurple(MERGE_AT_TRAFFIC_SPEED_RULE);
	PrintPurple(EXIT_AT_TRAFFIC_SPEED_RULE);
	PrintPurple(CHANGE_LANES_RULE);
	PrintPurple(OBEY_POSTED_SIGN_RULE);
	PrintPurple(OBEY_SIGN_IF_POSTED_RULE);
	PrintPurple(FOLLOW_INTERSECTION_RULE);
	PrintPurple(YIELD_FOR_PEDESTRIAN_RULE);
	default: {os<<blue_on<<"Not sure. step_rule Value is "<<(int)my_step_rule<<std::endl;break;}
	os<<std::endl ;
	}
	return os;
}



std::ostream& lcmprint::PrintEnum(std::ostream &os, const behavior_type My_behavior_type){
	switch(My_behavior_type)
	{
	PrintBlue(ACC_BEHAVIOR);
	PrintBlue(LANE_CHANGE_BEHAVIOR);
	PrintBlue(LANE_KEEPING_BEHAVIOR);
	PrintBlue(TURN_LEFT_BEHAVIOR);
	PrintBlue(TURN_RIGHT_BEHAVIOR);
	PrintBlue(TURN_ALONG_CURVY_ROAD_BEHAVIOR);
	PrintBlue(YIELD_TO_PEDESTRIAN_BEHAVIOR);
	PrintBlue(YIELD_TO_CROSS_TRAFFIC_BEHAVIOR);
	PrintBlue(HANDLE_INTERSECTION_BEHAVIOR);
	PrintBlue(STOP_AT_STOP_BAR_BEHAVIOR);
	PrintBlue(ENTERING_ROUNDABOUT_BEHAVIOR);
	PrintBlue(EXITING_ROUNDABOUT_BEHAVIOR);
	PrintBlue(MERGING_BEHAVIOR);
	PrintBlue(EXITING_FLOW_BEHAVIOR);
	PrintBlue(EVASIVE_LANE_CHANGE_BEHAVIOR);
	PrintBlue(STOP_AT_TRAFFIC_LIGHT_BEHAVIOR);
	PrintBlue(COLLISION_IMMINENT_BEHAVIOR);
	PrintBlue(UNKNOWN_BEHAVIOR);
	default: {os<<blue_on<<"UNKNOWN_BEHAVIOR_IS_ACTIVE. behaviorType value is "<<(int)My_behavior_type<<std::endl;break;}
	}
	return os;
}

std::ostream& lcmprint::PrintEnum(std::ostream &os, const active_maneuver_enum my_active_maneuver){
	switch(my_active_maneuver)
	{
	case NO_MANEUVER: {os<<"NO_MANEUVERc";break;}
	case LANE_KEEPING_MANEUVER: {os<<"LANE_KEEPING_MANEUVER\n";break;}
	case LANE_FOLLOWING_AND_CHANGING_MANEUVER: {os<<"LANE_FOLLOWING_AND_CHANGING_MANEUVER\n";break;}
	case INTERSECTION_DRIVING_MANEUVER: {os<<"INTERSECTION_DRIVING_MANEUVER\n";break;}
	case MERGE_MANEUVER: {os<<"MERGE_MANEUVER\n";break;}
	case MANEUVER_EXIT: {os<<"MANEUVER_EXIT\n";break;}
	case U_TURN_MANEUVER: {os<<"U_TURN_MANEUVER\n";break;}
	default: {os<<"UNKNOWN_MANEUVER_IS_ACTIVE. Value is "<<(int)my_active_maneuver<<std::endl;break;}
	}
	return os;
}


std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::behaviorplan_t& my_behaviorPlan)
{

	os<<" behaviorPlan behaviorType = ";
	PrintEnum(os,(behavior_type)my_behaviorPlan.behavior_type);

	//os<<" behaviorPlan Active Manuever = ";
	//PrintEnum(os,(active_maneuver_enum)my_behaviorPlan.active_maneuver);
	os<<" behaviorPlan Accel Urgency Type  = ";
	PrintEnum(os,(behavior_accel_request)my_behaviorPlan.acceleration_urgency);

	os<<" speed_target_current_lane_mps = " <<my_behaviorPlan.speed_target_current_lane_mps<<" speed_target_distance_current_lane_m= "<<my_behaviorPlan.speed_target_distance_current_lane_m<<\
			" accel_target_current_lane_mpss "<<my_behaviorPlan.accel_target_current_lane_mpss<<std::endl;
	//os<<" LCZ start "<<my_behaviorPlan.lane_change_zone_start_m<<" LCZ end "<<my_behaviorPlan.lane_change_zone_end_m;
			os<<" left_LCX_intention "<<my_behaviorPlan.left_lane_intention<<" right_LCX_intention "<<my_behaviorPlan.right_lane_intention<<std::endl;
	//os<<" speed_target_distance_left_lane_m "<<my_behaviorPlan.speed_target_distance_left_lane_m<<" speed_target_distance_right_lane_m "<<my_behaviorPlan.speed_target_distance_right_lane_m<<\
			" speed_target_left_lane_mps "<<my_behaviorPlan.speed_target_left_lane_mps<<" speed_target_right_lane_mps "<<my_behaviorPlan.speed_target_right_lane_mps;
	os<<" reroute needed ";
	lcmprint::PrintEnum(os,(reroute_type_enum)my_behaviorPlan.reroute_needed_f);
	//os<<" relevant_obj_IDs "<<my_behaviorPlan.relevant_obj_IDs<<" relevant obj count "<<my_behaviorPlan.relevant_obj_count<<std::endl;

	return os;
}


std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::lanesituation_t& MyLane ){
	os<<" ---------------- LANE SITUATION INFO BEGIN  ----------------------------------"<<std::endl;
	os<<" Lane SA ";
	lcmprint::PrintEnum(os,(lane_situation_enum)MyLane.lanesituation_index);

	os<<" In Lane objects: "<<std::endl;
	if(MyLane.obj_count==0){
		os<<" No In lane objects found "<<std::endl;
	}
	else{
		for(int i=0;i<MyLane.obj_count;i++){
			os<<"obj["<<MyLane.obj_IDs[i]<<"] dist: "<<MyLane.obj_dist[i]<<"  ,  ";
		}
	}

	os<<" Parallel objects: "<<std::endl;
	if(MyLane.parallel_obj_count==0){
		os<<" No parallel lane objects found "<<std::endl;
	}
	else{
		for(int i=0;i<MyLane.parallel_obj_count;i++){
			os<<"obj["<<MyLane.parallel_obj_uid[i]<<"] clear_dist_ahead: "<<MyLane.clear_dist_ahead_parallel[i]<<"   ,   ";
		}
	}

	os<<" Merge objects: "<<std::endl;
	if(MyLane.merge_obj_count==0){
		os<<" No merge objects found "<<std::endl;
	}
	else{
		for(int i=0;i<MyLane.merge_obj_count;i++){
			os<<"obj["<<MyLane.merge_obj_uid[i]<<"] clear_dist_ahead: "<<MyLane.clear_dist_ahead_merge[i]<<"   ,   ";
		}
	}

	os<<" Cross objects: "<<std::endl;
	if(MyLane.cross_obj_count==0){
		os<<" No cross objects found "<<std::endl;
	}
	else{
		for(int i=0;i<MyLane.cross_obj_count;i++){
			os<<"obj["<<MyLane.cross_obj_uid[i]<<"] clear_dist_ahead: "<<MyLane.clear_dist_ahead_cross[i]<<"   ,   ";
		}
	}

	os<<" ---------------- LANE SITUATION INFO END  ----------------------------------"<<std::endl;

	return os;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const std::vector<exlcm::lanesituation_t>& MyLanes ){

	for (auto &MyLane:MyLanes){
		os<<MyLane<<std::endl;
	}
	return os;

}



std::ostream& lcmprint::operator<<(std::ostream& os, const exlcm::situation_t& MyHV )
{
	os<<" ---------------- SITUATION INFO BEGIN  ----------------------------------"<<std::endl;

	os<<" ---------------- SITUATION INFO END  ----------------------------------"<<std::endl;

	return os;
}

std::ostream& lcmprint::operator<<(std::ostream& os, const exlcm::objsituation_t& MyObj )
{
	os<<" Obj uid = "<<MyObj.uid<<std::endl;

	for(int i=0;i<MyObj.valid_pred_count;i++){
		os<<"PREDICTION "<<i<<std::endl;
		os<<MyObj.pred_obj_SA[i]<<std::endl<<std::endl;
	}
	return os;
}

std::ostream& lcmprint::operator<<(std::ostream& os, const exlcm::objprediction_t& MyObj )
{
	os<<" intersection_uid = "<<MyObj.intersection_uid<<" X intersection_uid = "<<MyObj.intersecting_intersection_id<<" lane_segment_uid = "<<MyObj.lane_segment_uid<<std::endl;
	os<<" collision_possible = "<<(double)MyObj.collision_possible<<" host_dist_to_conflict_m = "<<MyObj.host_dist_to_conflict_m<<" dist_to_conflict_m = "\
			<<MyObj.dist_to_conflict_m<<
			//" host_time_to_clear_conflict_sec = "<<MyObj.host_time_to_clear_conflict_sec<<" time_to_clear_conflict_sec = "<<Mybj.time_to_clear_conflict_sec<<
						" time_to_collision_sec = "<<MyObj.time_to_collision_sec<<" time_to_intersection_sec = "<<\
			MyObj.time_to_intersection_sec<<" time_at_intersection_sec = "<<MyObj.time_at_intersection_sec<<" lane_station_m = "<<MyObj.lane_station_m<<std::endl;
	os<<" X_m = "<<MyObj.x_m<<" Y_m = "<<MyObj.y_m<<std::endl;
	os<<" Speeed "<<MyObj.in_lane_speed_mps<<" Accel = "<<MyObj.in_lane_accel_mpss<<std::endl;
	os<<" expected_to_yield_f "<<purple_on;
	switch(MyObj.expected_to_yield_f)
	{
	PrintPurple(YIELD_DUE_TO_STATIC_INFERIORITY);
	PrintPurple(DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY);
	PrintPurple(YIELD_DUE_TO_ARRIVAL_INFERIORITY);
	PrintPurple(DO_NOT_YIELD_DUE_TO_ARRIVAL_SUPERIORITY);
	PrintPurple(YIELD_SITUATION_AMBIGUOUS);
	PrintPurple(DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY_RV_AT_INT);
	PrintPurple(YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT);
	PrintPurple(DO_NOT_YIELD_DUE_TO_PERCEIVED_ARRIVAL_SUPERIORITY);
	PrintPurple(DO_NOT_YIELD_IN_VIOLATION_OF_TRAFFIC_RULES);
	default: {os<<"YIELD Type Unknown. Value is "<<MyObj.expected_to_yield_f<<std::endl;break;}
	}
	os<<color_off;
	os<<" relative_location "<<purple_on;
	os<<(eSAObjLocation)MyObj.relative_location<<color_off;
	os<<"Stationary Status ";lcmprint::PrintEnum(os,(sa_obj_stationary_status_enum)MyObj.Stationary_Status);

	return os;
}



std::ostream& lcmprint::operator<<(std::ostream& os, const exlcm::intersection_t &MyInt )
{
	os<<" intersection UID = "<<MyInt.uid<<" center_east_m = "<<MyInt.center_east_m<<" center_north_m = "<<MyInt.center_north_m<<std::endl;
	os<<" Intersection_type = ";
	switch(MyInt.Intersection_type)
	{
	case SIDE_STREET_INTERSECTION_TYPE: {os<<"SIDE_STREET_INTERSECTION_TYPE\n";break;}
	case CROSSROADS_INTERSECTION_TYPE: {os<<"CROSSROADS_INTERSECTION_TYPE\n";break;}
	case MERGE_INTERSECTION_TYPE: {os<<"MERGE_INTERSECTION_TYPE\n";break;}
	case EXIT_INTERSECTION_TYPE: {os<<"EXIT_INTERSECTION_TYPE\n";break;}
	case FORK_INTERSECTION_TYPE: {os<<"FORK_INTERSECTION_TYPE\n";break;}
	case TRAFFIC_CIRCLE_INTERSECTION_TYPE: {os<<"TRAFFIC_CIRCLE_INTERSECTION_TYPE\n";break;}
	default: {os<<"UNKNOWN_INTERSECTION_TYPE. Value is "<<MyInt.Intersection_type<<std::endl;break;}
	}
	return os;
}



std::ostream& lcmprint::operator<<(std::ostream& os,const eSAObjLocation &my_eSAObjLocation){
	switch(my_eSAObjLocation){
	PrintYellow(SA_NOT_NEAR);
	PrintYellow(SA_AHEAD);
	PrintYellow(SA_BEHIND);
	PrintYellow(SA_LEFT);
	PrintYellow(SA_RIGHT);
	PrintYellow(SA_AHEAD_LEFT);
	PrintYellow(SA_AHEAD_RIGHT);
	PrintYellow(SA_BEHIND_RIGHT);
	PrintYellow(SA_BEHIND_LEFT);
	PrintYellow(SA_AHEAD_FAR_LEFT);
	PrintYellow(SA_AHEAD_FAR_RIGHT);
	PrintYellow(SA_BEHIND_FAR_LEFT);
	PrintYellow(SA_BEHIND_FAR_RIGHT);
	PrintYellow(SA_ONCOMING);
	PrintYellow(SA_ONCOMING_LEFT);
	PrintYellow(SA_ONCOMING_RIGHT);
	PrintYellow(SA_ONCOMING_FAR_LEFT);
	PrintYellow(SA_ONCOMING_FAR_RIGHT);
	PrintYellow(SA_INTERSECTING_FROM_LEFT);
	PrintYellow(SA_INTERSECTING_FROM_RIGHT);
	PrintYellow(SA_MERGING_FROM_LEFT);
	PrintYellow(SA_MERGING_FROM_RIGHT);
	PrintYellow(SA_INTERSECTING_FROM_LEFT_AHEAD);
	PrintYellow(SA_INTERSECTING_FROM_RIGHT_AHEAD);
	PrintYellow(SA_MERGING_FROM_LEFT_AHEAD);
	PrintYellow(SA_MERGING_FROM_RIGHT_AHEAD);
	PrintYellow(SA_AHEAD_RIGHT_EDGE);
	PrintYellow(SA_AHEAD_LEFT_EDGE);
	PrintYellow(SA_BEHIND_RIGHT_EDGE);
	PrintYellow(SA_BEHIND_LEFT_EDGE);
	default: {os<<"UNKNOWN SAObjLocation. Value is "<<my_eSAObjLocation<<std::endl;break;}
	}
	return os;
}

void lcmprint::Printbehaviorpolicycosts(std::ostream &os,const exlcm::behaviorpolicycosts_t& Mybehaviorpolicycost,std::string indent){
	os<<" BehaviorType =  "<<Mybehaviorpolicycost.BehaviorType;
	os<<indent<<" SpatialTask =  "<<Mybehaviorpolicycost.SpatialTask;
	os<<indent<<" TemporalRule = "<<Mybehaviorpolicycost.TemporalRule;
	os<<indent<<" Behavior Policy cost = "<<Mybehaviorpolicycost.PolicyCost;
	os<<indent<<" SpatialManeuverCost = "<<Mybehaviorpolicycost.SpatialManeuverCost;
	os<<indent<<" Temporal Maneuver cost = "<<Mybehaviorpolicycost.TemporalManeuverCost;
	//os<<endl<<endl;
}

std::ostream& lcmprint::operator<<(std::ostream &os, const exlcm::routesegmentlist_t& my_routeSegmentList){

	for(int i=0;i<my_routeSegmentList.segment_count;i++){
		os<<"    "<<blue_on<<my_routeSegmentList.segment_group[i]<<"  "<<color_off<<my_routeSegmentList.segment_uid[i];
	}

	os<<std::endl;
	return os;

}


std::ostream& bold_on(std::ostream& os)
{
	return os << "\e[1m";
}

std::ostream& bold_off(std::ostream& os)
{
	return os << "\e[0m";
}

std::ostream& red_on(std::ostream& os)
{
	return os << "\033[1;31m";
}

std::ostream& green_on(std::ostream& os)
{
	return os << "\033[1;32m";
}

std::ostream& yellow_on(std::ostream& os)
{
	return os << "\033[1;33m";
}

std::ostream& blue_on(std::ostream& os)
{
	return os << "\033[1;34m";
}

std::ostream& purple_on(std::ostream& os)
{
	return os << "\033[1;35m";
}


std::ostream& color_off(std::ostream& os)
{
	return os << "\033[0m";
}
