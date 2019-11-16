#pragma once

#include "../PCH/pch.hpp"
#include "shared_enums.h"
#include "lcm_def_includes.hpp"
#include "BehaviorEnums.h"

#define PrintPurple(VAR) 	case VAR: {os<<purple_on<<#VAR<<color_off<<"\n";break;}
#define PrintBlue(VAR) 	case VAR: {os<<blue_on<<#VAR<<color_off<<"\n";break;}
#define PrintRed(VAR) 	case VAR: {os<<red_on<<#VAR<<color_off<<"\n";break;}
#define PrintYellow(VAR) 	case VAR: {os<<yellow_on<<#VAR<<color_off<<"\n";break;}


namespace lcmprint{

extern std::ostream& operator<<(std::ostream& os, const exlcm::policycosts_t& );
extern std::ostream& operator<<(std::ostream &os, const exlcm::pathstep_t& my_pathstep);
extern std::ostream& operator<<(std::ostream &os, const exlcm::pathplan_t& my_pathPlan);
extern std::ostream& operator<<(std::ostream &os, const exlcm::behaviorplan_t& );
extern std::ostream& operator<<(std::ostream &os, const exlcm::lanesituation_t& MyLane );
extern std::ostream& operator<<(std::ostream &os, const std::vector<exlcm::lanesituation_t>& MyLanes );
extern std::ostream& operator<<(std::ostream &os, const exlcm::situation_t& MyHV );
extern std::ostream& operator<<(std::ostream &os, const exlcm::objsituation_t& MyObj );
extern std::ostream& operator<<(std::ostream &os, const exlcm::objprediction_t& MyObj );
extern std::ostream& operator<<(std::ostream &os, const exlcm::intersection_t& MyInt );
extern std::ostream& operator<<(std::ostream& os, const eSAObjLocation &my_eSAObjLocation);
extern std::ostream& operator<<(std::ostream &os, const exlcm::routesegmentlist_t& my_routeSegmentList);
extern std::ostream& operator<<(std::ostream &os, const exlcm::connectiontrafficsignal_t& my_connection_traffic_signal);
extern std::ostream& operator<<(std::ostream &os, const exlcm::connectiontrafficsign_t& my_connection_traffic_sign);
extern std::ostream& operator<<(std::ostream &os, const exlcm::trafficsignalinterpretation_t& my_traffic_signal_interpretation);
extern std::ostream& operator<<(std::ostream &os, const exlcm::trafficsigninterpretation_t& my_traffic_sign_interpretation);
extern std::ostream& operator<<(std::ostream &os, const exlcm::useroutputs_t& my_useroutputs);
extern std::ostream& operator<<(std::ostream &os, const exlcm::localization_t &);
extern void Printbehaviorpolicycosts(std::ostream &os,const exlcm::behaviorpolicycosts_t&, std::string indent = "");


extern std::ostream& PrintEnum(std::ostream &os, const behavior_type );
extern std::ostream& PrintEnum(std::ostream &os, const active_maneuver_enum  );
extern std::ostream& PrintEnum(std::ostream &os, const lane_intention_enum );
extern std::ostream& PrintEnum(std::ostream &os, const behavior_accel_request );
extern std::ostream& PrintEnum(std::ostream &os, const reroute_type_enum );
extern std::ostream& PrintEnum(std::ostream &os, const route_step_task_enum );
extern std::ostream& PrintEnum(std::ostream &os, const right_of_way_rule_enum );
extern std::ostream& PrintEnum(std::ostream &os, const active_maneuver_enum  );
extern std::ostream& PrintEnum(std::ostream &os, const traffic_signal_state );
extern std::ostream& PrintEnum(std::ostream &os, const direction_enum  );
extern std::ostream& PrintEnum(std::ostream &os, const lane_change_feedback_enum  );
extern std::ostream& PrintEnum(std::ostream &os, const lane_situation_enum  );
extern std::ostream& PrintEnum(std::ostream &os, const TempRules My_rule);
extern std::ostream& PrintEnum(std::ostream &os, const sa_obj_stationary_status_enum My_stationary_state);



}
extern std::ostream& bold_on(std::ostream& os);
extern std::ostream& bold_off(std::ostream& os);
extern std::ostream& red_on(std::ostream& os);
extern std::ostream& blue_on(std::ostream& os);
extern std::ostream& green_on(std::ostream& os);
extern std::ostream& yellow_on(std::ostream& os);
extern std::ostream& purple_on(std::ostream& os);
extern std::ostream& color_off(std::ostream& os);

using namespace lcmprint;
