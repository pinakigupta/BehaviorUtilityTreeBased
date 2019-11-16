#include "../include/Env.hpp"
#include "../include/lcm_def_includes.hpp"
#include "../include/BehaviorUtils.hpp"
#include "../include/io_handler.hpp"
#include "../include/lcmprint.hpp"
#include "../include/BehaviorTertiary.hpp"
#include "../include/behaviorHorizon.hpp"
#include "../include/behaviorprint.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"


namespace BU=BehaviorUtils;


std::ostream& Env::operator <<(std::ostream &os, const exlcm::pathstep_t &MyStep)
{



	os << "step_index_n = " << MyStep.step_index_n << "   current_segment_group = " << MyStep.current_segment_group<< "  target_segment_group = " << MyStep.target_segment_group<<'\n';
	os<<"  current_segment_station_m = "<<MyStep.current_segment_station_m<<"  target_segment_station_m = "<<MyStep.target_segment_station_m;
	os<<"  posted_speed_lim_mps "<<MyStep.posted_speed_lim_mps<<" Min Time to destination " <<MyStep.min_time_to_destination_s<<'\n';
	os<<" step_task = ";
	lcmprint::PrintEnum(os,(route_step_task_enum)MyStep.step_task);
	os<<" step_rule = ";
	PrintEnum(os,(TempRules)MyStep.step_rule);
	os<<" step_direction = ";
	lcmprint::PrintEnum(os,(direction_enum)MyStep.direction);


	return os;
}

std::ostream& Env::operator <<(std::ostream &os, const Env::PolicyState &MyState)
{
	os << MyState.step;
	os<<" Observed step_rule(s) = { ";
	for(auto rules:MyState.observed_step_rule){
		lcmprint::PrintEnum(os,(right_of_way_rule_enum)rules);
		os<<"  ";
	}
	os<<"}";
	os<<" Current segment lane(s) = "<<MyState.CurrentGrpLanes<<" target segment lane(s) = "<<MyState.TargetGrpLanes<<'\n';
	os<<" segment Length = "<<MyState.CurrentGrpLen<<" Target segment Length = "<<MyState.TgtGrpLen<<" Connection Length = "<<MyState.CurrentToTgtGrpConnLen<<'\n';
	os << red_on<<"DAR_m = "<< MyState.DAR_m<<" Current Grp Origin DAR_m = "<<MyState.GrpOrigin_DAR_m<<" Current Grp End DAR_m = "<<MyState.GrpEnd_DAR_m<<" DTAR = "<< vehicle.DTAR<<color_off<<endl;
	os<<"  Min Travel Time "<<MyState.MinTravelTm<<\
			"  Is this opposing lane ? "<<MyState.SuspectedOpposingGrp<<'\n' ;

	return os;
}

std::ostream& Env::operator <<(std::ostream &os, const PolicyDequeType &MyDeque)
{

	if(MyDeque.empty()){
		os<<"Deque is empty"<<'\n';
		return os;
	}
	for (int i=0;i<MyDeque.size();i++){
		os << MyDeque.at(i) << '\n'<<'\n';
	}
	return os;
}



std::ostream& BehaviorTertiary::operator<<(std::ostream &os, const set<behavior_type>& My_behavior_type_set){
	os<<"{  ";
	for(const auto& it:My_behavior_type_set){
		os<<it<<"    ";
	}
	os<<"  }  "<<'\n';
	return os;
}



std::ostream& BehaviorTertiary::operator<<(std::ostream &os, const BehaviorTertiary::Behavior& myBehavior)
{

	os<<"-----------------------------------BEGIN ---------------------------------------------"<<'\n';

	os<<"  behaviorType = ";
	lcmprint::PrintEnum(os,(behavior_type)myBehavior.behaviorType);

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal = myBehavior;
	BehaviorTertiary::Maneuver_Spatial My_Maneuver_Spatial = myBehavior;
	os<<'\n'<<" Behavior Maneuver_Spatial: "<<'\n';
	os<<My_Maneuver_Spatial;
	os<<'\n'<<" Behavior Maneuver_Temporal: "<<'\n';
	os<<My_Maneuver_Temporal;

	os<<"Behavior Target State: "<<myBehavior.TargetState<<'\n';

	try{
		PrintSpatialTaskGroup(os,myBehavior.SpatialTask);
	}catch(...){
		os<<" Error occured while printing Behavior SpatialTaskGroup"<<'\n';
	}


	os<<endl;
	os<<purple_on<<" BehaviorDesired Cost Tree "<<endl<<color_off;
	Policy::operator<<(os,myBehavior.BehaviorPolicyCosts);

	os<<endl<<endl;

	os<<"-----------------------------------END ---------------------------------------------"<<'\n';
	return os;
}



std::ostream& BehaviorTertiary::operator<<(std::ostream &os, const BehaviorTertiary::Maneuver_Temporal& My_Maneuver_Temporal)
{
	os << "ProjectedAcceleration = "<<My_Maneuver_Temporal.ProjectedAcceleration <<yellow_on<<" Maneuver Temporal Cost = "<<color_off<<My_Maneuver_Temporal.Temporal_cost<<\
			yellow_on<<" Maneuver Temporal Policy Cost = "<<color_off<<My_Maneuver_Temporal.Maneuver_Temporal::PolicyCost<<\
			"  speedTargetDistance = " <<My_Maneuver_Temporal.speedTargetDistance<< "  speedTargetDistanceActual = " <<My_Maneuver_Temporal.speedTargetDistanceActual << "  speedTarget= "\
			<< My_Maneuver_Temporal.speedTarget<<" Current HV speed = "<<vehicle.curSpeed_mps<<" primary_target_object ID " <<My_Maneuver_Temporal.primary_target_object<<'\n' ;
	os<< " Temporal TargetSet = ";
	if(My_Maneuver_Temporal.TemporalTargetsSet)
		os<<" TRUE";
	else
		os<<" FALSE ";
	try{
		os<<" Temporal Rule = ";
		lcmprint::PrintEnum(os,My_Maneuver_Temporal.TemporalRule);
		//BehaviorTertiary::Print(cout,(TempRules)My_Maneuver_Temporal.TemporalRule);
		os<<'\n' ;
	}catch(...){
		os<<" Error occured while printing Temporal Rule"<<'\n';
	}
	return os;
}

std::ostream& BehaviorTertiary::operator<<(std::ostream &os, const BehaviorTertiary::Maneuver_Spatial& My_Maneuver_Spatial)
{
	os << "leftLaneChangeIntention = ";
	lcmprint::PrintEnum(os,(lane_intention_enum)My_Maneuver_Spatial.leftLaneChangeIntention) ;
	os<<"  rightLaneChangeIntention = " ;
	lcmprint::PrintEnum(os,(lane_intention_enum)My_Maneuver_Spatial.rightLaneChangeIntention) ;
	os<<"  laneChangeZoneStart_m= "<< My_Maneuver_Spatial.laneChangeZoneStart_m<<" laneChangeZoneEnd_m = "<<My_Maneuver_Spatial.laneChangeZoneEnd_m;
	os<<"  TgtlaneChangeZoneStart_m= "<< My_Maneuver_Spatial.TgtlaneChangeZoneStart_m<<" TgtlaneChangeZoneEnd_m = "<<My_Maneuver_Spatial.TgtlaneChangeZoneEnd_m<<yellow_on<<\
			" Total Spatial Maneuver cost = "<<color_off<<My_Maneuver_Spatial.Spatial_cost<<yellow_on<<" Maneuver Spatial Policy Cost = "<<color_off<<My_Maneuver_Spatial.Maneuver_Spatial::PolicyCost<<'\n' ;
	os<<" Spatial Task = ";
	lcmprint::PrintEnum(os,(route_step_task_enum)My_Maneuver_Spatial.SpatialTask);
	os<<" reRouteNeeded = ";
	lcmprint::PrintEnum(os,(reroute_type_enum)My_Maneuver_Spatial.reRouteNeeded);
	os<<" Alternate route count  = "<<My_Maneuver_Spatial.alternaterouteNeeded.size();
	if(My_Maneuver_Spatial.alternaterouteNeeded.size()>0){
		os<<" alternaterouteNeeded = ";
		for(auto route:My_Maneuver_Spatial.alternaterouteNeeded)
			lcmprint::PrintEnum(os,(reroute_type_enum)route);
	}

	os<<"  Behavior Turn indicator distance = "<<My_Maneuver_Spatial.turn_indicator_target_distance;
	try{
		os<<" Behavior Turn indicator request = ";
		switch(My_Maneuver_Spatial.turn_indicator_command)
		{
		case TURN_INDICATOR_LEFT: {os<<green_on<<"TURN_INDICATOR_LEFT"<<color_off<<'\n';break;}
		case TURN_INDICATOR_RIGHT: {os<<green_on<<"TURN_INDICATOR_RIGHT"<<color_off<<'\n';break;}
		case TURN_INDICATOR_EMERGENCY_FLASH: {os<<green_on<<"TURN_INDICATOR_EMERGENCY_FLASH"<<color_off<<'\n';break;}
		case TURN_INDICATOR_NONE: {os<<green_on<<"TURN_INDICATOR_NONE"<<color_off<<'\n';break;}
		case TURN_INDICATOR_UNKNOWN: {os<<green_on<<"TURN_INDICATOR_UNKNOWN"<<color_off<<'\n';break;}
		default: {os<<blue_on<<"Not sure. Maneuver Spatial  Turn indicator request  is "<<My_Maneuver_Spatial.turn_indicator_command<<'\n';break;}
		}
	}catch(...){
		os<<" Error occured while printing Maneuver Spatial  Turn indicator request"<<'\n';
	}
	os<<"Direction";
	lcmprint::PrintEnum(os,(direction_enum)My_Maneuver_Spatial.direction);
	os<<'\n' ;

	return os;
}

