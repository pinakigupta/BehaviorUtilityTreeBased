#include "../PCH/pch.hpp"

#include "../include/Env.hpp"
#include "../include/shared_enums.h"
#include "../include/BehaviorTertiary.hpp"
#include "../include/lcmprint.hpp"

extern Vehicle::vehicleState  vehicle;
namespace BU=BehaviorUtils;

std::map<double,double> BehaviorTertiary::AccelFactorDefault = { {0.0, 1.0} , {30.0, 1.0}, {40.0, 0.9}, {50.0, 0.75}, {70.0, 0.5}, {90.0, 0.25} ,{120.0, 0.0}, {150.0, 0.0}   };
BehaviorTertiary::Maneuver_Temporal BehaviorTertiary::Default_Maneuver_Temporal;




BehaviorTertiary::Maneuver_Spatial::Maneuver_Spatial(Env::PolicyState &TargetState){
	auto MyTask = TargetState.step.step_task;

	this->reRouteNeeded = NO_REROUTE;
	this->leftLaneChangeIntention = LANE_CHANGE_IS_PROHIBITED;
	this->rightLaneChangeIntention = LANE_CHANGE_IS_PROHIBITED;
	this->laneChangeZoneStart_m = 0;
	this->laneChangeZoneEnd_m = 0;
	this->SpatialTask = MyTask;
	this->SpatialTargetsSet = true;
	this->Spatial_cost = 0.0;


	if (MyTask == TURN_TASK){
		*this = BehaviorElementary::turn(TargetState);
		if(SA.HVData.intersection_relevant_f||behaviorinput::hvLoc.inside_intersection_f)
			this->activeManeuver = INTERSECTION_DRIVING_MANEUVER ; // Task Turn doesn't include serpentile roads here. Thats included in lane keeping. So will assume turn means intersection.
	}
	else if(MyTask==PROCEED_AFTER_LANE_CHANGE_TASK){
		this->activeManeuver=LANE_KEEPING_MANEUVER;
		*this = BehaviorElementary::laneKeepBehavior(TargetState);
	}
	else if((MyTask == PROCEED_TASK)||(MyTask == PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK)||(MyTask == PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK)|| \
			(MyTask == PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK)||(MyTask == PROCEED_THROUGH_PARKING_AISLE_TASK)){
		this->activeManeuver=LANE_KEEPING_MANEUVER;
		*this = BehaviorElementary::laneKeepBehavior(TargetState);
	}
	else if(MyTask == CHANGE_LANES_TASK){
		*this = BehaviorElementary::PlannedlaneChange(TargetState);
		this->activeManeuver=LANE_FOLLOWING_AND_CHANGING_MANEUVER;
	}
	else if(MyTask == EXIT_TASK){
		*this = BehaviorElementary::ExitBehavior(TargetState);
		this->activeManeuver=MANEUVER_EXIT;
	}
	this->Maneuver_Spatial::CalculateCost();
}

BehaviorTertiary::Maneuver_Spatial::Maneuver_Spatial(active_maneuver_enum my_active_maneuver,lane_intention_enum my_LeftLCInt, \
		lane_intention_enum  my_rightLCInt, double my_LCZ_Start_m, double my_LCZ_End_m, \
		decltype(exlcm::pathstep_t::step_task) my_SpatialTask , short  my_TI_Cmnd ){
	this->leftLaneChangeIntention = my_LeftLCInt;
	this->rightLaneChangeIntention = my_rightLCInt;
	this->laneChangeZoneStart_m = my_LCZ_Start_m;
	this->laneChangeZoneEnd_m = my_LCZ_End_m;
	this->SpatialTask = my_SpatialTask;
	this->turn_indicator_command = my_TI_Cmnd;
	this->activeManeuver = my_active_maneuver;
	this->reRouteNeeded = NO_REROUTE;
	this->Spatial_cost = 0.0;
	this->Maneuver_Spatial::CalculateCost();
}

BehaviorTertiary::Maneuver_Spatial::Maneuver_Spatial()
{

	this->reRouteNeeded = NO_REROUTE;
	this->leftLaneChangeIntention = LANE_CHANGE_IS_PROHIBITED;
	this->rightLaneChangeIntention = LANE_CHANGE_IS_PROHIBITED;
	this->laneChangeZoneStart_m = 0;
	this->laneChangeZoneEnd_m = 0;
	this->TgtlaneChangeZoneStart_m = 0.0 ;
	this->TgtlaneChangeZoneEnd_m = 0.0;
	this->SpatialTask = STANDBY_TASK;
	this->Spatial_cost = 0.0;
	this->Maneuver_Spatial::CalculateCost();
}

BehaviorTertiary::Maneuver_Temporal::Maneuver_Temporal(Env::PolicyState &TargetState,std::pair<int,int> lane){
	auto Rule = TargetState.step.step_rule;
	this->TemporalRule = Rule;
	this->TemporalTargetsSet = true;
	if(Rule == FOLLOW_INTERSECTION_RULE){
		*this =  BehaviorTertiary::InterSectionManeuver(TargetState);
	}
	else if(Rule == YIELD_UNTIL_CLEAR_RULE){
		*this =  BehaviorTertiary::ControlComplexBehavior(TargetState,true);
	}
	else if(Rule == YIELD_FOR_PEDESTRIAN_RULE){
		*this =  BehaviorSecondary::CrossWalkBehavior(TargetState);
	}
	else if(Rule == YIELD_ZONE){
		*this =  BehaviorTertiary::YieldToTraffic(TargetState);
	}
	else if(Rule == STOP_AND_YIELD_UNTIL_CLEAR_RULE){
		*this = BehaviorSecondary::stopAtStopBar(TargetState);
	}
	else  if (Rule == STOP_AND_STAND_BY){
		*this = BehaviorSecondary::stopAndStandBy(TargetState);
	}
	else  if ((Rule == RIGHT_OF_WAY_AFTER_SIGNAL)||( Rule == YIELD_UNTIL_CLEAR_AFTER_SIGNAL)){
		*this = BehaviorSecondary::HandleTrafficSignals(TargetState);
	}
	else  if (Rule == RIGHT_OF_WAY_AFTER_SIGNAL_STOP_ZONE){
		*this = BehaviorSecondary::HandleTrafficSignals(TargetState,true);
	}
	else  if (Rule == RIGHT_ON_RED_RULE){
		if(RIGHT_ON_RED_ENABLE)
			*this = BehaviorSecondary::stopAtStopBar(TargetState);
		else
			*this = BehaviorSecondary::HandleTrafficSignals(TargetState);
	}
	else if(Rule==RIGHT_OF_WAY_AFTER_RAIL_CROSSING_CLEAR){
		*this = BehaviorElementary::laneKeepBehavior(TargetState); // For now as we also have a Stop sign being placed at the the railway crossing.
	}
	else if (Rule == EXIT) {
		*this = BehaviorElementary::ExitBehavior(TargetState);
	}
	else if (Rule == CHANGE_LANES) {
		*this = BehaviorElementary::PlannedlaneChange(TargetState);
	}
	else if (Rule == OBSERVED_MAYBE_TRAFFIC_SIGN) {
		*this = BehaviorSecondary::HandleObservedTrafficSigns(TargetState);
		*this = *this + BehaviorSecondary::HandleTrafficSignals(TargetState,false,true);
	}
	else if (Rule == OBSERVED_TRAFFIC_SIGN) {
		*this = BehaviorSecondary::HandleObservedTrafficSigns(TargetState);
		*this = *this + BehaviorSecondary::HandleTrafficSignals(TargetState,false,true);
		*this = *this + BehaviorElementary::turn(TargetState,DEC_VEL_TO_YIELD_MPS);
	}
	else if (Rule == RIGHT_OF_WAY ){
		try{
			this->speedTarget = TargetState.step.posted_speed_lim_mps;
		}catch(...){
			this->speedTarget = vehicle.speedTarget;
			this->speedTargetDistance = -999;
			this->BehaviorTertiary::Maneuver_Temporal::CalculateCost();
			return;
		}
		if(lane.first!=HV_LANE_SITUATION)
			*this = BehaviorElementary::AdjacentlaneKeepBehavior(TargetState,lane.first,lane.second);
		else
			*this = BehaviorElementary::laneKeepBehavior(TargetState);
	}
	else{ // ASSUME_RIGHT_OF_WAY_RULE
		this->speedTarget = vehicle.speedTarget;
		this->speedTargetDistance = vehicle.speedTargetDistance;
		this->BehaviorTertiary::Maneuver_Temporal::CalculateCost();

	}

	this->EstimatedTimeToTarget = (this->speedTarget-vehicle.curSpeed_mps)/(this->ProjectedAcceleration);

	if ((this->speedTargetDistance == 0.0) && (this->speedTarget == 0.0))
	{

	}
	else if (this->speedTargetDistance == 0.0)
	{
		//	exit(0);
	}
	else if ((this->speedTargetDistance == -999) && (this->speedTarget == -999))
	{

		this->speedTarget = TargetState.step.posted_speed_lim_mps;
		this->ProjectedAcceleration = BehaviorUtils::targetAcceleration(SA_LOOK_AHEAD_HORIZON,vehicle.curSpeed_mps,this->speedTarget);
	}
	else if (this->speedTargetDistance == -999)
	{
		this->ProjectedAcceleration = BehaviorUtils::targetAcceleration(SA_LOOK_AHEAD_HORIZON,vehicle.curSpeed_mps,speedTarget);
	}
	else if (this->speedTarget == -999)
	{
		this->speedTarget = TargetState.step.posted_speed_lim_mps;
		this->ProjectedAcceleration = BehaviorUtils::targetAcceleration(SA_LOOK_AHEAD_HORIZON,vehicle.curSpeed_mps,this->speedTarget);
	}
	else
	{
		this->ProjectedAcceleration = BehaviorUtils::targetAcceleration(this->speedTargetDistance,vehicle.curSpeed_mps,this->speedTarget);
	}

}



BehaviorTertiary::Maneuver_Temporal::Maneuver_Temporal()
{
	this->speedTarget = -999;
	this->speedTargetDistance = -999;
	this->AccelTarget = -999;
	this->ProjectedAcceleration = 0;
	this->TemporalRule = TempRules::NO_RULES;
	this->speedTargetDistanceActual = this->speedTargetDistance;
	this->EstimatedTimeToTarget = (this->speedTarget-vehicle.curSpeed_mps)/(this->ProjectedAcceleration);
	this->BehaviorTertiary::Maneuver_Temporal::CalculateCost();
}



BehaviorTertiary::Maneuver_Temporal::Maneuver_Temporal(double my_accelTarget,double my_speedTarget,double my_speedTargetDistance,double my_ProjectedAcceleration,\
		decltype(exlcm::pathstep_t::step_rule) my_TemporalRule)
{
	this->speedTarget = my_speedTarget;
	this->AccelTarget = my_accelTarget;
	this->speedTargetDistance = my_speedTargetDistance;
	this->ProjectedAcceleration = my_ProjectedAcceleration;
	this->TemporalRule = my_TemporalRule;
	this->TemporalTargetsSet = true;
	this->speedTargetDistanceActual = this->speedTargetDistance;
	this->EstimatedTimeToTarget = (this->speedTarget-vehicle.curSpeed_mps)/(this->ProjectedAcceleration);
	this->BehaviorTertiary::Maneuver_Temporal::CalculateCost();
}



behavior_type BehaviorTertiary::CalculateBehavior(\
		decltype(exlcm::pathstep_t::step_task) SpatialTask,decltype(exlcm::pathstep_t::step_rule) TemporalRule ){
	behavior_type My_Behavior_Type;
	switch(SpatialTask)
	{
	case STANDBY_TASK: {My_Behavior_Type = STOP_AT_STOP_BAR_BEHAVIOR;break;}
	case CHANGE_LANES_TASK: { switch(TemporalRule){
	case TempRules::CHANGE_LANES_OPPORTUNISTIC: {My_Behavior_Type = EVASIVE_LANE_CHANGE_BEHAVIOR;break;}
	default:My_Behavior_Type = LANE_CHANGE_BEHAVIOR;break;}
	break;
	}
	default: { switch(TemporalRule){
	case TempRules::LEFT_TURN: {My_Behavior_Type = TURN_LEFT_BEHAVIOR;break;}
	case TempRules::RIGHT_TURN: {My_Behavior_Type = TURN_RIGHT_BEHAVIOR;break;}
	case TempRules::TURN_ALONG_CURVY_ROAD_: {My_Behavior_Type = TURN_ALONG_CURVY_ROAD_BEHAVIOR;break;}
	case TempRules::ACC: {My_Behavior_Type = ACC_BEHAVIOR;break;}
	case TempRules::YIELD: {My_Behavior_Type = YIELD_TO_PEDESTRIAN_BEHAVIOR;break;}
	case TempRules::YIELD_TO_CROSS_TRAFFIC: {My_Behavior_Type = YIELD_TO_CROSS_TRAFFIC_BEHAVIOR;break;}
	case TempRules::INTERSECTION: {My_Behavior_Type = HANDLE_INTERSECTION_BEHAVIOR;break;}
	case TempRules::STOP: {My_Behavior_Type = STOP_AT_STOP_BAR_BEHAVIOR;break;}
	case TempRules::RIGHT_OF_WAY: {My_Behavior_Type = LANE_KEEPING_BEHAVIOR;break;}
	case TempRules::STOP_AT_TRAFFIC_LIGHT: {My_Behavior_Type = STOP_AT_TRAFFIC_LIGHT_BEHAVIOR;break;}
	case TempRules::YIELD_AT_TRAFFIC_LIGHT: {My_Behavior_Type = YIELD_TO_CROSS_TRAFFIC_BEHAVIOR;break;}
	case TempRules::EXIT: {My_Behavior_Type = EXITING_FLOW_BEHAVIOR;break;}
	case TempRules::NO_RULES: {My_Behavior_Type = UNKNOWN_BEHAVIOR;break;}
	case TempRules::SLOWDOWN_FOR_SAFETY: {My_Behavior_Type = SAFETY_FIRST_BEHAVIOR;break;}
	case TempRules::COLLISION_IMMINENT: {My_Behavior_Type = COLLISION_IMMINENT_BEHAVIOR;break;}
	default: {My_Behavior_Type = UNKNOWN_BEHAVIOR;break;}
	}
	}
	}
	return My_Behavior_Type;
}

void BehaviorTertiary::Behavior::CalculateBehavior(){
	this->behaviorType = BehaviorTertiary::CalculateBehavior(this->SpatialTask,this->TemporalRule);
	this->ConsideredBehaviorAgents.insert(this->behaviorType);
}

BehaviorTertiary::Behavior::Behavior()
{
	this->behaviorType = UNKNOWN_BEHAVIOR;
	this->ConsideredBehaviorAgents.clear();
	this->ConsideredBehaviorAgents.insert(this->behaviorType);
	this->laneChangeZoneStart_m = 0;
	this->laneChangeZoneEnd_m = 0;
	this->TgtlaneChangeZoneStart_m = 0.0 ;
	this->TgtlaneChangeZoneEnd_m = 0.0;

	this->BehaviorTertiary::Behavior::CalculateCost();
}



BehaviorTertiary::Behavior::Behavior(Env::PolicyState &TargetState,std::pair<int,int> lane):Maneuver_Spatial(TargetState), Maneuver_Temporal(TargetState,lane)
{
	this->TargetState=TargetState;
	this->behaviorType = BehaviorTertiary::CalculateBehavior(this->SpatialTask,this->TemporalRule);
	this->BehaviorTertiary::Behavior::CalculateCost();
	this->ConsideredBehaviorAgents.clear();
	this->ConsideredBehaviorAgents.insert(this->behaviorType);

}

BehaviorTertiary::Behavior::Behavior(BehaviorTertiary::Maneuver_Spatial &My_Maneuver_Spatial,\
		BehaviorTertiary::Maneuver_Temporal &My_Maneuver_Temporal):Maneuver_Spatial(My_Maneuver_Spatial), Maneuver_Temporal(My_Maneuver_Temporal)
{
	this->behaviorType = BehaviorTertiary::CalculateBehavior(this->SpatialTask,this->TemporalRule);
	this->BehaviorTertiary::Behavior::CalculateCost();
	this->ConsideredBehaviorAgents.clear();
	this->ConsideredBehaviorAgents.insert(this->behaviorType);
}


BehaviorTertiary::Behavior::Behavior(Behavior& Root,BehaviorTertiary::Maneuver_Spatial& My_Maneuver_Spatial, BehaviorTertiary::Maneuver_Temporal& My_Maneuver_Temporal):\
		Maneuver_Spatial(My_Maneuver_Spatial), Maneuver_Temporal(My_Maneuver_Temporal)
		{
	this->behaviorType = BehaviorTertiary::CalculateBehavior(this->SpatialTask,this->TemporalRule);
	this->BehaviorTertiary::Behavior::CalculateCost();
	this->ConsideredBehaviorAgents = Root.ConsideredBehaviorAgents;
	this->ConsideredBehaviorAgents.insert(this->behaviorType);
	this->TargetState = Root.TargetState;
		}

bool BehaviorTertiary::operator<(const BehaviorTertiary::Maneuver_Temporal& Maneuver_TempA, \
		const BehaviorTertiary::Maneuver_Temporal& Maneuver_TempB){
	if(Maneuver_TempA.Temporal_cost<Maneuver_TempB.Temporal_cost)
		return true;

	return false;

}


BehaviorTertiary::Maneuver_Temporal BehaviorTertiary::operator+(const BehaviorTertiary::Maneuver_Temporal& Maneuver_TempA, \
		const BehaviorTertiary::Maneuver_Temporal& Maneuver_TempB)
{
	Maneuver_Temporal My_Maneuver_Temporal = Maneuver_TempA;



	if ((Maneuver_TempA.Temporal_cost > Maneuver_TempB.Temporal_cost) || !Maneuver_TempA.TemporalTargetsSet)
		My_Maneuver_Temporal =   Maneuver_TempB;
	else
		My_Maneuver_Temporal =   Maneuver_TempA;

	if((Maneuver_TempA.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&(Maneuver_TempB.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&Maneuver_TempA.TemporalTargetsSet&&Maneuver_TempB.TemporalTargetsSet){
		if((Maneuver_TempA.speedTargetDistanceActual>Maneuver_TempB.speedTargetDistanceActual)|| !Maneuver_TempA.TemporalTargetsSet)
			My_Maneuver_Temporal =   Maneuver_TempB;
		else if(Maneuver_TempA.speedTargetDistanceActual<Maneuver_TempB.speedTargetDistanceActual)
			My_Maneuver_Temporal =   Maneuver_TempA;
	}


	if ((Maneuver_TempA.horn_request == 1)||(Maneuver_TempB.horn_request == 1))
		My_Maneuver_Temporal.horn_request = 1;

	if(Maneuver_TempA.primary_target_object==-1)
		My_Maneuver_Temporal.primary_target_object=Maneuver_TempB.primary_target_object;
	else if(Maneuver_TempB.primary_target_object==-1)
		My_Maneuver_Temporal.primary_target_object=Maneuver_TempA.primary_target_object;

	if(Maneuver_TempA.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
		My_Maneuver_Temporal.BehaviorAccelRequestType = Maneuver_TempB.BehaviorAccelRequestType;
	else if(Maneuver_TempB.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
		My_Maneuver_Temporal.BehaviorAccelRequestType = Maneuver_TempA.BehaviorAccelRequestType;

	My_Maneuver_Temporal.Maneuver_Temporal::PolicyCost = Maneuver_TempA.Maneuver_Temporal::PolicyCost + Maneuver_TempB.Maneuver_Temporal::PolicyCost;


	return My_Maneuver_Temporal;
}





BehaviorTertiary::Behavior BehaviorTertiary::operator+( BehaviorTertiary::Behavior& BehaviorA,  BehaviorTertiary::Behavior& BehaviorB)
{
	BehaviorTertiary::Behavior My_Behavior = BehaviorA;

	My_Behavior.Behavior::PolicyCost = My_Behavior.Behavior::PolicyCost + BehaviorB.Behavior::PolicyCost;
	My_Behavior.SetBehaviorPolicyCost();


	if(BehaviorTertiary::SpatialTaskGroup[BehaviorA.SpatialTask]!=BehaviorTertiary::SpatialTaskGroup[BehaviorB.SpatialTask])
		return My_Behavior;

	else if(BehaviorTertiary::SpatialTaskGroup[BehaviorA.SpatialTask]==LANE_SHIFT){
		if(BehaviorA.TargetState.DAR_m<=BehaviorB.TargetState.DAR_m+0.01) // added 0.01 to account for some precision error.
			return BehaviorA;
		else if(BehaviorTertiary::SpatialTaskGroup[BehaviorB.SpatialTask]==LANE_SHIFT)
			return BehaviorA;
		else{
			return BehaviorB;
		}
	}


	if((BehaviorA.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&(BehaviorB.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&BehaviorA.TemporalTargetsSet&&BehaviorB.TemporalTargetsSet){
		if((BehaviorA.speedTargetDistanceActual >BehaviorB.speedTargetDistanceActual)|| !BehaviorA.TemporalTargetsSet)
			My_Behavior =   BehaviorB;
		else if(BehaviorA.speedTargetDistanceActual<BehaviorB.speedTargetDistanceActual)
			My_Behavior =   BehaviorA;
	}
	else if (BehaviorA.Behavior_cost == BehaviorB.Behavior_cost){
		if(BehaviorTertiary::BehaviorPreference[BehaviorA.behaviorType]>BehaviorTertiary::BehaviorPreference[BehaviorB.behaviorType])
			My_Behavior = BehaviorA;
		else
			My_Behavior = BehaviorB;
	}
	else if (BehaviorA.Behavior_cost > BehaviorB.Behavior_cost){
		My_Behavior =   BehaviorB;
	}
	else{
		My_Behavior =   BehaviorA;
	}

	if ((BehaviorB.turn_indicator_command == TURN_INDICATOR_NONE) || (BehaviorB.turn_indicator_command == TURN_INDICATOR_UNKNOWN)){
		My_Behavior.turn_indicator_command = BehaviorA.turn_indicator_command;
		//	My_Behavior.turn_indicator_target_distance = BehaviorA.turn_indicator_target_distance;
	}

	if ((BehaviorA.turn_indicator_command == TURN_INDICATOR_NONE) || (BehaviorA.turn_indicator_command == TURN_INDICATOR_UNKNOWN)){
		My_Behavior.turn_indicator_command = BehaviorB.turn_indicator_command;
		//	My_Behavior.turn_indicator_target_distance = BehaviorB.turn_indicator_target_distance;
	}

	if ((!((BehaviorA.turn_indicator_command == TURN_INDICATOR_NONE) || (BehaviorA.turn_indicator_command == TURN_INDICATOR_UNKNOWN))) && \
			(!((BehaviorB.turn_indicator_command == TURN_INDICATOR_NONE) || (BehaviorB.turn_indicator_command == TURN_INDICATOR_UNKNOWN))) ){
		if (BehaviorB.TargetState.DAR_m > BehaviorA.TargetState.DAR_m){
			My_Behavior.turn_indicator_command = BehaviorA.turn_indicator_command;
			//	My_Behavior.turn_indicator_target_distance = BehaviorA.turn_indicator_target_distance;
		}
		else{
			My_Behavior.turn_indicator_command = BehaviorB.turn_indicator_command;
			//	My_Behavior.turn_indicator_target_distance = BehaviorB.turn_indicator_target_distance;
		}
	}

	if ((BehaviorA.horn_request == 1)||(BehaviorB.horn_request == 1))
		My_Behavior.horn_request = 1;

	if(BehaviorA.primary_target_object==-1)
		My_Behavior.primary_target_object=BehaviorB.primary_target_object;
	else if(BehaviorB.primary_target_object==-1)
		My_Behavior.primary_target_object=BehaviorA.primary_target_object;

	if(BehaviorA.reRouteNeeded==NO_REROUTE)
		My_Behavior.reRouteNeeded = BehaviorB.reRouteNeeded;
	else if(BehaviorB.reRouteNeeded==NO_REROUTE)
		My_Behavior.reRouteNeeded = BehaviorA.reRouteNeeded;

	if(BehaviorA.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
		My_Behavior.BehaviorAccelRequestType = BehaviorB.BehaviorAccelRequestType;
	else if(BehaviorB.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
		My_Behavior.BehaviorAccelRequestType = BehaviorA.BehaviorAccelRequestType;


	My_Behavior.alternaterouteNeeded = BehaviorA.alternaterouteNeeded;
	My_Behavior.alternaterouteNeeded.insert(BehaviorB.alternaterouteNeeded.begin(),BehaviorB.alternaterouteNeeded.end());


	My_Behavior.ConsideredBehaviorAgents = BehaviorA.ConsideredBehaviorAgents;
	My_Behavior.ConsideredBehaviorAgents.insert(BehaviorB.ConsideredBehaviorAgents.begin(),BehaviorB.ConsideredBehaviorAgents.end());
	My_Behavior.ConsideredBehaviorAgents.insert(BehaviorA.behaviorType);
	My_Behavior.ConsideredBehaviorAgents.insert(BehaviorB.behaviorType);

	//My_Behavior.Behavior::PolicyCost = My_Behavior.Behavior::PolicyCost + BehaviorB.Behavior::PolicyCost;
	My_Behavior.SetBehaviorPolicyCost(BehaviorA,BehaviorB);



	return My_Behavior;
}



BehaviorTertiary::Behavior BehaviorTertiary::operator+(const BehaviorTertiary::Behavior& BehaviorA, const BehaviorTertiary::Maneuver_Temporal& Maneuver_Temp)
{
	BehaviorTertiary::Behavior My_Behavior = BehaviorA;
	double accelfact = BehaviorUtils::LookUp1DMap(BehaviorTertiary::AccelFactorDefault,Maneuver_Temp.speedTargetDistance);

	bool WinnerBehavior = false;
	if((BehaviorA.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&(Maneuver_Temp.speedTarget<=SPD_THRSH_FOR_DISTBSD_COST)&&Maneuver_Temp.TemporalTargetsSet&&BehaviorA.TemporalTargetsSet){
		if(BehaviorA.speedTargetDistanceActual<Maneuver_Temp.speedTargetDistanceActual)
			WinnerBehavior =   true;
	}

	if ( !Maneuver_Temp.TemporalTargetsSet){
	}
	else if((Maneuver_Temp.Temporal_cost >= My_Behavior.Temporal_cost)||WinnerBehavior){
		My_Behavior.Behavior::PolicyCost = My_Behavior.Behavior::PolicyCost + Maneuver_Temp.Maneuver_Temporal_cost_wt*Maneuver_Temp.Maneuver_Temporal::PolicyCost;
		if(My_Behavior.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
			My_Behavior.BehaviorAccelRequestType=Maneuver_Temp.BehaviorAccelRequestType;
		if(My_Behavior.primary_target_object==-1)
			My_Behavior.primary_target_object=Maneuver_Temp.primary_target_object;
	}
	else{

		My_Behavior.Behavior::PolicyCost = My_Behavior.Behavior::PolicyCost + Maneuver_Temp.Maneuver_Temporal_cost_wt*Maneuver_Temp.Maneuver_Temporal::PolicyCost;

		BehaviorTertiary::Maneuver_Spatial My_Maneuver_Spatial;
		BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
		My_Maneuver_Spatial = My_Behavior;
		My_Maneuver_Temporal = Maneuver_Temp;
		My_Behavior = BehaviorTertiary::Behavior(My_Behavior,My_Maneuver_Spatial,My_Maneuver_Temporal);


		if ((My_Behavior.turn_indicator_command == TURN_INDICATOR_NONE) || (My_Behavior.turn_indicator_command == TURN_INDICATOR_UNKNOWN)){
			My_Behavior.turn_indicator_command = BehaviorA.turn_indicator_command;
			//	My_Behavior.turn_indicator_target_distance = BehaviorA.turn_indicator_target_distance;
		}
		else if (!((BehaviorA.turn_indicator_command == TURN_INDICATOR_NONE) || (BehaviorA.turn_indicator_command == TURN_INDICATOR_UNKNOWN))) {
			if (My_Behavior.TargetState.DAR_m > BehaviorA.TargetState.DAR_m){
				My_Behavior.turn_indicator_command = BehaviorA.turn_indicator_command;
				//		My_Behavior.turn_indicator_target_distance = BehaviorA.turn_indicator_target_distance;
			}
		}

		if(My_Behavior.primary_target_object==-1)
			My_Behavior.primary_target_object=BehaviorA.primary_target_object;

		if(My_Behavior.BehaviorAccelRequestType==COMFORT_ACCEL_REQUEST_BEHAVIOR)
			My_Behavior.BehaviorAccelRequestType=BehaviorA.BehaviorAccelRequestType;

		My_Behavior.ConsideredBehaviorAgents.insert(BehaviorA.behaviorType);

	}


	return My_Behavior;


}


void BehaviorTertiary::Behavior::CalculateCost(){
	this->BehaviorTertiary::Maneuver_Temporal::CalculateCost();
	this->BehaviorTertiary::Maneuver_Spatial::CalculateCost();
	this->Behavior_cost = this->Spatial_cost*this->Maneuver_Spatial_cost_wt+this->Temporal_cost*this->Maneuver_Temporal_cost_wt;
	double accelfact = BehaviorUtils::LookUp1DMap(BehaviorTertiary::AccelFactorDefault,this->speedTargetDistance);
	this->Behavior::PolicyCost = fabs(this->Maneuver_Spatial_cost_wt*this->Spatial_cost*accelfact) + fabs(this->Maneuver_Temporal_cost_wt*this->Temporal_cost*accelfact);
	this->SetBehaviorPolicyCost();
}

void BehaviorTertiary::Maneuver_Temporal::CalculateCost(){
	double accelfact = BehaviorUtils::LookUp1DMap(BehaviorTertiary::AccelFactorDefault,this->speedTargetDistance);
	this->Temporal_cost = (this->ProjectedAcceleration*accelfact)/BehaviorTertiary::MaxAccelLimit + this->Temporal_cost_bias;
	this->Temporal_cost = fmin(fmax(this->Temporal_cost ,-1),1);
	if(this->speedTargetDistance> 120)
		this->Temporal_cost = 1.0;
	this->Maneuver_Temporal::PolicyCost =  fabs(this->Maneuver_Temporal_cost_wt*this->Temporal_cost*accelfact);
	if(this->speedTarget>SAFE_FOLLOWING_MAX_SPEED)
		this->Maneuver_Temporal::PolicyCost = 1.0;

}

void BehaviorTertiary::Maneuver_Spatial::CalculateCost(){
	if(BehaviorTertiary::SpatialTaskGroup[this->SpatialTask]==LANE_SHIFT)
		this->Spatial_cost += 0.25; // Punish lane shift
	this->Maneuver_Spatial::PolicyCost =  fabs(this->Maneuver_Spatial_cost_wt*this->Spatial_cost);
}






BehaviorTertiary::Maneuver_Spatial BehaviorTertiary::operator*(const BehaviorTertiary::Maneuver_Spatial& myClass, double Factor){
	BehaviorTertiary::Maneuver_Spatial Temp_myClass = myClass;
	Temp_myClass.Spatial_cost= myClass.Spatial_cost*Factor;
	return Temp_myClass;
}

BehaviorTertiary::Maneuver_Temporal BehaviorTertiary::operator*(const BehaviorTertiary::Maneuver_Temporal& myClass, double Factor){
	BehaviorTertiary::Maneuver_Temporal Temp_myClass = myClass ;
	Temp_myClass.Temporal_cost *=Factor;
	Temp_myClass.ProjectedAcceleration*=Factor;
	Temp_myClass.speedTargetDistance = BU::targetDistance(Temp_myClass.speedTarget,Temp_myClass.ProjectedAcceleration);
	if(Temp_myClass.speedTargetDistance<0)
		Temp_myClass.speedTargetDistance = 0;
	return Temp_myClass;
}

BehaviorTertiary::Behavior BehaviorTertiary::operator*(const BehaviorTertiary::Behavior& myClass, double Factor){
	BehaviorTertiary::Behavior Temp_myClass = myClass;
	Temp_myClass.Temporal_cost= myClass.Temporal_cost*Factor;
	return Temp_myClass;
}

// For utility convex function - risk loving, concave - risk averse ,  linear - risk neutral. opposite for cost functions.




