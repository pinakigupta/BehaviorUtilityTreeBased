#include "../../include/BehaviorGapUtils.hpp"
#include "../../include/BehaviorUtils.hpp"
#include "../../include/AgentPool.hpp"
#include "../../include/BehaviorOptimalPolicy.hpp"

GapType IDEAL_LC_ZONE;
GapType BLIND_ZONE;


BehaviorTertiary::Behavior GetSimpleLaneBehavior(Env::PolicyState &TargetState, std::pair<int,int> Mylane){
	std::pair<int,int>  lane = Mylane;
	lane.first = GetAlternateLane(lane.first,TargetState);
	Env::PolicyState *MyTargetStateptr;
	MyTargetStateptr = &TargetState;
	MyTargetStateptr = MyTargetStateptr->Next_State;

	BehaviorTertiary::Behavior Next_Lane_Behavior_First_Step = BehaviorTertiary::Behavior(*MyTargetStateptr,lane);
	BehaviorTertiary::Behavior Next_Lane_Behavior = Next_Lane_Behavior_First_Step;


	double EndOfStepDistanceFromHV =0;
	if((MyTargetStateptr->DAR_m - vehicle.DTAR)<BEHAVIOR_LOOK_AHEAD_HORIZON){
		do{
			if(BehaviorTertiary::SpatialTaskGroup[MyTargetStateptr->step.step_task]!=BehaviorTertiary::SpatialTaskType::LANE_PROPAGATE) //Simple lane propagation in the next lane. Not another lane change etc.
				break;
			BehaviorTertiary::Behavior NextBehaviorDesired(*MyTargetStateptr,lane);
			Next_Lane_Behavior = Next_Lane_Behavior + NextBehaviorDesired;
			MyTargetStateptr = MyTargetStateptr->Next_State;
			if(MyTargetStateptr==nullptr){
				break;
			}
			EndOfStepDistanceFromHV = (MyTargetStateptr->DAR_m - vehicle.DTAR);
		}while(EndOfStepDistanceFromHV< BEHAVIOR_LOOK_AHEAD_HORIZON);
	}
	return Next_Lane_Behavior;
}

int GetAlternateLane(int LANE_SITUATION,Env::PolicyState &TargetState){
	if(LANE_SITUATION>=0)
		return LANE_SITUATION;
	Env::PolicyState *MyTargetStateptr;
	MyTargetStateptr = &TargetState;

	double Start_DAR_m = TargetState.DAR_m;

	double EndOfStepDistanceFromHV =0;
	if((MyTargetStateptr->DAR_m - vehicle.DTAR)<BEHAVIOR_LOOK_AHEAD_HORIZON){
		do{
			if(MyTargetStateptr->step.direction==BEAR_LEFT)
				return LEFT_ADJACENT_LANE_SITUATION;
			else if(MyTargetStateptr->step.direction==BEAR_RIGHT)
				return RIGHT_ADJACENT_LANE_SITUATION;
			else if(MyTargetStateptr->step.direction==BEAR_LEFT_OPPOSING_LANE)
				return LEFT_OPPOSING_LANE_SITUATION;
			else if(MyTargetStateptr->step.direction==BEAR_RIGHT_OPPOSING_LANE)
				return RIGHT_OPPOSING_LANE_SITUATION;

			MyTargetStateptr = MyTargetStateptr->Next_State;
			if(MyTargetStateptr==nullptr){
				break;
			}
			EndOfStepDistanceFromHV = (MyTargetStateptr->DAR_m - vehicle.DTAR);
		}while(EndOfStepDistanceFromHV< BEHAVIOR_LOOK_AHEAD_HORIZON);

	}

	return HV_LANE_SITUATION;
}





void BehaviorGapUtils::PlotGaps(BehaviorGapUtils::GapObjectVec& Behind,BehaviorGapUtils::GapObject& Adjacent,BehaviorGapUtils::GapObjectVec& Ahead,BehaviorGapUtils::GapObject& Center_Ahead){
	GapPredFuncTypeVec MyGapfuncs ;
	GapPredFuncTypeVec MyGapOverLapfuncs;
	std::vector<std::string> Mylabels ;
	for(int i=0;i<Behind.size();i++){
		MyGapOverLapfuncs.push_back(Behind[i].GapOverlapPredFunc);
		MyGapfuncs.push_back(Behind[i].GapPredFunc);
		Mylabels.push_back(" BehindGapPred ");
	}
	MyGapOverLapfuncs.push_back(Adjacent.GapOverlapPredFunc);
	MyGapfuncs.push_back(Adjacent.GapPredFunc);
	Mylabels.push_back(" AdjacentGapPred ");
	for(int i=0;i<Ahead.size();i++){
		MyGapOverLapfuncs.push_back(Ahead[i].GapOverlapPredFunc);
		MyGapfuncs.push_back(Ahead[i].GapPredFunc);
		Mylabels.push_back(" AheadGapPred ");
	}


	if(!behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause){
		BehaviorGapUtils::PlotPredictedGap(MyGapfuncs,MyGapOverLapfuncs,Center_Ahead.GapPredFunc,Mylabels);
	}
}


void FilterSmallLCZ(BehaviorTertiary::Behavior& MyBehavior,double laneChangeZoneEnd_Thresh ){
	if(MyBehavior.laneChangeZoneEnd_m<laneChangeZoneEnd_Thresh){ // Safety check
		MyBehavior.laneChangeZoneStart_m = 0;
		MyBehavior.laneChangeZoneEnd_m = 0;
		MyBehavior.leftLaneChangeIntention=LANE_CHANGE_IS_NOT_DESIRED ;
		MyBehavior.rightLaneChangeIntention=LANE_CHANGE_IS_NOT_DESIRED ;
	}

}


namespace Policy{
extern void SearchAlternatePolicy(Policy::DetailedPolicy&, const Env::PolicyState& , int );
extern BehaviorTertiary::Behavior& CreateBehaviorFromPolicyDeque(BehaviorTertiary::Behavior& , PolicyDequeType &, \
		int , int ,  int , bool ,bool);
}


BehaviorTertiary::Behavior BehaviorElementary::PlannedlaneChange(Env::PolicyState &TargetState)
{
	BehaviorTertiary::Behavior LaneKeep_Behavior = BehaviorElementary::laneKeepBehavior(TargetState);
	BehaviorTertiary::Behavior Current_Lane_Behavior = LaneKeep_Behavior;

	if(AgentPool::SeekAgentFromPool(Current_Lane_Behavior,__FUNCTION__))
		return Current_Lane_Behavior;


	BehaviorTertiary::Behavior BehaviorDesiredLeftLane, BehaviorDesiredRightLane;
	BehaviorDesiredLeftLane = BehaviorElementary::StaticlaneKeeping(TargetState);
	BehaviorDesiredRightLane = BehaviorElementary::StaticlaneKeeping(TargetState);
	IDEAL_LC_ZONE = std::make_pair(0,LCX_MIN_GAP_AHEAD);
	BLIND_ZONE = std::make_pair(-LCX_MIN_GAP_BEHIND,VEHICLE_LENGTH/2);

	if(TargetState.SuspectedOpposingGrp)
		BLIND_ZONE = std::make_pair(-LCX_MIN_GAP_BEHIND_STATIC_OBJ,VEHICLE_LENGTH/2);

	double env_s1 = TargetState.DAR_m-vehicle.DTAR;
	double env_s0 = env_s1 - (TargetState.step.target_segment_station_m-TargetState.step.current_segment_station_m);
	vehicle.PredAccel_mpss = vehicle.curAccel_mpss ;

	int UrGencyOverRide=-1;
	if((behaviorinput::routeSegmentList.reroute_status==LANE_CHANGE_ROUTE)||(behaviorinput::routeSegmentList.reroute_status==LANE_CHANGE_ROUTE_DLCX_RQRD)||(behaviorinput::routeSegmentList.reroute_status==LANE_CHANGE_ROUTE_DLCX_ALLWD))
		UrGencyOverRide = LANE_CHANGE_IS_URGENT;


	Current_Lane_Behavior.SpatialTask = CHANGE_LANES_TASK;
	Current_Lane_Behavior.CalculateBehavior();

	PolicyDequeType StraightPolicyDQ ;
	Policy::InputPolicyType StraightPlan;
	Current_Lane_Behavior.alternaterouteNeeded.insert(STRAIGHT_ROUTE);

	std::string StepStr = "<"+std::to_string(TargetState.step.step_index_n)+">";


	//BU::codeYellow( _CF_ +"Current_Lane_Behavior",Current_Lane_Behavior,1);
	BehaviorTertiary::Maneuver_Temporal StraightTemporalManeuver;
	if(BehaviorUtils::find(Policy::AllCandidatePolicies,(int)STRIAGHT_LANE_SITUATION)){
		StraightTemporalManeuver = 	Policy::AllCandidatePolicies[STRIAGHT_LANE_SITUATION].PolicyOptimalBehavior;
		Current_Lane_Behavior = Current_Lane_Behavior + StraightTemporalManeuver;
	}else{
		Current_Lane_Behavior.alternaterouteNeeded.insert(STRIAGHT_LANE_SITUATION);
	}

	bool TrajCommitted = false;
	lane_change_feedback_enum traj_fb = behaviorinput::_trajectoryplan.lane_change_feedback;

	if(TargetState.step.direction==BEAR_LEFT)
	{
		Current_Lane_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_LEFT;
		Current_Lane_Behavior.direction = BEAR_LEFT;
		TrajCommitted = (traj_fb==COMMITED_TO_LEFT);
		BU::codeGreen(_CF_ + "Executing Left lane change Behavior. {env_s0,env_s1} " + StepStr,std::make_pair(env_s0,env_s1),1);
		ExecuteLaneChange(LEFT_ADJACENT_LANE_SITUATION,env_s0,env_s1 , Current_Lane_Behavior, BehaviorDesiredLeftLane, TargetState,UrGencyOverRide, TrajCommitted );
		Current_Lane_Behavior.LeftAdjacentLane.push_back(BehaviorDesiredLeftLane);
	}
	else if(TargetState.step.direction==BEAR_RIGHT)
	{
		Current_Lane_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_RIGHT;
		Current_Lane_Behavior.direction = BEAR_RIGHT;
		TrajCommitted = (traj_fb==COMMITED_TO_RIGHT);
		BU::codeGreen(_CF_ + "Executing Right lane change Behavior. {env_s0,env_s1} "+ StepStr,std::make_pair(env_s0,env_s1),1);
		ExecuteLaneChange(RIGHT_ADJACENT_LANE_SITUATION,env_s0,env_s1 , Current_Lane_Behavior, BehaviorDesiredRightLane, TargetState,UrGencyOverRide, TrajCommitted);
		Current_Lane_Behavior.RightAdjacentLane.push_back(BehaviorDesiredRightLane);
	}
	else if(TargetState.step.direction==BEAR_LEFT_OPPOSING_LANE)
	{
		BLIND_ZONE = std::make_pair(-OPPORTUNISTIC_LCX_BEHIND_CLR_DIST_THRSH,OPPORTUNISTIC_LCX_AHEAD_CLR_DIST_THRSH); // Flip the zone
		Current_Lane_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_LEFT;
		Current_Lane_Behavior.direction = BEAR_LEFT_OPPOSING_LANE;
		TrajCommitted = (traj_fb==COMMITED_TO_LEFT);
		BU::codeGreen(_CF_ + "Executing Opposing Left lane change Behavior. {env_s0,env_s1} "+ StepStr,std::make_pair(env_s0,env_s1),1);
		ExecuteLaneChange(LEFT_OPPOSING_LANE_SITUATION,env_s0,env_s1 , Current_Lane_Behavior, BehaviorDesiredLeftLane, TargetState,UrGencyOverRide, TrajCommitted);
		Current_Lane_Behavior.LeftAdjacentLane.push_back(BehaviorDesiredLeftLane);
		Current_Lane_Behavior.leftLaneChangeIntention=LANE_CHANGE_OPPOSING;

	}
	else if(TargetState.step.direction==BEAR_RIGHT_OPPOSING_LANE)
	{
		BLIND_ZONE = std::make_pair(-OPPORTUNISTIC_LCX_BEHIND_CLR_DIST_THRSH,OPPORTUNISTIC_LCX_AHEAD_CLR_DIST_THRSH); // Flip the zone
		Current_Lane_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_RIGHT;
		Current_Lane_Behavior.direction = BEAR_RIGHT_OPPOSING_LANE;
		TrajCommitted = (traj_fb==COMMITED_TO_RIGHT);
		BU::codeGreen(_CF_ + "Executing Opposing Right lane change Behavior. {env_s0,env_s1} "+ StepStr,std::make_pair(env_s0,env_s1),1);
		ExecuteLaneChange(RIGHT_OPPOSING_LANE_SITUATION,env_s0,env_s1 , Current_Lane_Behavior, BehaviorDesiredRightLane, TargetState,UrGencyOverRide, TrajCommitted);
		Current_Lane_Behavior.RightAdjacentLane.push_back(BehaviorDesiredRightLane);
		Current_Lane_Behavior.rightLaneChangeIntention=LANE_CHANGE_OPPOSING;
	}
	else if(TargetState.step.direction==STRAIGHT_DIRECTION)
	{
		BU::codeGreen( _CF_ +" Lane change completed at segment "+ StepStr, vehicle.curSegmentUid);
	}



	AgentPool::StoreAgentInPool(Current_Lane_Behavior,__FUNCTION__);


	return Current_Lane_Behavior;
}








