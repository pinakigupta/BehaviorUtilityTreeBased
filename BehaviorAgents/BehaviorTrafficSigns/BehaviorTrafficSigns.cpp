#include "../../PCH/pch.hpp"
#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/shared_enums.h"
#include "../../include/AgentPool.hpp"


struct  vehicle;
namespace BU=BehaviorUtils;

std::map< std::pair<LaneSegUidType,int>, std::tuple<int,double, double> > LastValidTrafficSignStates; //    key-> <laneseg uid, conn idx>  :  value- ><Sign,Station,Time>

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::HandleObservedTrafficSigns(Env::PolicyState &TargetState){
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	My_Maneuver_Temporal.TemporalRule = TempRules::RIGHT_OF_WAY;// Do this as we are not really sure if there is a traffic sign or not.
	int StepIdx = TargetState.step.step_index_n;
	std::string CurrStepStr = "<StepIdx:"+std::to_string(StepIdx)+">";
	if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
		return My_Maneuver_Temporal;

	double CurrentspeedTarget=TargetState.step.posted_speed_lim_mps;
	double env_s = TargetState.DAR_m-vehicle.DTAR;
//	const double TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGN_STATE = 10;
	const double TimeGap=1;
	const double LARGE_DISTANCE_FROM_TRAFFIC_SIGN = 60;
	My_Maneuver_Temporal= BehaviorElementary::laneKeepBehavior(TargetState);

	int connID;
	BehaviorUtils::getGrpConn(TargetState.CurrentGrpLanes,TargetState.TargetGrpLanes,&connID);
	if(connID==-1){
		BehaviorUtils::codeRed( _CF_  + "In function HandleTrafficSigns() Couldn't find connection between {current , target } group " + CurrStepStr,\
				std::make_pair(TargetState.step.current_segment_group,TargetState.step.target_segment_group),TimeGap);
		AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
		return My_Maneuver_Temporal;
	}

	int CurrentTrafficSignState;
	double CurrentTrafficSignStation;
	auto timenow = BehaviorUtils::TimeNow();

	auto MyPair = std::make_pair(TargetState.CurrentGrpLanes.back(),connID);

	if(behaviorinput::TrafficSignStatesList.find(MyPair)!=behaviorinput::TrafficSignStatesList.end()){
		exlcm::connectiontrafficsign_t CurrentTrafficSign;
		double CurrentTrafficSignRcvTm;
		std::tie(CurrentTrafficSign, CurrentTrafficSignRcvTm)= behaviorinput::TrafficSignStatesList[MyPair];
		CurrentTrafficSignState = CurrentTrafficSign.sign[0];
		CurrentTrafficSignStation = CurrentTrafficSign.station_m[0];

		if(timenow-CurrentTrafficSignRcvTm > TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGN_STATE ){ // Dead reckon if not getting the traffic signal upfate
			//if(env_s <LARGE_DISTANCE_FROM_TRAFFIC_SIGN)
			//	My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
			AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
			return My_Maneuver_Temporal;
		}


		// Holding the previous state if no useful updates are received (only if that's the front state)
		bool CurrentTrafficSignStateAmbiguous = (CurrentTrafficSignState == 0);

		if (CurrentTrafficSignStateAmbiguous ){
			if(env_s <LARGE_DISTANCE_FROM_TRAFFIC_SIGN){
				//My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
				if(LastValidTrafficSignStates.find(MyPair)!=LastValidTrafficSignStates.end()){
					if ((timenow-std::get<2>(LastValidTrafficSignStates[MyPair]))<TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGN_STATE){
						CurrentTrafficSignState = std::get<0>(LastValidTrafficSignStates[std::make_pair(TargetState.CurrentGrpLanes.back(),connID)]);
						CurrentTrafficSignStation = std::get<1>(LastValidTrafficSignStates[std::make_pair(TargetState.CurrentGrpLanes.back(),connID)]);
					}
				}
			}
		}
		else{
			LastValidTrafficSignStates[MyPair]= std::make_tuple(CurrentTrafficSignState, CurrentTrafficSignStation, BehaviorUtils::TimeNow());
			//	TimeInKnownState = BehaviorUtils::TimeNow();
		}

		Env::PolicyState NewTargetState = TargetState;
		double TargetStation = TargetState.step.current_segment_station_m;
		if (TargetStation<0)
			TargetStation = fabs(TargetStation) + BU::getSegLen(MyPair.first) ;
		NewTargetState.DAR_m = CurrentTrafficSignStation- TargetStation + TargetState.DAR_m;


		BehaviorTertiary::Maneuver_Temporal Maneuver_wo_SIGN = My_Maneuver_Temporal;
		// Now execute the TRAFFIC SIGN BEHAVIOR. *** There can be multiple traffic signs at the same location. s
		if(CurrentTrafficSignState==STOP_SIGN_OBJECT) {// STOP (assume followed by yield)
			BehaviorTertiary::Maneuver_Temporal StopAtStopSign  = BehaviorSecondary::stopAtStopBar(NewTargetState) ;
			My_Maneuver_Temporal= My_Maneuver_Temporal + StopAtStopSign ;
		//	My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorElementary::turn(NewTargetState,DEC_VEL_TO_YIELD_MPS);
			TargetState.observed_step_rule.insert(STOP_AND_YIELD_UNTIL_CLEAR_RULE);
			BU::codeRed( _CF_  + "NewTargetState "+ CurrStepStr,NewTargetState,TimeGap);
			BU::codeRed( _CF_  + "Stop at STOP_SIGN Behavior "+ CurrStepStr,StopAtStopSign,TimeGap);
			BU::codeRed( _CF_  + "My_Maneuver_Temporal  "+ CurrStepStr,My_Maneuver_Temporal,TimeGap);

		}

		if(CurrentTrafficSignState==YIELD_SIGN_OBJECT) {// YIELD
			My_Maneuver_Temporal= My_Maneuver_Temporal + BehaviorTertiary::ControlComplexBehavior(NewTargetState,true) ;
	//		My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorElementary::turn(NewTargetState,DEC_VEL_TO_YIELD_MPS);
			TargetState.observed_step_rule.insert(YIELD_UNTIL_CLEAR_RULE);
			BU::codeRed( _CF_  + "Yield at YIELD_SIGN_OBJECT "+ CurrStepStr," ",TimeGap);
		}

		if((CurrentTrafficSignState==PEDESTRIAN_CROSSING_SIGN_OBJECT)||true) {// If PEDESTRIANs are detected at intersections always yield.
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::CrossWalkBehavior(NewTargetState);
	//		if(CurrentTrafficSignState==PEDESTRIAN_CROSSING_SIGN_OBJECT)
	//			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorElementary::turn(NewTargetState,DEC_VEL_TO_YIELD_MPS);
			if(CurrentTrafficSignState==PEDESTRIAN_CROSSING_SIGN_OBJECT)
				TargetState.observed_step_rule.insert(YIELD_FOR_PEDESTRIAN_RULE);
			BU::codeRed( _CF_  + "Yield at PEDESTRIAN_CROSSING_SIGN_OBJECT "+ CurrStepStr," ",TimeGap);
		}


	}
	else{
		BU::codeRed( _CF_  + "Traffic sign details NOT found in behaviorinput::TrafficSignStatesList for (segment, connection) "+ CurrStepStr,MyPair,TimeGap);
	}


	AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
	return My_Maneuver_Temporal;
}
