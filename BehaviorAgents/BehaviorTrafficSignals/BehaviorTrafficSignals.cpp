#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/shared_enums.h"
#include "../../include/AgentPool.hpp"


namespace BU=BehaviorUtils;
struct  vehicle;

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::StopAtTrafficLight(Env::PolicyState &TargetState, bool StopZone)
{
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	My_Maneuver_Temporal = BehaviorSecondary::stopAndStandBy(TargetState,StopZone);
	My_Maneuver_Temporal.TemporalRule = STOP_AT_TRAFFIC_LIGHT;
	return My_Maneuver_Temporal;
}

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::YieldAtTrafficLight(Env::PolicyState &TargetState)
{
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	My_Maneuver_Temporal = BehaviorTertiary::ControlComplexBehavior(TargetState,false);
	//My_Maneuver_Temporal.TemporalRule = YIELD_AT_TRAFFIC_LIGHT;
	return My_Maneuver_Temporal;
}

std::map< std::pair<LaneSegUidType,int>, std::tuple<eTrafficSignalState,double,double> > LastValidTrafficSignalStates;  // key-> <laneseg uid, conn idx>  :  value- ><Signal,Station,Time>


std::ostream& BehaviorSecondary::PrintEnum(std::ostream &os, const eTrafficLightType My_TrafficLightType ){
	switch(My_TrafficLightType)
	{
	case RED_TYPE: {os<<red_on<<"RED_TYPE"<<color_off<<endl;break;}
	case FLASHING_RED_TYPE: {os<<red_on<<"FLASHING_RED_TYPE"<<color_off<<endl;break;}
	case YELLOW_TYPE: {os<<yellow_on<<"YELLOW_TYPE"<<color_off<<endl;break;}
	case FLASHING_YELLOW_TYPE: {os<<yellow_on<<"FLASHING_YELLOW_TYPE"<<color_off<<endl;break;}
	case GREEN_TYPE: {os<<green_on<<"GREEN_TYPE"<<color_off<<endl;break;}
	case FLASHING_GREEN_TYPE: {os<<green_on<<"FLASHING_GREEN_TYPE"<<color_off<<endl;break;}
	case UNKNOWN_TYPE: {os<<purple_on<<"UNKNOWN_TYPE"<<color_off<<endl;break;}
	case PEDESTRIAN_TYPE: {os<<blue_on<<"PEDESTRIAN_TYPE"<<color_off<<endl;break;}
	default: {os<<purple_on<<"Not sure. step_task Value is "<<(int)My_TrafficLightType<<endl;break;}
	os<<endl ;
	}
	return os;
}

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::HandleTrafficSignals(Env::PolicyState &TargetState,bool StopZone, bool UnmappedTrafficLight){

	auto CurrentspeedTarget=TargetState.step.posted_speed_lim_mps;
	BehaviorTertiary::Behavior LaneKeepBehavior = BehaviorElementary::laneKeepBehavior(TargetState);
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal= LaneKeepBehavior;//object slicing
	eTrafficLightType My_TrafficLightType;
	int StepIdx = TargetState.step.step_index_n;
	std::string CurrStepStr = "<StepIdx:"+std::to_string(StepIdx)+">";

	if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
		return My_Maneuver_Temporal;

	int connID;
	const double TimeGap=1e7;
	double env_s = TargetState.DAR_m-vehicle.DTAR;
	const double SMALL_DISTANCE_FROM_TRAFFIC_LIGHT = 7;
	const double MID_DISTANCE_FROM_TRAFFIC_LIGHT = 20;
	const double LARGE_DISTANCE_FROM_TRAFFIC_LIGHT = 60;
	const double STOP_ZONE_ENDPT_BEYOND_LANESEG_END = 10;
	//const double SMALL_DISTANCE_FROM_TRAFFIC_LIGHT = 7;'
	//	const double TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGNAL_STATE = 10;
	auto timenow = BehaviorUtils::TimeNow();
	eTrafficSignalState CurrentTrafficSignalState = TRAF_SIG_UNKNOWN;
	double CurrentTrafficSignalStation;
	//BehaviorUtils::codeBlue( _CF_ +"inside HandleTrafficSignals StopZone = ",StopZone,1);


	try{
		BehaviorUtils::getGrpConn(TargetState.CurrentGrpLanes,TargetState.TargetGrpLanes,&connID);
	}catch(...){
		BehaviorUtils::codeRed( _CF_  + "In function HandleTrafficSignals() Couldn't find connection between {current , target } group " + CurrStepStr,\
				std::make_pair(TargetState.step.current_segment_group,TargetState.step.target_segment_group),TimeGap);
		return My_Maneuver_Temporal;
	}
	auto MyPair = std::make_pair(TargetState.CurrentGrpLanes.back(),connID);

	//static auto TimeInKnownState = BehaviorUtils::TimeNow();

	if(env_s<-STOP_ZONE_ENDPT_BEYOND_LANESEG_END){
		return My_Maneuver_Temporal;
	}

	if(behaviorinput::TrafficSignalStatesList.find(MyPair)!=behaviorinput::TrafficSignalStatesList.end()){
		exlcm::connectiontrafficsignal_t CurrentTrafficSignal;
		double CurrentTrafficSignalRcvTm;
		std::tie(CurrentTrafficSignal, CurrentTrafficSignalRcvTm)= behaviorinput::TrafficSignalStatesList[MyPair];
		CurrentTrafficSignalState = CurrentTrafficSignal.signal_state[0];
		CurrentTrafficSignalStation = CurrentTrafficSignal.restriction_station_m_start;

		if(timenow-CurrentTrafficSignalRcvTm > TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGNAL_STATE ){ // Dead reckon if not getting the traffic signal upfate
			//My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
			return My_Maneuver_Temporal;
		}


		// Holding the previous state if no useful updates are received (only if that's the front state)
		bool CurrentTrafficSignalStateAmbiguous = (CurrentTrafficSignalState == TRAF_SIG_STATE_NO_DETECTION)|| (CurrentTrafficSignalState == TRAF_SIG_UNKNOWN) || (CurrentTrafficSignalState == TRAF_SIG_OFF);

		if (CurrentTrafficSignalStateAmbiguous ){
			//My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
			if(env_s <LARGE_DISTANCE_FROM_TRAFFIC_LIGHT){
				if(LastValidTrafficSignalStates.find(MyPair)!=LastValidTrafficSignalStates.end()){
					if ((timenow-std::get<2>(LastValidTrafficSignalStates[MyPair]))<TIME_THRESH_TO_DEAD_RECKON_UNKNOWN_TRAFFICSIGNAL_STATE){
						CurrentTrafficSignalState = std::get<0>(LastValidTrafficSignalStates[std::make_pair(TargetState.CurrentGrpLanes.back(),connID)]);
						CurrentTrafficSignalStation = std::get<1>(LastValidTrafficSignalStates[std::make_pair(TargetState.CurrentGrpLanes.back(),connID)]);
					}

				}
			}
		}
		else{
			LastValidTrafficSignalStates[MyPair]= std::make_tuple(CurrentTrafficSignalState, CurrentTrafficSignalStation, BehaviorUtils::TimeNow());
			//	TimeInKnownState = BehaviorUtils::TimeNow();
		}

		Env::PolicyState NewTargetState = TargetState;
		bool NewStopZone = StopZone;
		if(UnmappedTrafficLight){
			My_Maneuver_Temporal.TemporalRule = TempRules::RIGHT_OF_WAY;
			double TargetStation = TargetState.step.current_segment_station_m;
			if (TargetStation<0)
				TargetStation = fabs(TargetStation) + BU::getSegLen(MyPair.first) ;
			NewTargetState.DAR_m = CurrentTrafficSignalStation- TargetStation + TargetState.DAR_m;
			if(NewTargetState.step.direction==TURN_RIGHT){
				if(RIGHT_ON_RED_ENABLE)
					CurrentTrafficSignalState=TRAF_SIG_STATE_FLASHING_RED;
			}
			if(vehicle.DTAR>NewTargetState.DAR_m)
				NewStopZone = true;
		}



		if((CurrentTrafficSignalState == TRAF_SIG_STATE_RED )|| (CurrentTrafficSignalState == TRAF_SIG_STATE_STRAIGHT_ARROW_RED ) || \
				(CurrentTrafficSignalState == TRAF_SIG_STATE_LEFT_ARROW_RED ) || (CurrentTrafficSignalState == TRAF_SIG_STATE_RIGHT_ARROW_RED ) || \
				(CurrentTrafficSignalState == TRAF_SIG_RAILROAD_CROSSING_RED )){
			My_TrafficLightType = RED_TYPE;
			My_Maneuver_Temporal =  My_Maneuver_Temporal + BehaviorSecondary::StopAtTrafficLight(NewTargetState,NewStopZone);
			BehaviorUtils::codeRed( _CF_  + "StopAtTrafficLight Behavior for RED TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			TargetState.observed_step_rule.insert(YIELD_UNTIL_CLEAR_AFTER_SIGNAL_RULE);
		}
		else if((CurrentTrafficSignalState == TRAF_SIG_STATE_YELLOW )|| (CurrentTrafficSignalState == TRAF_SIG_STATE_STRAIGHT_ARROW_YELLOW ) || \
				(CurrentTrafficSignalState == TRAF_SIG_STATE_LEFT_ARROW_YELLOW ) || (CurrentTrafficSignalState == TRAF_SIG_STATE_RIGHT_ARROW_YELLOW )){
			My_TrafficLightType = YELLOW_TYPE;
			double MAX_DECEL = 2;
			double MAX_ACCEL = 1;
			double TIME_TO_REACH_NEXT_SEG = 3;
			double TgtAccelForStop = BU::targetAcceleration(env_s,0);
			double env_s1 = env_s + NewTargetState.CurrentToTgtGrpConnLen;
			double TgtAccelForGO = BU::targetAcceleration(env_s1,CurrentspeedTarget);
			bool ShouldWeGo = ((LaneKeepBehavior.speedTargetDistance>env_s1)||(LaneKeepBehavior.speedTargetDistance<0));
			double tgtA,tgtV,tgtD=env_s1;
			std::tie(tgtV,tgtA) = BU::targetSpeedAndAccel(tgtD,TIME_TO_REACH_NEXT_SEG);
			if((tgtA>MAX_ACCEL)||(fabs(TgtAccelForStop)>MAX_DECEL))
				ShouldWeGo = false;
			else if(tgtV>CurrentspeedTarget){
				double TimeMaxAccel = (CurrentspeedTarget-vehicle.curSpeed_mps)/MAX_ACCEL;
				double DistMaxAccel = BU::targetDistance(CurrentspeedTarget,MAX_ACCEL);
				double DistMaxSpeed = env_s1 - DistMaxAccel;
				double TimeMaxSpeed = DistMaxSpeed/CurrentspeedTarget;
				double TimeToNextState = TimeMaxAccel + TimeMaxSpeed;
				if(TimeToNextState>TIME_TO_REACH_NEXT_SEG)
					ShouldWeGo = false;
			}

			//Lets keep it simple and overspeed if necessary now.

			if(!ShouldWeGo){;
			BehaviorUtils::codeYellow( _CF_ +"StopAtTrafficLight Behavior for YELLOW TrafficStates (STOP) " + CurrStepStr,CurrentTrafficSignalState,TimeGap);
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::StopAtTrafficLight(NewTargetState,NewStopZone);
			}
			else{
				BehaviorUtils::codeBlue( _CF_ +"Go to Next State Behavior for YELLOW TrafficStates as the target distance is "+ CurrStepStr,env_s,TimeGap);
				My_Maneuver_Temporal = BehaviorTertiary::Maneuver_Temporal(tgtA,tgtV,tgtD,LaneKeepBehavior.ProjectedAcceleration-0.5,TempRules::RIGHT_OF_WAY);
				//guaranteeing to win over Lane Keep
			}
			//TargetState.observed_step_rule.insert(ASSUME_RIGHT_OF_WAY_RULE);

		}
		else if((CurrentTrafficSignalState == TRAF_SIG_STATE_GREEN ) || (CurrentTrafficSignalState == TRAF_SIG_STATE_STRAIGHT_ARROW_GREEN ) || \
				(CurrentTrafficSignalState == TRAF_SIG_STATE_LEFT_ARROW_GREEN ) || (CurrentTrafficSignalState == TRAF_SIG_STATE_RIGHT_ARROW_GREEN )){
			My_TrafficLightType = GREEN_TYPE;
			BehaviorUtils::codeGreen( _CF_ +"Default Behavior for GREEN TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			//TargetState.observed_step_rule.insert(ASSUME_RIGHT_OF_WAY_RULE);
		}
		else if((CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_RED ) || (CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_LEFT_ARROW_RED ) || \
				(CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_RED )){
			My_TrafficLightType = FLASHING_RED_TYPE;
			BehaviorUtils::codeRed( _CF_  + "Stop and Yield Behavior for FLASHING RED TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::stopAtStopBar(NewTargetState) + BehaviorSecondary::YieldAtTrafficLight(NewTargetState);
			TargetState.observed_step_rule.insert(STOP_AND_YIELD_UNTIL_CLEAR_RULE);


		}
		else if ((CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_YELLOW ) ||  (CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_LEFT_ARROW_YELLOW ) || \
				(CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_RIGHT_ARROW_YELLOW )){
			// Need to be more cautious for TTC like evaluations
			My_TrafficLightType = FLASHING_YELLOW_TYPE;
			BehaviorUtils::codeYellow( _CF_ +"Default Behavior for FLASHING YELLOW TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::YieldAtTrafficLight(NewTargetState);
			//TargetState.observed_step_rule.insert(ASSUME_RIGHT_OF_WAY_RULE);
		}
		else if (CurrentTrafficSignalState == TRAF_SIG_STATE_FLASHING_GREEN){
			My_TrafficLightType = FLASHING_GREEN_TYPE;
			BehaviorUtils::codeRed( _CF_  + "Default Behavior for FLASHING GREEN TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			//TargetState.observed_step_rule.insert(ASSUME_RIGHT_OF_WAY_RULE);
		}
		else if (CurrentTrafficSignalState == TRAF_SIG_PEDISTRIAN_SAME_DIR_WARNING){
			My_TrafficLightType = PEDESTRIAN_TYPE;
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::CrossWalkBehavior(NewTargetState);
			BehaviorUtils::codeRed( _CF_  + "Yield to Pedestrian Behavior for PEDESTRIAN WARNING TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
			TargetState.observed_step_rule.insert(YIELD_FOR_PEDESTRIAN_RULE);
		}
		else if (CurrentTrafficSignalState == TRAF_SIG_PEDISTRIAN_SAME_DIR_CROSS){
			My_TrafficLightType = PEDESTRIAN_TYPE;
			My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorSecondary::StopAtTrafficLight(NewTargetState);
			My_Maneuver_Temporal.TemporalRule = YIELD;
			TargetState.observed_step_rule.insert(YIELD_FOR_PEDESTRIAN_RULE);
			BehaviorUtils::codeRed( _CF_  + "Stop and Yield Behavior for PEDESTRIAN CROSSING TrafficStates "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
		}
		else{
			My_TrafficLightType = UNKNOWN_TYPE;
			//	if(!UnmappedTrafficLight)
			//		My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
			BehaviorUtils::codeRed( _CF_  + "No Action taken for Traffic Signal State "+ CurrStepStr,CurrentTrafficSignalState,TimeGap);
		}



	}
	else{
		//if(!UnmappedTrafficLight)
		//	My_Maneuver_Temporal.TemporalRule = TempRules::NO_RULES;
		My_TrafficLightType = UNKNOWN_TYPE;
		if(behaviorinput::TrafficSignalStatesList.empty()){
			BehaviorUtils::codeRed( _CF_  + "Looks like behaviorinput::TrafficSignalStatesList is empty!!!"+ CurrStepStr," ",TimeGap);
		}
		else{
			BehaviorUtils::codeRed( _CF_  + "Couldn't find the desired connection lane segment pair. {LanseSegment UID, Connection No} "+ CurrStepStr,MyPair,TimeGap);
		}
	}

	if(BehaviorUtils::codeRed( _CF_  + "Final Signal Behavior "+ CurrStepStr," ",TimeGap)){
		lcmprint::PrintEnum(std::cout,(traffic_signal_state)CurrentTrafficSignalState);
		BehaviorSecondary::PrintEnum(std::cout,(eTrafficLightType)My_TrafficLightType);
		//	std::cout<<My_Maneuver_Temporal<<endl;
	}



	AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
	return My_Maneuver_Temporal;


}
