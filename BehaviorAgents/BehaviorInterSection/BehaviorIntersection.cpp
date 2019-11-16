#include "../../PCH/pch.hpp"

#include "../../include/BehaviorCals.hpp"
#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/Env.hpp"
#include "../../include/shared_enums.h"
#include "../../include/BehaviorObject.hpp"
#include "../../include/AgentPool.hpp"



struct  vehicle;
namespace BU=BehaviorUtils;
using namespace BehaviorElementary;


BehaviorTertiary::Maneuver_Temporal Maneuver_CROSS_TRAFFIC(std::vector<BehaviorTertiary::Maneuver_Temporal> All_Maneuver_CROSS_TRAFFIC,std::vector<double> All_Maneuver_CROSS_TRAFFIC_probs,\
		double  speedTargetDistanceActual){
	BehaviorTertiary::Maneuver_Temporal Maneuver_Yield_VNM  = BehaviorTertiary::VNM_RationalizedAgent(All_Maneuver_CROSS_TRAFFIC,All_Maneuver_CROSS_TRAFFIC_probs);
	if(Maneuver_Yield_VNM.primary_target_object>0)
		Maneuver_Yield_VNM.speedTargetDistanceActual = speedTargetDistanceActual;
	return Maneuver_Yield_VNM;
}

BehaviorTertiary::Maneuver_Temporal BehaviorTertiary::YieldToTraffic(Env::PolicyState& TargetState,int MajorBranchYieldStatus, bool CreepAllowed, bool NOYieldToDistCrossLanes)
{
	auto speedTarget=TargetState.step.posted_speed_lim_mps;
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
		return My_Maneuver_Temporal;

	BehaviorTertiary::Maneuver_Temporal  My_Manuever_LaneKeep = BehaviorElementary::StaticlaneKeeping(TargetState);//(-999,speedTarget,-999,+999,RIGHT_OF_WAY);//Fake accel to make sure it doesn't turn on
	My_Maneuver_Temporal = My_Manuever_LaneKeep;
	ClockType timenow;


	LaneSegUidType Approach_Lane_ID = -1;
	Approach_Lane_ID = TargetState.CurrentGrpLanes.back();

	static map <int,BehaviorTertiary::Maneuver_Temporal> PrevYieldManeuverAtIntersection ;
	static map <int,double> PrevYieldManeuverAtIntersectionTm ;

	int YieldZoneIdx= TargetState.step.step_index_n;


	std::vector<std::future<BehaviorTertiary::Maneuver_Temporal> > CrossTrafficBehaviorVec;
	std::vector<BehaviorTertiary::Maneuver_Temporal> ManeuverYieldVec;
	std::set<ObjUidType> ConsideredForYieldObjects;
	std::map<ObjUidType,double> ConsideredObjectsTimeDiff;
	timenow = std::chrono::high_resolution_clock::now();
	const double TIME_THRESH = 0.5;
	BehaviorTertiary::Maneuver_Temporal Maneuver_Yield_VNM;

	std::vector<BehaviorTertiary::Maneuver_Temporal> All_Maneuver_CROSS_TRAFFIC;
	std::vector<double> All_Maneuver_CROSS_TRAFFIC_probs;

	double EndOfCurrGrpDist = max(0.0,TargetState.GrpEnd_DAR_m-vehicle.DTAR);

	for(auto & obj:SA.objData)
	{

		double TimeDiff = 0;
		TimeDiff= 1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>(timenow-SA.objDataUpdateClock[obj.second.uid]).count());
		ConsideredObjectsTimeDiff[obj.first] = TimeDiff;


		if (!((obj.second.objtype==VEHICLE_OBSTACLE_TYPE)||(obj.second.objtype==TRUCK_OBSTACLE_TYPE)))
			continue;




		ConsideredForYieldObjects.insert(obj.first);
		All_Maneuver_CROSS_TRAFFIC.clear();
		All_Maneuver_CROSS_TRAFFIC_probs.clear();
		for(int i=0;i<obj.second.valid_pred_count;i++){
			auto ObjHypothesis = std::make_pair(obj.first,obj.second.pred_obj_SA[i]);

			if(obj.second.pred_obj_SA[i].x_m<0)
				continue;
			BehaviorTertiary::Maneuver_Temporal CrossTraffic = BehaviorSecondary::YieldToCrossTrafficObject(TargetState,ObjHypothesis,MajorBranchYieldStatus,NOYieldToDistCrossLanes);
			All_Maneuver_CROSS_TRAFFIC.push_back(CrossTraffic);
			All_Maneuver_CROSS_TRAFFIC_probs.push_back(obj.second.pred_obj_probability[i]);
		}

		//	BU::codeBlue( _CF_ +" All_Maneuver_CROSS_TRAFFIC_probs ",All_Maneuver_CROSS_TRAFFIC_probs,1);
		//	BU::codeBlue( _CF_ +" All_Maneuver_CROSS_TRAFFIC ",All_Maneuver_CROSS_TRAFFIC,1);

		Maneuver_Yield_VNM = Maneuver_CROSS_TRAFFIC(All_Maneuver_CROSS_TRAFFIC,All_Maneuver_CROSS_TRAFFIC_probs,Approach_Lane_ID);
		ManeuverYieldVec.push_back(Maneuver_Yield_VNM);
		CrossTrafficBehaviorVec.push_back(std::async(Maneuver_CROSS_TRAFFIC, All_Maneuver_CROSS_TRAFFIC, All_Maneuver_CROSS_TRAFFIC_probs, EndOfCurrGrpDist));




		//My_Maneuver_Temporal = My_Maneuver_Temporal + Maneuver_Yield_VNM;



	}


	for(int i=0;i<CrossTrafficBehaviorVec.size();i++)
		My_Maneuver_Temporal = My_Maneuver_Temporal + CrossTrafficBehaviorVec[i].get();

	BehaviorTertiary::Maneuver_Temporal Temp;
	std::vector<BehaviorTertiary::Maneuver_Temporal> Yield_Manuever_Stack;
	std::vector<double> Yield_Manuever_Probs = {0.4, 0.6};
	BehaviorTertiary::Maneuver_Temporal Filtered_Maneuver_Temporal;

	// Smooth out the maneuvers @ intersection using previous maneuvers
	if(CreepAllowed){
		if(BU::find(PrevYieldManeuverAtIntersection,YieldZoneIdx)){
			Yield_Manuever_Stack.push_back(PrevYieldManeuverAtIntersection[YieldZoneIdx]);
			Yield_Manuever_Stack.push_back(My_Maneuver_Temporal);
			Filtered_Maneuver_Temporal = BehaviorTertiary::VNM_RationalizedAgent(Yield_Manuever_Stack,Yield_Manuever_Probs);
			Temp = My_Maneuver_Temporal;
			My_Maneuver_Temporal = My_Maneuver_Temporal + Filtered_Maneuver_Temporal;
		}
	}
	if((BU::ModuleTimeNow()-PrevYieldManeuverAtIntersectionTm[YieldZoneIdx]>0.01)){
		PrevYieldManeuverAtIntersection[YieldZoneIdx] = My_Maneuver_Temporal;
		PrevYieldManeuverAtIntersectionTm[YieldZoneIdx] = BU::ModuleTimeNow();
	}


	BehaviorTertiary::Maneuver_Temporal CrossWalkManeuver = BehaviorSecondary::CrossWalkBehavior(TargetState);



	// My_Maneuver_Temporal = My_Maneuver_Temporal + CrossWalkManeuver;//May not want this indiscrimately.


	AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
	return My_Maneuver_Temporal;
}

std::pair<InterSectionUidType, double> _ClosestInterSectionInfo;

BehaviorTertiary::Maneuver_Temporal BehaviorTertiary::InterSectionCreepManeuver(Env::PolicyState& TargetState){

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal = BehaviorSecondary::stopAtStopBar(TargetState);
	My_Maneuver_Temporal.TemporalRule=INTERSECTION;

	return My_Maneuver_Temporal;

}


BehaviorTertiary::Behavior BehaviorTertiary::InterSectionManeuver(Env::PolicyState& TargetState, bool MinorConfirmed){

	Env::PolicyState PrevState = Env::prev(TargetState);
	int CurrentGrp = TargetState.step.current_segment_group;
	_ClosestInterSectionInfo = Env::FindClosestInterSectionInfo(TargetState); // Have to look @ two rules including just entered intersection
	BehaviorTertiary::Behavior MyInterSectionBehavior;
	MyInterSectionBehavior.TargetState = TargetState;
	if(AgentPool::SeekAgentFromPool(MyInterSectionBehavior,__FUNCTION__))
		return MyInterSectionBehavior;
	MyInterSectionBehavior = BehaviorTertiary::ControlComplexBehavior(TargetState,MinorConfirmed);

	if(_ClosestInterSectionInfo.second<INTERSECTION_VICINITY_THRESHOLD)
		MyInterSectionBehavior = MyInterSectionBehavior + BehaviorTertiary::Behavior(PrevState);


	AgentPool::StoreAgentInPool(MyInterSectionBehavior,__FUNCTION__);
	return MyInterSectionBehavior;
}


BehaviorTertiary::Behavior BehaviorTertiary::ControlComplexBehavior(Env::PolicyState& TargetState, bool MinorConfirmed){

	const double TimeGap = 7.5;
	const int UID_idx = 0,  Exit_idx = 0, Entry_idx = 1, PtOfNoreturn_idx = 2;
	//Env::PolicyState InterSectionTargetState = TargetState;
	BehaviorTertiary::Behavior BehaviorDesiredLK = BehaviorElementary::laneKeepBehavior(TargetState);
	BehaviorTertiary::Behavior BehaviorDesired = BehaviorDesiredLK;

	static std::map<InterSectionUidType,double> CreepStartTime;

	auto TargetStep = TargetState.step;
	int StepIdx = TargetStep.step_index_n;
	int CurrentGrp = TargetStep.current_segment_group;
	std::string CurrGrpStr = "<StepIdx:"+std::to_string(StepIdx)+">";

	double SpdRef = TargetStep.posted_speed_lim_mps;
	double INTERSECTION_FOV_DIST_THRESH = SA.cross_lane_SpdLmt()*INTERSECTION_FOV_TTC_THRESH;
	bool MajorBranch = false;
	bool IsThisATurnTask = (TargetStep.step_task == TURN_TASK);


	bool PtOfNoReturnCrossed = false;
	bool CreepAllowed = true;


	MajorBranch = (SA.HVLaneYieldPriority(_ClosestInterSectionInfo.first)==LN_DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY);//( IsThisATurnTask || !RightOfWay || MinorConfirmed);
	int MajorBranchYieldStatus = SA.HVLaneYieldPriority(_ClosestInterSectionInfo.first);
	bool Entered = false;

	bool MinorBranch = (SA.HVLaneYieldPriority(_ClosestInterSectionInfo.first)==LN_YIELD_DUE_TO_STATIC_INFERIORITY);

	//BehaviorUtils::codeRed( _CF_  + "RightOfWay "+ CurrGrpStr,RightOfWay,1);
	BehaviorUtils::codeRed( _CF_  + "MajorBranch "+ CurrGrpStr,MajorBranch,1);

	BehaviorTertiary::Maneuver_Temporal Temporal_INT = BehaviorTertiary::YieldToTraffic(TargetState,MajorBranch,false);
	BehaviorTertiary::Maneuver_Temporal YIELD_ONLY_TO_NEAR_CROSS_LANES = BehaviorTertiary::YieldToTraffic(TargetState,MajorBranch,false,true);


	if(_ClosestInterSectionInfo.second<INTERSECTION_VICINITY_THRESHOLD)
	{


		//	BU::codeBlue(_CF_ + "PrevState "+ CurrGrpStr,PrevState,1);

		BehaviorUtils::codeGreen( _CF_ +"--------------------------- Beginning of Intersection ------->"+ CurrGrpStr,_ClosestInterSectionInfo.first,1e3);

		BehaviorUtils::codeGreen( _CF_ +" Closest intersection distance = "+ CurrGrpStr,_ClosestInterSectionInfo.second,TimeGap);


		if (Env::_ClosestInterSectionInfo.first==_ClosestInterSectionInfo.first){
			if(vehicle.curConnCompletionRatio>0.8)
				_InterSectionEntryExitLog[_ClosestInterSectionInfo.first] = std::make_tuple( true, true , true );
			else if(vehicle.curConnCompletionRatio>0.4)
				_InterSectionEntryExitLog[_ClosestInterSectionInfo.first] = std::make_tuple( false, true , true );
			else
				_InterSectionEntryExitLog[_ClosestInterSectionInfo.first] = std::make_tuple( false, true , false );
		}
		else{
			_InterSectionEntryExitLog[_ClosestInterSectionInfo.first] = std::make_tuple( false, false, false );
		}
		Entered = std::get<Entry_idx>(_InterSectionEntryExitLog[_ClosestInterSectionInfo.first]);

		auto SetPtOfNoreturn = [] (std::tuple<bool,bool,bool>& MyTuple, bool value)  {
			std::get<PtOfNoreturn_idx>(MyTuple) = value;
		}; // lambda here


		if(vehicle.curConnCompletionRatio>0.1){
			PtOfNoReturnCrossed = (SA.cross_lane_dist()<VEHICLE_LENGTH/2)||std::get<PtOfNoreturn_idx>(_InterSectionEntryExitLog[_ClosestInterSectionInfo.first]);
			CreepAllowed = (SA.cross_view_range()>INTERSECTION_FOV_DIST_THRESH);
			CreepAllowed = CreepAllowed && (!PtOfNoReturnCrossed);
		}


		if(PtOfNoReturnCrossed){
			SetPtOfNoreturn(_InterSectionEntryExitLog[_ClosestInterSectionInfo.first],true);
		}



		BU::codeYellow( _CF_ +" _ClosestInterSectionInfo " + CurrGrpStr, _ClosestInterSectionInfo,1);
		BU::codeYellow( _CF_ +" Entered " + CurrGrpStr, Entered,1);
		BU::codeYellow( _CF_ +" PtOfNoReturnCrossed " + CurrGrpStr, PtOfNoReturnCrossed,1);
		BU::codeYellow( _CF_ +" _InterSectionEntryExitLog " + CurrGrpStr, _InterSectionEntryExitLog[_ClosestInterSectionInfo.first],1);



	//	BU::codeYellow( _CF_ +" OptCreepStation " + CurrGrpStr, SA.OptCreepStation(_ClosestInterSectionInfo.first),0.01);
		if( Entered && !PtOfNoReturnCrossed ){

			//TargetStep.step_rule=INTERSECTION;



			if(IsThisATurnTask&&(!MajorBranch)){
				if(CreepAllowed){
					BehaviorDesired.BehaviorAccelRequestType = SLOW_ACCEL_REQUEST_BEHAVIOR;
					Env::PolicyState CreepState = TargetState;
					CreepState.DAR_m = TargetState.GrpEnd_DAR_m + SA.OptCreepStation(_ClosestInterSectionInfo.first);
					if(CreepState.DAR_m<vehicle.DTAR)
						if(!BU::find(CreepStartTime,_ClosestInterSectionInfo.first))
							CreepStartTime[_ClosestInterSectionInfo.first]=BU::TimeNow();
					if(MinorBranch)
						BehaviorDesired = BehaviorDesired + BehaviorTertiary::InterSectionCreepManeuver(CreepState);
				}
				else{
					BehaviorDesired.BehaviorAccelRequestType = AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR;
				}
			}


			//BU::codeGreen( _CF_ +"Maneuver Temporal INT : "+ CurrGrpStr,Temporal_INT,1);
			//	BU::codeBlue(_CF_ + "Executing Intersection "+ CurrGrpStr,TargetState,1);
	/*		if(BU::find(CreepStartTime,_ClosestInterSectionInfo.first)){
				if(BU::TimeNow()-CreepStartTime[_ClosestInterSectionInfo.first]>INTERSECTION_TM_THRESHOLD_FOR_FAR_LANE_THREAT_ASSESSMENT)
					BehaviorDesired = BehaviorDesired + YIELD_ONLY_TO_NEAR_CROSS_LANES ;
			}
			else */
				BehaviorDesired = BehaviorDesired + Temporal_INT ;


		}
		else if( Entered && PtOfNoReturnCrossed){
			if(IsThisATurnTask)
				BehaviorDesired.BehaviorAccelRequestType = AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR;
		}
		else{

		}

	}
	else{
		_InterSectionEntryExitLog[_ClosestInterSectionInfo.first] = std::make_tuple(false , false, false ); // UID,Exit,Entry,Point of No Return
		//BU::codePurple( _CF_ +" _ClosestInterSectionInfo " + CurrGrpStr, _ClosestInterSectionInfo,1);
	}




	BehaviorTertiary::Behavior BehaviorTurn = BehaviorElementary::turn(TargetState);
	BehaviorDesired = BehaviorDesired + BehaviorTurn;



	return BehaviorDesired;
}

