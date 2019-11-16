#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/shared_enums.h"
#include "../../include/BehaviorObject.hpp"
#include "../../include/AgentPool.hpp"



namespace BU=BehaviorUtils;

BehaviorTertiary::Maneuver_Temporal ObjCollisionImminentBehavior(exlcm::objsituation_t MyObj){
	BehaviorTertiary::Maneuver_Temporal  My_Maneuver_Temporal;
	ClockType timenow = std::chrono::high_resolution_clock::now();


	if(MyObj.valid_pred_count<=0)
		return My_Maneuver_Temporal;

	if(MyObj.pred_obj_SA[0].relative_location==SA_NOT_NEAR) // Just use the highest probability hypothesis for Collision Imminent Behavior
		return My_Maneuver_Temporal;

	if(!MyObj.pred_obj_SA[0].collision_possible)
		return My_Maneuver_Temporal;

	if (1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>(timenow-SA.objDataUpdateClock[MyObj.uid]).count()) > 0.25)
		return My_Maneuver_Temporal;

	if(MyObj.pred_obj_SA[0].host_dist_to_conflict_m > COLLISION_IMMINENT_DISTANCE_THRESH)
		return My_Maneuver_Temporal;

	if(MyObj.pred_obj_SA[0].time_to_collision_sec>COLLISION_IMMINENT_TTC_THRESH)
		return My_Maneuver_Temporal;

	My_Maneuver_Temporal =  BehaviorSecondary::holdStillUntillIntersectionIsClear(); // This will get more complicated later on.
	My_Maneuver_Temporal.primary_target_object = MyObj.uid; // I guess any one of the collision possible objects could be a target, in case there are multiple
	My_Maneuver_Temporal.TemporalRule = COLLISION_IMMINENT;
	BehaviorUtils::codeRed( _CF_  + " COLLISION_IMMINENT Behavior objects is", MyObj,1);

	return My_Maneuver_Temporal;
}

BehaviorTertiary::Maneuver_Temporal BehaviorElementary::CollisionImminentBehavior(){
	ClockType timenow = std::chrono::high_resolution_clock::now();
	BehaviorTertiary::Maneuver_Temporal  My_Maneuver_Temporal;

	std::vector<std::future<BehaviorTertiary::Maneuver_Temporal> > CollisionImminentBehaviorVec;


	for(auto & obj:SA.objData)
		CollisionImminentBehaviorVec.push_back(std::async(ObjCollisionImminentBehavior,obj.second));


	for(int i=0;i<CollisionImminentBehaviorVec.size();i++){
		My_Maneuver_Temporal = My_Maneuver_Temporal + CollisionImminentBehaviorVec[i].get();
		if(My_Maneuver_Temporal.TemporalRule == COLLISION_IMMINENT)
			return My_Maneuver_Temporal;

	}

	return My_Maneuver_Temporal;

}


BehaviorTertiary::Behavior  BehaviorElementary::GetMaxSpeedBehavior(const Env::PolicyState &TargetState,double SpdFactor ){
	auto speedTarget=TargetState.step.posted_speed_lim_mps*SpdFactor;
	//vehicle.behaviorType=LANE_KEEPING_BEHAVIOR;
	double my_ProjectedAcceleration = BehaviorUtils::targetAcceleration(SA_LOOK_AHEAD_HORIZON,speedTarget);
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal(my_ProjectedAcceleration,speedTarget,-999, my_ProjectedAcceleration ,RIGHT_OF_WAY);
	 // my_accelTarget, my_speedTarget, my_speedTargetDistance, my_ProjectedAcceleration,	 my_TemporalRule
	BehaviorTertiary::Maneuver_Spatial  My_Maneuver_Spatial( LANE_KEEPING_MANEUVER, LANE_CHANGE_IS_PROHIBITED,LANE_CHANGE_IS_PROHIBITED,0,\
			0, PROCEED_TASK ,BehaviorTertiary::TURN_INDICATOR_NONE );
	BehaviorTertiary::Behavior My_Behavior(My_Maneuver_Spatial,My_Maneuver_Temporal);
	My_Behavior.TargetState = TargetState;
	return My_Behavior;
}

BehaviorTertiary::Behavior BehaviorElementary::AdjacentlaneKeepBehavior(Env::PolicyState &TargetState,int LANE_SITUATION,int location){
	BehaviorTertiary::Behavior My_Behavior = GetMaxSpeedBehavior(TargetState);
	BehaviorTertiary::Maneuver_Temporal Maneuver_ACC;
	int LaneObjIdx,hypothesisIdx;
	ObjUidType ObjUID;
	ObjUID = SA.ahead_obj_id(LANE_SITUATION,location,&LaneObjIdx,&hypothesisIdx);
	if(location==0)
		Maneuver_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead(LANE_SITUATION),ObjUID,hypothesisIdx);
	else
		Maneuver_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead(LANE_SITUATION,location),ObjUID,hypothesisIdx);
	My_Behavior = My_Behavior + Maneuver_ACC;

	return My_Behavior;
}


BehaviorTertiary::Behavior BehaviorElementary::SimplelaneKeepBehavior(Env::PolicyState &TargetState){
	BehaviorTertiary::Behavior My_Behavior= BehaviorElementary::StaticlaneKeeping(TargetState);
	My_Behavior = My_Behavior + BehaviorElementary::AdaptiveCruise(TargetState);
	My_Behavior.TargetState = TargetState;
	return My_Behavior;
}


BehaviorTertiary::Behavior BehaviorElementary::laneKeepBehavior(Env::PolicyState &TargetState){
	std::string CurrGrpStr = "<step_hash:"+std::to_string(HashPathStep(TargetState.step))+">";
	int Task = TargetState.step.step_task;
	BehaviorTertiary::Behavior MyBehavior;
	MyBehavior.TargetState = TargetState;
	if(AgentPool::SeekAgentFromPool(MyBehavior,__FUNCTION__))
		return MyBehavior;

	MyBehavior = BehaviorElementary::StaticlaneKeeping(TargetState); //BehaviorElementary::GetMaxSpeedBehavior(TargetState) ;
	MyBehavior.SpatialTask = Task;


	int hypothesisIdx,LaneObjIdx;
	ObjUidType ObjUID;
	BehaviorTertiary::Maneuver_Temporal Maneuver_Pure_ACC;

	double clear_dist_offset = 0;
	double env_s = TargetState.GrpOrigin_DAR_m - vehicle.DTAR;

	bool IsThisaSharedLaneTask = (Task==PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK)||(Task==PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK)||(Task==PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK);

	if(SA.ahead_obj_id(HV_LANE_SITUATION)>0){
	    ObjUID = SA.ahead_obj_id(HV_LANE_SITUATION,0,&LaneObjIdx,&hypothesisIdx);
		if(env_s>OPPORTUNISTIC_LCX_EOS_DIST_THRESHOLD){
			if(BU::ExtractParamFromObjSA(ObjUID,"stationary_status")[hypothesisIdx]==STATIONARY_STATUS_STATIONARY)
				if(IsThisaSharedLaneTask)
					clear_dist_offset = -SAFE_FOLLOWING_ADDOFFSET_STATIONARY;

		}
	//	cout<<"clear_dist_offset "<<clear_dist_offset<<" env_s "<<env_s<<endl;
	    double clear_dist_ahead = SA.clear_dist_ahead(HV_LANE_SITUATION);
		Maneuver_Pure_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,clear_dist_ahead+clear_dist_offset,ObjUID,hypothesisIdx);
		MyBehavior =  MyBehavior + Maneuver_Pure_ACC;
	/*	if(BU::codeGreen( _CF_ +" Maneuver_Pure_ACC: ",Maneuver_Pure_ACC,1)){
			cout<<"hypothesisIdx "<<hypothesisIdx<<" ObjUID "<<ObjUID<<" SA.clear_dist_ahead(HV_LANE_SITUATION) "<<SA.clear_dist_ahead(HV_LANE_SITUATION)<<endl;
		} */


	}



	std::vector<std::future<BehaviorTertiary::Maneuver_Temporal> > SharedLaneObjBehaviorVec;


	if(Task==PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK){

		for(auto & obj:SA.objData){
			if (!((obj.second.objtype==VEHICLE_OBSTACLE_TYPE)||(obj.second.objtype==TRUCK_OBSTACLE_TYPE)||(obj.second.objtype==PEDESTRIAN_OBSTACLE_TYPE)))
				continue;

			MyBehavior = MyBehavior + BehaviorElementary::SharedLaneACCObj(TargetState,obj.second.uid,SA_AHEAD_LEFT_EDGE);
		//	MyBehavior.SpatialTask = PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK;



		}

		//for(int i=0;i<SharedLaneObjBehaviorVec.size();i++)
		//	MyBehavior = MyBehavior + SharedLaneObjBehaviorVec[i].get();
		MyBehavior.SpatialTask = PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK;

	}
	else if((Task==PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK)||(Task==PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK)){



		for(auto & obj:SA.objData){
			if (!((obj.second.objtype==VEHICLE_OBSTACLE_TYPE)||(obj.second.objtype==TRUCK_OBSTACLE_TYPE)||(obj.second.objtype==PEDESTRIAN_OBSTACLE_TYPE)))
				continue;

			SharedLaneObjBehaviorVec.push_back(std::async(std::launch::deferred,BehaviorElementary::SharedLaneACCObj,TargetState,obj.second.uid,SA_AHEAD_RIGHT_EDGE));


		}


		for(int i=0;i<SharedLaneObjBehaviorVec.size();i++)
			MyBehavior = MyBehavior + SharedLaneObjBehaviorVec[i].get();
		MyBehavior.SpatialTask = PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK;

		//	BU::codeBlue( _CF_ +" Neighborhood ACC Behavior ",MyBehavior,1);
	}
	else if(Task==PROCEED_THROUGH_PARKING_AISLE_TASK){



		for(auto & obj:SA.objData){
			if (!((obj.second.objtype==VEHICLE_OBSTACLE_TYPE)||(obj.second.objtype==TRUCK_OBSTACLE_TYPE)||(obj.second.objtype==PEDESTRIAN_OBSTACLE_TYPE)))
				continue;

			SharedLaneObjBehaviorVec.push_back(std::async(BehaviorElementary::SharedLaneACCObj,TargetState,obj.second.uid,SA_AHEAD_RIGHT_EDGE));


		}


		for(int i=0;i<SharedLaneObjBehaviorVec.size();i++)
			MyBehavior = MyBehavior + SharedLaneObjBehaviorVec[i].get();
		MyBehavior.SpatialTask = PROCEED_THROUGH_PARKING_AISLE_TASK;

		MyBehavior = MyBehavior + BehaviorSecondary::NonCrossWalkPedestrianBehavior(TargetState);

	}
	else{
		MyBehavior =  BehaviorElementary::SimplelaneKeepBehavior(TargetState);
	}

	//BU::codeBlue( _CF_ +"Inside laneKeepBehavior() MyBehavior  "+CurrGrpStr,MyBehavior,1);

	AgentPool::StoreAgentInPool(MyBehavior,__FUNCTION__);

	return MyBehavior;
}

BehaviorTertiary::Behavior BehaviorElementary::StaticlaneKeeping(Env::PolicyState &TargetState,double SpdFactor)
{
	BehaviorTertiary::Behavior My_Behavior;
	My_Behavior.TargetState = TargetState;
	if(AgentPool::SeekAgentFromPool(My_Behavior,__FUNCTION__))
		return My_Behavior;

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	BehaviorTertiary::Maneuver_Spatial  My_Maneuver_Spatial;


	My_Behavior = GetMaxSpeedBehavior(TargetState,SpdFactor);
	My_Maneuver_Spatial = My_Behavior; //slicing
	My_Maneuver_Temporal = My_Behavior;




	BehaviorTertiary::Behavior MyBehavior(My_Maneuver_Spatial,My_Maneuver_Temporal);





	MyBehavior = MyBehavior + BehaviorTertiary::Behavior(My_Behavior,My_Maneuver_Spatial,My_Maneuver_Temporal);


	AgentPool::StoreAgentInPool(MyBehavior,__FUNCTION__, true);

	return MyBehavior;
}


BehaviorTertiary::Behavior& BehaviorElementary::CreateAndAddDefaultManeuvers(BehaviorTertiary::Behavior& BehaviorDesired){
	/* PG: Need to check if being used 
	auto DefaultspeedTarget=BehaviorDesired.TargetState.step.posted_speed_lim_mps;
	auto DefaultAccelTarget = BehaviorUtils::targetAcceleration(SA_LOOK_AHEAD_HORIZON,vehicle.curSpeed_mps,DefaultspeedTarget);
	BehaviorTertiary::Default_Maneuver_Temporal = 
		BehaviorTertiary::Maneuver_Temporal(DefaultAccelTarget,DefaultspeedTarget,-999,DefaultAccelTarget,RIGHT_OF_WAY);
	BehaviorTertiary::Default_Maneuver_Temporal = 
		BehaviorElementary::StaticlaneKeeping(Policy::ActiveDetailedPolicy.PolicyDQ[Policy::ActiveDetailedPolicy.Front]);*/

	return BehaviorDesired;
}



