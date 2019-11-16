#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/BehaviorObject.hpp"
#include "../../include/Env.hpp"
#include "../../include/shared_enums.h"
#include "../../include/AgentPool.hpp"


struct  vehicle;



BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::YieldToPedestrian(Env::PolicyState &closestState,std::pair<ObjUidType, exlcm::objprediction_t> &obj,bool InParkingAisle){

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal= BehaviorTertiary::Default_Maneuver_Temporal;
	std::string objstr = std::to_string(obj.first);
	objstr = "objID:<"+objstr+">";
	const double Timegap = 2.5;
	My_Maneuver_Temporal.primary_target_object = obj.first;


	if (obj.second.intersection_uid == -1){
		if((obj.second.host_dist_to_conflict_m<50)&&(obj.second.time_to_collision_sec<1e6)){
			obj.second.intersection_uid = Env::_ClosestInterSectionInfo.first;
			BehaviorUtils::codeRed( _CF_  + "SA object intersection UID was -1. Being set to HV intersection UID " + objstr, Env::_ClosestInterSectionInfo.first,Timegap );
		}
		else{
			BehaviorUtils::codeRed( _CF_  + "SA object intersection UID is -1 for obj "+ objstr, obj.first,Timegap );
		}
	}

	if((obj.second.intersection_uid != Env::_ClosestInterSectionInfo.first )&&(!InParkingAisle)) {
		BehaviorUtils::codeRed( _CF_  + "Different intersections !!! {obj intersection_uid,HV intersection_uid} "+ objstr,std::make_pair(obj.second.intersection_uid,Env::_ClosestInterSectionInfo.first),Timegap);
		return My_Maneuver_Temporal;
	}

	int rel_loc = obj.second.relative_location;
	if(rel_loc==SA_INTERSECTING_FROM_LEFT ||rel_loc== SA_INTERSECTING_FROM_RIGHT|| rel_loc==SA_AHEAD)
	{

		auto closestStateRule = closestState.step.step_rule;
		//If the HV is near a STOP state and the yield state is pretty close
		My_Maneuver_Temporal.speedTargetDistanceActual = obj.second.host_dist_to_conflict_m; // If the ped walks in front of the cross walk we need to still make the target distance to the ped not the cross walk
		My_Maneuver_Temporal.primary_target_object = obj.first;
		if((fabs(closestState.DAR_m-vehicle.DTAR)<10) && (closestStateRule == STOP||closestStateRule == MULTIWAY_STOP||closestStateRule == RIGHT_OF_WAY_AFTER_SIGNAL||\
				closestStateRule == YIELD_UNTIL_CLEAR_AFTER_SIGNAL||closestStateRule == RIGHT_OF_WAY_AFTER_SIGNAL_STOP_ZONE||closestStateRule == STOP_AT_TRAFFIC_LIGHT)){
			My_Maneuver_Temporal =  BehaviorTertiary::Maneuver_Temporal(0,0,0,-999,YIELD);
			My_Maneuver_Temporal.speedTargetDistanceActual = closestState.DAR_m-vehicle.DTAR;
			BehaviorUtils::codeRed( _CF_  + "Stopping near stop zone as impending CROSSWALK not clear & collision possible" + objstr,obj.first);
		}
		else if ((obj.second.host_dist_to_conflict_m < PED_YIELD_DISTANCE)||(obj.second.time_to_collision_sec < PED_YIELD_TTC_THRESH)){
			My_Maneuver_Temporal =  BehaviorTertiary::Maneuver_Temporal(0,0,0,-999,YIELD);
			BehaviorUtils::codeRed( _CF_  + "CROSSWALK not clear & collision possible.Collision Predicted in (seconds) "+ objstr,obj.second.time_to_collision_sec,Timegap);
			BehaviorUtils::codeBlue( _CF_ +"Predicted distance to conflict ..."+ objstr,obj.second.host_dist_to_conflict_m);

		}
		else{
			My_Maneuver_Temporal = BehaviorTertiary::Maneuver_Temporal(0,0,obj.second.host_dist_to_conflict_m,\
					BehaviorUtils::targetAcceleration(obj.second.host_dist_to_conflict_m ,vehicle.curSpeed_mps,0),YIELD);
			BehaviorUtils::codeBlue( _CF_ +"Collision Predicted in (seconds) "+ objstr,obj.second.time_to_collision_sec);
			BehaviorUtils::codeBlue( _CF_ +"Predicted distance to conflict ..."+ objstr,obj.second.host_dist_to_conflict_m);
		}

	}
	else if (obj.second.time_to_collision_sec < PED_SLOWDOWN_TTC_THRESH){
		BehaviorUtils::codeGreen( _CF_ +"Non intersecting Pedestrian uid  ",obj.first ,1);
		if (BehaviorUtils::codeGreen( _CF_ +"Non intersecting Pedestrian relative location  "+ objstr,rel_loc ,Timegap))
			cout<<green_on<<static_cast<eSAObjLocation>(rel_loc)<<color_off<<endl;
	}


	BehaviorUtils::codeYellow( _CF_ +" Pedestrian speedTargetDistanceActual = "+ objstr,My_Maneuver_Temporal.speedTargetDistanceActual,Timegap);


	My_Maneuver_Temporal.primary_target_object = obj.first;
	return My_Maneuver_Temporal;
}


BehaviorTertiary::Maneuver_Temporal Maneuver_Pedestrian_Yield(std::vector<BehaviorTertiary::Maneuver_Temporal> All_Maneuver_CROSS_TRAFFIC,std::vector<double> All_Maneuver_CROSS_TRAFFIC_probs){
	BehaviorTertiary::Maneuver_Temporal Maneuver_Yield_VNM  = BehaviorTertiary::VNM_RationalizedAgent(All_Maneuver_CROSS_TRAFFIC,All_Maneuver_CROSS_TRAFFIC_probs);
	return Maneuver_Yield_VNM;
}


BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::CrossWalkBehavior(Env::PolicyState &TargetState)
{

	float env_s= TargetState.DAR_m-vehicle.DTAR;
	auto spdLimit = TargetState.step.posted_speed_lim_mps;
	float env_v=spdLimit;
	float env_a = BehaviorUtils::targetAcceleration(env_s,vehicle.curSpeed_mps,env_v);
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	BehaviorTertiary::Maneuver_Temporal Maneuver_YIELD_DYNAMIC;
	if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
		return My_Maneuver_Temporal;
	My_Maneuver_Temporal= BehaviorElementary::StaticlaneKeeping(TargetState) ;

	BehaviorTertiary::Maneuver_Temporal Maneuver_Temporal_LK = My_Maneuver_Temporal;

	if(env_s>50)
		return My_Maneuver_Temporal;


	ClockType timenow;
	timenow = std::chrono::high_resolution_clock::now();
	double epsilon = 15;

	Env::PolicyState closestState = Env::FindClosestState(TargetState);

	std::vector<std::future<BehaviorTertiary::Maneuver_Temporal> > CrossWalkBehaviorVec;

	for(auto & obj:SA.objData){

		if (1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>(timenow-SA.objDataUpdateClock[obj.second.uid]).count()) > 0.5){
			BehaviorUtils::codeRed( _CF_  + "This is an object from past  ",obj.second.uid);
			continue;  // No Update for this object received from SA for some time ...
		}



		if (!((obj.second.objtype==PEDESTRIAN_OBSTACLE_TYPE)||(obj.second.objtype==CYCLE_OBSTACLE_TYPE)))
			continue;

		std::vector<BehaviorTertiary::Maneuver_Temporal> All_Maneuver_YIELD_DYNAMIC;
		std::vector<double> All_Maneuver_YIELD_DYNAMIC_probs;
		for(int i=0;i<obj.second.valid_pred_count;i++){
			auto ObjHypothesis = std::make_pair(obj.first,obj.second.pred_obj_SA[i]);
			BehaviorTertiary::Maneuver_Temporal ThisYieldManuever = BehaviorSecondary::YieldToPedestrian(closestState,ObjHypothesis);
			All_Maneuver_YIELD_DYNAMIC.push_back(ThisYieldManuever);
			All_Maneuver_YIELD_DYNAMIC_probs.push_back(obj.second.pred_obj_probability[i]);
		}

		//Maneuver_YIELD_DYNAMIC = Maneuver_YIELD_DYNAMIC + BehaviorSecondary::YieldToPedestrian(std::make_pair(obj.first,obj.second.pred_obj_SA[0]));
	//	Maneuver_YIELD_DYNAMIC = Maneuver_YIELD_DYNAMIC + BehaviorTertiary::VNM_RationalizedAgent(All_Maneuver_YIELD_DYNAMIC,All_Maneuver_YIELD_DYNAMIC_probs);//BehaviorSecondary::YieldToPedestrian(std::make_pair(obj.first,obj.second.pred_obj_SA[0]));
		CrossWalkBehaviorVec.push_back(std::async(Maneuver_Pedestrian_Yield,All_Maneuver_YIELD_DYNAMIC,All_Maneuver_YIELD_DYNAMIC_probs));



	//	My_Maneuver_Temporal = My_Maneuver_Temporal + Maneuver_YIELD_DYNAMIC;

	}

	for(int i=0;i<CrossWalkBehaviorVec.size();i++){
		BehaviorTertiary::Maneuver_Temporal ThisPedManuever = CrossWalkBehaviorVec[i].get();
		My_Maneuver_Temporal = My_Maneuver_Temporal + ThisPedManuever;

	}



	AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
	return My_Maneuver_Temporal;
}


BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::NonCrossWalkPedestrianBehavior(Env::PolicyState &TargetState)
{


		BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
		if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
			return My_Maneuver_Temporal;
		My_Maneuver_Temporal= BehaviorElementary::StaticlaneKeeping(TargetState);


		Env::PolicyState closestState = Env::FindClosestState(TargetState);

		ClockType timenow;
		timenow = std::chrono::high_resolution_clock::now();
		double epsilon = 15;

		std::vector<std::future<BehaviorTertiary::Maneuver_Temporal> > CrossWalkBehaviorVec;

		for(auto & obj:SA.objData){

			if (1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>(timenow-SA.objDataUpdateClock[obj.second.uid]).count()) > 0.5){
				BehaviorUtils::codeRed( _CF_  + "This is an object from past  ",obj.second.uid);
				continue;  // No Update for this object received from SA for some time ...
			}



			if (!((obj.second.objtype==PEDESTRIAN_OBSTACLE_TYPE)||(obj.second.objtype==CYCLE_OBSTACLE_TYPE)))
				continue;

			std::vector<BehaviorTertiary::Maneuver_Temporal> All_Maneuver_YIELD_DYNAMIC;
			std::vector<double> All_Maneuver_YIELD_DYNAMIC_probs;
			for(int i=0;i<obj.second.valid_pred_count;i++){
				auto ObjHypothesis = std::make_pair(obj.first,obj.second.pred_obj_SA[i]);
				All_Maneuver_YIELD_DYNAMIC.push_back(BehaviorSecondary::YieldToPedestrian(closestState,ObjHypothesis,true));
				All_Maneuver_YIELD_DYNAMIC_probs.push_back(obj.second.pred_obj_probability[i]);
			}

			//Maneuver_YIELD_DYNAMIC = Maneuver_YIELD_DYNAMIC + BehaviorSecondary::YieldToPedestrian(std::make_pair(obj.first,obj.second.pred_obj_SA[0]));
		//	Maneuver_YIELD_DYNAMIC = Maneuver_YIELD_DYNAMIC + BehaviorTertiary::VNM_RationalizedAgent(All_Maneuver_YIELD_DYNAMIC,All_Maneuver_YIELD_DYNAMIC_probs);//BehaviorSecondary::YieldToPedestrian(std::make_pair(obj.first,obj.second.pred_obj_SA[0]));
			CrossWalkBehaviorVec.push_back(std::async(Maneuver_Pedestrian_Yield,All_Maneuver_YIELD_DYNAMIC,All_Maneuver_YIELD_DYNAMIC_probs));
		}

		for(int i=0;i<CrossWalkBehaviorVec.size();i++){
			BehaviorTertiary::Maneuver_Temporal ThisPedManuever = CrossWalkBehaviorVec[i].get();
			My_Maneuver_Temporal = My_Maneuver_Temporal + ThisPedManuever;

		}

		AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
		return My_Maneuver_Temporal;
}
