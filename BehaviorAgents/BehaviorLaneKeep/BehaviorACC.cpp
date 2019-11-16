#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"

#include "../../include/shared_enums.h"
#include "../../include/BehaviorObject.hpp"


namespace BU=BehaviorUtils;



BehaviorTertiary::Maneuver_Temporal BehaviorElementary::AdaptiveCruiseObj(Env::PolicyState &TargetState,double clear_dist_ahead, double target_speed_ahead, double target_accel_ahead,ObjUidType objID){

	const double TimeGap = 10;
	const int  Exit_idx = 0, Entry_idx = 1, PtOfNoreturn_idx = 2;
	double DefaultspeedTarget=TargetState.step.posted_speed_lim_mps;
	double env_v = 0.0;

	if(target_speed_ahead>SAFE_FOLLOWING_MIN_SPEED)
		env_v = target_speed_ahead;
	env_v = fmin(env_v,SAFE_FOLLOWING_MAX_SPEED);
	double env_s = (clear_dist_ahead);
	double env_a = BehaviorUtils::targetAcceleration(env_s,env_v);
	BehaviorTertiary::Maneuver_Temporal  My_Maneuver_Temporal = BehaviorTertiary::Maneuver_Temporal\
			(env_a,DefaultspeedTarget,SA_LOOK_AHEAD_HORIZON,std::numeric_limits<double>::max(),TempRules::NO_RULES);
	My_Maneuver_Temporal.primary_target_object = objID;
	TempRules ACC_Rule=TempRules::ACC;
	if(SA.objData[objID].objtype==PEDESTRIAN_OBSTACLE_TYPE){
		ACC_Rule = TempRules::YIELD;
		env_s -=SAFE_PEDESTRIAN_YIELDING_OFFSET_m;
	}

	BehaviorTertiary::Maneuver_Temporal STOPPED_ACC(0,0,0,-998,ACC_Rule); // Fake accel value to make it the winning behavior.

	bool SlowMovingObjTooClosetoSlowHV = ((env_v <SAFE_FOLLOWING_MIN_SPEED )&&(vehicle.curSpeed_mps<SAFE_FOLLOWING_HV_MIN_SPEED)&&(env_s<SAFE_FOLLOWING_MIN_DIST));
	if(SlowMovingObjTooClosetoSlowHV||(env_s< SAFE_FOLLOWING_CRITICAL_MIN_DIST)){
		STOPPED_ACC.primary_target_object = objID;
		STOPPED_ACC.speedTargetDistanceActual = 0;
		return STOPPED_ACC;
	}

	//BehaviorUtils::codeGreen( _CF_ +" SAFE_FOLLOWING_MIN_SPEED ",SAFE_FOLLOWING_MIN_SPEED,0.5);

	if(clear_dist_ahead<=BEHAVIOR_LOOK_AHEAD_HORIZON){
		BehaviorTertiary::Maneuver_Temporal Maneuver_Smooth_ACC(env_a,env_v,env_s,env_a,ACC_Rule);
		Maneuver_Smooth_ACC.speedTargetDistanceActual = clear_dist_ahead;
		Maneuver_Smooth_ACC.primary_target_object = objID;
		if(_InterSectionEntryExitLog.find(Env::_ClosestInterSectionInfo.first)!=_InterSectionEntryExitLog.end()){
			bool Entered = std::get<Entry_idx>(_InterSectionEntryExitLog[Env::_ClosestInterSectionInfo.first]);
			bool PtOfNoreturnCrossed = std::get<PtOfNoreturn_idx>(_InterSectionEntryExitLog[Env::_ClosestInterSectionInfo.first]);
			bool NotExitedInterSection = !(std::get<Exit_idx>(_InterSectionEntryExitLog[Env::_ClosestInterSectionInfo.first]));
			if(PtOfNoreturnCrossed && NotExitedInterSection){
				Maneuver_Smooth_ACC.Temporal_cost_bias = 0.0 ;
				Maneuver_Smooth_ACC.CalculateCost();
			}
		}
		My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_Smooth_ACC;
	}



	return  My_Maneuver_Temporal;
}

BehaviorTertiary::Maneuver_Temporal BehaviorElementary::AdaptiveCruiseObj(Env::PolicyState &TargetState,double clear_dist_ahead, ObjUidType objID, int HypIdx){






	double half_length = SA.objData[objID].length_m/2;
	double safe_offset = max(half_length ,SAFE_FOLLOWING_OFFSET/2)+SAFE_FOLLOWING_OFFSET/2;
	clear_dist_ahead-=safe_offset;
	double SpeedTgt = BU::ExtractParamFromObjSA(objID,"vel")[HypIdx];
	double AccelTgt = BU::ExtractParamFromObjSA(objID,"accel")[HypIdx];

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal = BehaviorElementary::AdaptiveCruiseObj(TargetState,clear_dist_ahead,SpeedTgt,AccelTgt,objID);


	return My_Maneuver_Temporal;
}

BehaviorTertiary::Maneuver_Temporal BehaviorElementary::SharedLaneACCObj(Env::PolicyState TargetState, ObjUidType objID, int ExpectedStaticObjLoc){  // Mainly devised for neighborhood ACC
	BehaviorTertiary::Maneuver_Temporal NoACC = BehaviorElementary::StaticlaneKeeping(TargetState) ;
	NoACC.TemporalRule = RIGHT_OF_WAY;

	if(TEMP_DISALLOW_ACC)
		return  NoACC;



	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	My_Maneuver_Temporal.TemporalRule = TempRules::RIGHT_OF_WAY;
	std::vector<BehaviorTertiary::Maneuver_Temporal> ACC_Maneuvers;
	std::vector<double> ACC_Maneuver_Probabilities;
	exlcm::objsituation_t objSA = SA.objData[objID];

	if (!((objSA.objtype==VEHICLE_OBSTACLE_TYPE)||(objSA.objtype==TRUCK_OBSTACLE_TYPE)||(objSA.objtype==PEDESTRIAN_OBSTACLE_TYPE)))
		return My_Maneuver_Temporal;



	for(int i=0;i<objSA.pred_obj_SA.size();i++){
		exlcm::objprediction_t objPred = objSA.pred_obj_SA[i];
		int relative_location = objPred.relative_location;
		bool Objexpected_to_yield_f = (objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY || objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT || \
				objPred.expected_to_yield_f == YIELD_DUE_TO_ARRIVAL_INFERIORITY );
		bool collision_possible = objPred.collision_possible;



		if(relative_location==SA_NOT_NEAR)
			continue;
		else if(relative_location==SA_AHEAD){ //Assume this is parallel as lane SA ahead is taken care in the lane keep
			int hypothesisIdx;
			SA.parallel_obj_uid(HV_LANE_SITUATION,0,&hypothesisIdx);
			double clear_dist_parallel = SA.clear_dist_ahead_parallel(HV_LANE_SITUATION);
			ACC_Maneuvers.push_back( BehaviorElementary::AdaptiveCruiseObj(TargetState,clear_dist_parallel,objID,hypothesisIdx));
			ACC_Maneuver_Probabilities.push_back(objSA.pred_obj_probability[i]);
		}
		else if ((relative_location==SA_MERGING_FROM_LEFT)||(relative_location==SA_MERGING_FROM_RIGHT)){
			if(!Objexpected_to_yield_f && collision_possible){
				ACC_Maneuver_Probabilities.push_back(objSA.pred_obj_probability[i]);
				ACC_Maneuvers.push_back( BehaviorElementary::AdaptiveCruiseObj( TargetState,objPred.host_dist_to_conflict_m,  objID,  i) );
			}
		}
		else if ((relative_location==SA_INTERSECTING_FROM_LEFT) || (relative_location == SA_INTERSECTING_FROM_RIGHT) ){
			if(!Objexpected_to_yield_f && collision_possible){
				ACC_Maneuvers.push_back( BehaviorElementary::AdaptiveCruiseObj( TargetState,objPred.host_dist_to_conflict_m, 0,0, objID) );
				ACC_Maneuver_Probabilities.push_back(objSA.pred_obj_probability[i]);
			}
		}
		else if (relative_location==SA_ONCOMING){
			double distFromOncomingObj = SA.clear_dist_ahead(HV_LANE_SITUATION);
			if ((objPred.x_m<0)||(objPred.Stationary_Status==STATIONARY_STATUS_PARKED_ROADSIDE)||(objPred.Stationary_Status==STATIONARY_STATUS_PARKED_PARKING_AISLE))
				continue;
			if(collision_possible&& (distFromOncomingObj<SHARED_LANE_RV_BOTTLENECK_DIST_THRESH)){ // better be
				/*	BehaviorTertiary::Maneuver_Temporal  YieldToOncomingObj = BehaviorSecondary::holdStillUntillIntersectionIsClear();
				YieldToOncomingObj.primary_target_object = objID;
				ACC_Maneuvers.push_back( YieldToOncomingObj ); // speed is opposite */
				ACC_Maneuvers.push_back( BehaviorElementary::AdaptiveCruiseObj(TargetState, objPred.host_dist_to_conflict_m, 0,0, objID) );
				ACC_Maneuver_Probabilities.push_back(objSA.pred_obj_probability[i]);
			}
		}

		else if ( ((relative_location==SA_AHEAD_LEFT_EDGE) && (ExpectedStaticObjLoc == SA_AHEAD_LEFT_EDGE) )  || \
				((relative_location==SA_AHEAD_RIGHT_EDGE) || (ExpectedStaticObjLoc == SA_AHEAD_RIGHT_EDGE) ) ){

			std::map<double,double> SlowDownFactorTable = { {0.0, 0.75} , {30.0, 0.75}, {50.0, 0.8}, {70.0, 0.9}, {90.0, 0.99} ,{120.0, 0.99}}; // As a function of distance from the object
			double dist_from_HV =  SA.roadside_dist_ahead();
			double SlowDownFactor = BU::LookUp1DMap(SlowDownFactorTable,dist_from_HV);

			auto roadside_obj_IDs = SA.HVData.hv_lanesituation.roadside_obj_IDs;
			if(!BehaviorUtils::find(roadside_obj_IDs,objID))
				continue;

			if(objPred.x_m < 0)
				continue;

			if(dist_from_HV>0){
				double DefaultspeedTarget= TargetState.step.posted_speed_lim_mps;
				double SpeedTarget = DefaultspeedTarget * SlowDownFactor;
				double TargetAccel = BU::targetAcceleration(SA_LOOK_AHEAD_HORIZON,SpeedTarget);
				BehaviorTertiary::Maneuver_Temporal  SlowDown_Maneuver = BehaviorTertiary::Maneuver_Temporal(TargetAccel,SpeedTarget,-999,\
						TargetAccel ,TempRules::RIGHT_OF_WAY);
				SlowDown_Maneuver.speedTargetDistanceActual = dist_from_HV;
				SlowDown_Maneuver.primary_target_object = objID;
				/*	if(BU::codeBlue(" objID ",objID,1)){
					cout<<" dist_from_HV "<<dist_from_HV<<" SlowDownFactor "<<SlowDownFactor<<" prob "<<objSA.pred_obj_probability[i]<<endl;
					cout<<" SlowDown_Maneuver "<<SlowDown_Maneuver<<endl;
				} */
				ACC_Maneuvers.push_back(SlowDown_Maneuver);
				ACC_Maneuver_Probabilities.push_back(objSA.pred_obj_probability[i]);
			}



		}


	}
	My_Maneuver_Temporal = My_Maneuver_Temporal + BehaviorTertiary::VNM_RationalizedAgent(ACC_Maneuvers,ACC_Maneuver_Probabilities);

	/*if(BU::codeGreen( _CF_ +" My_Maneuver_Temporal: ",My_Maneuver_Temporal,1)){
		//cout<<yellow_on<<"SA.clear_dist_ahead(HV_LANE_SITUATION) "<<SA.clear_dist_ahead(HV_LANE_SITUATION)<<" hypothesisIdx "<<hypothesisIdx<<endl;
		//cout<<" My_Maneuver_Temporal "<<My_Maneuver_Temporal<<endl;
	}*/


	return My_Maneuver_Temporal;
}

BehaviorTertiary::Maneuver_Temporal BehaviorElementary::AdaptiveCruise(Env::PolicyState &TargetState)
{


	double DefaultspeedTarget=TargetState.step.posted_speed_lim_mps;
	BehaviorTertiary::Maneuver_Temporal  My_Maneuver_Temporal = BehaviorTertiary::Maneuver_Temporal(-999,DefaultspeedTarget,SA_LOOK_AHEAD_HORIZON,\
			std::numeric_limits<double>::max(),TempRules::NO_RULES);
	BehaviorTertiary::Maneuver_Temporal Maneuver_Pure_ACC,Maneuver_Merge_ACC,Maneuver_Parallel_ACC,Maneuver_Cross_ACC;
	int LaneObjIdx,hypothesisIdx;
	ObjUidType ObjUID;

	std::map<ObjUidType, std::vector<BehaviorTertiary::Maneuver_Temporal> > All_Maneuver_ACC;
	std::map<ObjUidType, std::vector<double> > All_Maneuver_ACC_prob;


	if(SA.ahead_obj_id(HV_LANE_SITUATION)>0){
		ObjUID = SA.ahead_obj_id(HV_LANE_SITUATION,0,&LaneObjIdx,&hypothesisIdx);
		Maneuver_Pure_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead(HV_LANE_SITUATION),ObjUID,hypothesisIdx);
		My_Maneuver_Temporal =  Maneuver_Pure_ACC;
		//	All_Maneuver_ACC[ObjUID].push_back(Maneuver_Pure_ACC);
		//	All_Maneuver_ACC_prob[ObjUID].push_back(BU::ExtractParamFromObjSA(ObjUID,"pred")[hypothesisIdx]);
		/*	if(BehaviorUtils::codeGreen( _CF_ +" Printing the ACC Behaviors: "," ",1000)){
			cout<<" Maneuver_Pure_ACC "<<Maneuver_Pure_ACC<<endl;
			cout<<"SA.HVData.center_ahead_obj_id "<<SA.ahead_obj_id(HV_LANE_SITUATION)<<endl;
			//SA.PrintSituation();
		} */
	}



	if(SA.merge_obj_uid(HV_LANE_SITUATION)>0){
		LaneObjIdx = 0;
		do{
			ObjUID = SA.merge_obj_uid(HV_LANE_SITUATION,LaneObjIdx,&hypothesisIdx);
			if(ObjUID<=0)
				break;

			exlcm::objsituation_t objSA =  SA.objData[ObjUID];
			if((hypothesisIdx>=objSA.valid_pred_count)||(hypothesisIdx<0)||(objSA.valid_pred_count<=0)){
				BU::codeRed(_CF_ +" something is really wrong in ACC ... { hypothesisIdx ,valid_pred_count } = ",std::make_pair(hypothesisIdx,objSA.valid_pred_count),1);
				LaneObjIdx++;
				continue;
			}


			exlcm::objprediction_t objPred = objSA.pred_obj_SA[hypothesisIdx];
			bool Objexpected_to_yield_f = (objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY || objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT || \
					objPred.expected_to_yield_f == YIELD_DUE_TO_ARRIVAL_INFERIORITY );
			bool collision_possible = objPred.collision_possible;
			if (Objexpected_to_yield_f || !collision_possible){
			}
			else{
				Maneuver_Merge_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead_merge(HV_LANE_SITUATION),ObjUID,hypothesisIdx);
				All_Maneuver_ACC_prob[ObjUID].push_back(BU::ExtractParamFromObjSA(ObjUID,"pred")[hypothesisIdx]);
				All_Maneuver_ACC[ObjUID].push_back(Maneuver_Merge_ACC);
				//My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_Merge_ACC;
			}

			LaneObjIdx++;
		}while(true);
	}

	int parallel_obj_uid = SA.parallel_obj_uid(HV_LANE_SITUATION);
	if(parallel_obj_uid>0){
		LaneObjIdx = 0;
		do{
			ObjUID = SA.parallel_obj_uid(HV_LANE_SITUATION,LaneObjIdx,&hypothesisIdx);
			if(ObjUID<=0)
				break;

			exlcm::objsituation_t objSA =  SA.objData[ObjUID];
			if((hypothesisIdx>=objSA.valid_pred_count)||(hypothesisIdx<0)||(objSA.valid_pred_count<=0)){
				BU::codeRed(_CF_ +" something is really wrong in ACC ... { hypothesisIdx ,valid_pred_count } = ",std::make_pair(hypothesisIdx,objSA.valid_pred_count),1);
				LaneObjIdx++;
				continue;
			}



			exlcm::objprediction_t objPred = objSA.pred_obj_SA[hypothesisIdx];
			bool Objexpected_to_yield_f = (objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY || objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT || \
					objPred.expected_to_yield_f == YIELD_DUE_TO_ARRIVAL_INFERIORITY );
			bool collision_possible = objPred.collision_possible;
			if (Objexpected_to_yield_f || !collision_possible){
			}
			else{
				Maneuver_Parallel_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead_parallel(HV_LANE_SITUATION),ObjUID,hypothesisIdx);
				All_Maneuver_ACC[ObjUID].push_back(Maneuver_Parallel_ACC);
				All_Maneuver_ACC_prob[ObjUID].push_back(BU::ExtractParamFromObjSA(ObjUID,"pred")[hypothesisIdx]);
				//My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_Parallel_ACC;
			}

			LaneObjIdx++;
		}while(true);


	}




	if(SA.cross_obj_uid(HV_LANE_SITUATION)>0){
		LaneObjIdx = 0;
		do{
			ObjUID = SA.cross_obj_uid(HV_LANE_SITUATION,LaneObjIdx);
			if(ObjUID<=0)
				break;



			exlcm::objsituation_t objSA =  SA.objData[ObjUID];
			if((hypothesisIdx>=objSA.valid_pred_count)||(hypothesisIdx<0)||(objSA.valid_pred_count<=0)){
				BU::codeRed(_CF_ +" something is really wrong in ACC ... { hypothesisIdx ,valid_pred_count } = ",std::make_pair(hypothesisIdx,objSA.valid_pred_count),1);
				LaneObjIdx++;
				continue;
			}



			exlcm::objprediction_t objPred = objSA.pred_obj_SA[hypothesisIdx];
			bool Objexpected_to_yield_f = (objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY || objPred.expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT || \
					objPred.expected_to_yield_f == YIELD_DUE_TO_ARRIVAL_INFERIORITY );
			bool collision_possible = objPred.collision_possible;
			if (Objexpected_to_yield_f || !collision_possible){
			}
			else{
				Maneuver_Cross_ACC = BehaviorElementary::AdaptiveCruiseObj(TargetState,SA.clear_dist_ahead_cross(HV_LANE_SITUATION),0,0,ObjUID);
				All_Maneuver_ACC[ObjUID].push_back(Maneuver_Cross_ACC);
				All_Maneuver_ACC_prob[ObjUID].push_back(BU::ExtractParamFromObjSA(ObjUID,"pred")[hypothesisIdx]);
				//My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_Cross_ACC;
			}

			LaneObjIdx++;
		}while(true);
	}

	std::map<ObjUidType, BehaviorTertiary::Maneuver_Temporal> Maneuver_Yield_VNMs;

	for(auto & obj:All_Maneuver_ACC){
		Maneuver_Yield_VNMs[obj.first] = BehaviorTertiary::VNM_RationalizedAgent(All_Maneuver_ACC[obj.first],All_Maneuver_ACC_prob[obj.first]);
	}

	for( auto & obj: Maneuver_Yield_VNMs){
		My_Maneuver_Temporal = My_Maneuver_Temporal + Maneuver_Yield_VNMs[obj.first];
	}



	return  My_Maneuver_Temporal;

}



