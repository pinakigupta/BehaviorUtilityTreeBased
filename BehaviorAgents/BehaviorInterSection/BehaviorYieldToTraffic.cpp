#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"

#include "../../include/Env.hpp"
#include "../../include/shared_enums.h"


struct  vehicle;



BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::YieldToCrossTrafficObject(Env::PolicyState &TargetState,std::pair<ObjUidType,exlcm::objprediction_t> &obj,int HvLaneYieldStatus, bool NOYieldToDistCrossLanes){

	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal_LK= BehaviorElementary::StaticlaneKeeping(TargetState);
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal = My_Maneuver_Temporal_LK;
	bool intersectionClear = true;
	const double timegap = 1e7;
	ObjUidType DEBUG_UID = 8579;

	std::string objstr = std::to_string(obj.first);
	objstr = "objID:<"+objstr+">";
	exlcm::objprediction_t ObjPred = obj.second;

	int rel_loc = obj.second.relative_location;
	int pred_rel_loc = obj.second.predicted_relative_location;
	int expected_to_yield_f = ObjPred.expected_to_yield_f;
	bool collision_possible = ObjPred.collision_possible;
	bool ShouldYieldToObj = true;
	bool ObjIsYieldingToHV = (ObjPred.behavior_type==YIELD_TO_CROSS_TRAFFIC_BEHAVIOR)&&(ObjPred.primary_target_object_id==0);

	bool RvHasStaticYieldPriority = (expected_to_yield_f == DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY_RV_AT_INT)||(expected_to_yield_f == DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY);
	int Stationary_Status = ObjPred.Stationary_Status;

	bool RvHasYieldPriority = (RvHasStaticYieldPriority)||(expected_to_yield_f == DO_NOT_YIELD_DUE_TO_ARRIVAL_SUPERIORITY)\
			||(expected_to_yield_f == DO_NOT_YIELD_DUE_TO_PERCEIVED_ARRIVAL_SUPERIORITY)||(expected_to_yield_f == DO_NOT_YIELD_IN_VIOLATION_OF_TRAFFIC_RULES);

	if(ObjIsYieldingToHV&&RvHasStaticYieldPriority)
		return My_Maneuver_Temporal_LK;

	if((rel_loc==SA_NOT_NEAR)||(Stationary_Status==STATIONARY_STATUS_PARKED_ROADSIDE)||(Stationary_Status==STATIONARY_STATUS_PARKED_PARKING_AISLE)){
		ShouldYieldToObj = false;
		//return My_Maneuver_Temporal;
	}


	InterSectionUidType HvInterSectionID = Env::_ClosestInterSectionInfo.first;
	bool IsObjAtHvIntersection = (ObjPred.intersection_uid ==HvInterSectionID ) && (HvInterSectionID>0);
	bool IsObjCrossingHVAtHVIntersection = (ObjPred.intersecting_intersection_id == HvInterSectionID ) && (HvInterSectionID>0); // Object may not be @ Hv intersection now but
	bool IsObjAtOrPredictedToBeAtHvIntersection = false;

	bool HvHasYieldPriority = (expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY || expected_to_yield_f == YIELD_DUE_TO_STATIC_INFERIORITY_HV_AT_INT || \
			expected_to_yield_f == YIELD_DUE_TO_ARRIVAL_INFERIORITY );



	if( IsObjAtHvIntersection || IsObjCrossingHVAtHVIntersection) {
		IsObjAtOrPredictedToBeAtHvIntersection = true;
	}
	else{
		BehaviorUtils::codeRed( _CF_  + "Different intersections !!! {HV intersection_uid, ObjPred.intersection_uid, ObjPred.intersecting_intersection_id } " + objstr , \
				std::make_tuple(HvInterSectionID, ObjPred.intersection_uid, ObjPred.intersecting_intersection_id),timegap);
		//BehaviorUtils::DebugObjExit(obj.first,DEBUG_UID);
		//return My_Maneuver_Temporal;
		ShouldYieldToObj = false;
	}

	if(HvLaneYieldStatus==LN_DO_NOT_YIELD_DUE_TO_STATIC_SUPERIORITY){
		//if(!(rel_loc==SA_INTERSECTING_FROM_LEFT || rel_loc==SA_INTERSECTING_FROM_RIGHT)) // Merging excluded
		//	return My_Maneuver_Temporal;





		if(!RvHasYieldPriority){
			//BehaviorUtils::DebugObjExit(obj.first,DEBUG_UID);
			//return My_Maneuver_Temporal;
			ShouldYieldToObj = false;
		}

	}

	bool IntersectingOrMerging = (rel_loc==SA_INTERSECTING_FROM_LEFT || rel_loc==SA_INTERSECTING_FROM_RIGHT || rel_loc== SA_MERGING_FROM_LEFT || rel_loc== SA_MERGING_FROM_RIGHT);
	bool IntersectingOrMergingPred = (pred_rel_loc==SA_INTERSECTING_FROM_LEFT || pred_rel_loc==SA_INTERSECTING_FROM_RIGHT || pred_rel_loc== SA_MERGING_FROM_LEFT || pred_rel_loc== SA_MERGING_FROM_RIGHT);
	bool IntersectingOrMergingFromAhead = (rel_loc==SA_INTERSECTING_FROM_LEFT_AHEAD || rel_loc==SA_INTERSECTING_FROM_RIGHT_AHEAD || rel_loc== SA_MERGING_FROM_LEFT_AHEAD || rel_loc== SA_MERGING_FROM_RIGHT_AHEAD);
	if( IntersectingOrMerging || (IntersectingOrMergingPred&&IsObjAtOrPredictedToBeAtHvIntersection)|| (IntersectingOrMergingFromAhead && !NOYieldToDistCrossLanes)	 )
	{
		BehaviorUtils::codeRed( _CF_  + "------------------YieldToCrossTrafficObject INFO  BEGIN " + objstr + "   --------------------------"," ",timegap);
		BehaviorUtils::codeBlue( _CF_ +"intersecting_obj Lane UID  " + objstr,ObjPred.lane_segment_uid ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"intersecting_obj Lane Station  "+objstr,ObjPred.lane_station_m ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"time_to_intersection_sec  "+objstr,ObjPred.time_to_intersection_sec ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"time_to_collision  "+objstr,ObjPred.time_to_collision_sec ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"HV intersection_uid  "+objstr, behaviorinput::hvLoc.matched_intersection_uid ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"HV Lane UID  "+objstr, behaviorinput::hvLoc.matched_laneseg_uid ,timegap);
		BehaviorUtils::codeBlue( _CF_ +"HV Lane Station "+objstr, behaviorinput::hvLoc.matched_lane_station_m ,timegap);



		if (BehaviorUtils::codeBlue( _CF_ +"intersecting _obj relative location  "," " ,timegap))
			cout<<blue_on<<static_cast<eSAObjLocation>(rel_loc)<<color_off<<endl;



		if((fabs(vehicle.curSpeed_mps)<2.0)&&IsObjAtOrPredictedToBeAtHvIntersection){ // This typically means we(HV) are already yielding/ stopping for other reasons and collision prediction wouldn't work as spd = 0
			if(HvLaneYieldStatus==LN_YIELD_DUE_TO_STATIC_INFERIORITY){
				collision_possible = true;
				cout<<"HvLaneYieldStatus "<<HvLaneYieldStatus<<endl;
			}
		}



		if (HvHasYieldPriority){
			if(!collision_possible){
				BehaviorUtils::codeRed( _CF_  + "All condition for intersection was being met. But ignoring as object was expected to yield to HV (collision not likely).  " + objstr, " ",timegap);
				BehaviorUtils::codeRed( _CF_  + " expected_to_yield_f " + objstr, ObjPred.expected_to_yield_f,timegap);
				//BehaviorUtils::DebugObjExit(obj.first,DEBUG_UID);
				//return My_Maneuver_Temporal;
				ShouldYieldToObj = false;
			}
		}

		BehaviorUtils::codeRed( _CF_  + "------------------YieldToCrossTrafficObject INFO  END " + objstr + "--------------------------"," ",timegap);
		My_Maneuver_Temporal = BehaviorSecondary::holdStillUntillIntersectionIsClear();
		My_Maneuver_Temporal.primary_target_object = obj.first;
	}
	else{
		BehaviorUtils::codeGreen( _CF_ +"Non intersecting_obj uid ",obj.first ,timegap);
		if (BehaviorUtils::codeGreen( _CF_ +"Non intersecting _obj relative location  "," " ,timegap))
			cout<<green_on<<static_cast<eSAObjLocation>(rel_loc)<<color_off<<endl;
	}


	if(!ShouldYieldToObj){
		return My_Maneuver_Temporal_LK;
	}

	return My_Maneuver_Temporal;
}


