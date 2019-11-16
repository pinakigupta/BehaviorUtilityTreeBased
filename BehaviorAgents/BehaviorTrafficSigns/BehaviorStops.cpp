#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/shared_enums.h"
#include "../../include/AgentPool.hpp"

struct  vehicle;

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::holdStillUntillIntersectionIsClear() // Rename this function to HoldStill( )
{
	//vehicle.behaviorType=YIELD_TO_CROSS_TRAFFIC_BEHAVIOR;
	BehaviorUtils::codeBlue( _CF_ +"INFO: Holding HV still until Everything is clear");
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal(0,0,0,-999,YIELD_TO_CROSS_TRAFFIC);
	//My_Maneuver_Temporal.BehaviorAccelRequestType = SUPER_AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR;
	return My_Maneuver_Temporal;

}

int manual_override=-1;

BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::stopAndStandBy(Env::PolicyState &TargetState, bool StopZone){
	double env_s=TargetState.DAR_m-vehicle.DTAR-CONTROLLER_OVERSHOOT_LOWERBOUND_AT_SIGNALS_m;
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	if((env_s<CONTROLLER_OVERSHOOT_LOWERBOUND_AT_STOPS_m)||(env_s<5 &&StopZone)){
		My_Maneuver_Temporal = BehaviorSecondary::holdStillUntillIntersectionIsClear();
	}
	else{
		My_Maneuver_Temporal = BehaviorSecondary::stopAtStopBar(TargetState);
	}


	My_Maneuver_Temporal.speedTargetDistanceActual = fmax(0,TargetState.DAR_m-vehicle.DTAR);
	return My_Maneuver_Temporal;
}




BehaviorTertiary::Maneuver_Temporal BehaviorSecondary::stopAtStopBar(Env::PolicyState &TargetState)
{

	static float env_s;
	auto Myrule = TargetState.step.step_rule;
	env_s=TargetState.DAR_m-vehicle.DTAR-CONTROLLER_OVERSHOOT_LOWERBOUND_AT_STOPS_m;
	float env_v=0.0;
	static map <int,bool> StoppedAtStopZone ;
	bool stoppedAtTarget= false;
	int StopZoneIdx= TargetState.step.step_index_n;
	BehaviorTertiary::Maneuver_Temporal My_Maneuver_Temporal;
	//if(AgentPool::SeekAgentFromPool(My_Maneuver_Temporal,TargetState,__FUNCTION__))
	//	return My_Maneuver_Temporal;

	My_Maneuver_Temporal.speedTargetDistanceActual = fmax(0,TargetState.DAR_m-vehicle.DTAR);
	My_Maneuver_Temporal= BehaviorElementary::laneKeepBehavior(TargetState);
	//if (behaviorinput::userin.confirmation_status == 1){
	//	TargetState.UserConfirmReceived=true;
	//}



	if(TargetState.DAR_m-vehicle.DTAR<0){
		//cout<<" TargetState.DAR_m-vehicle.DTAR "<<TargetState.DAR_m-vehicle.DTAR<<endl;
		return My_Maneuver_Temporal;
	}


	if(env_s<DIST_THRESHOLD_FOR_COMPLETE_STOP){
		if (StoppedAtStopZone.find(StopZoneIdx) != StoppedAtStopZone.end())
			stoppedAtTarget = StoppedAtStopZone[StopZoneIdx];

		if((vehicle.curSpeed_mps>VEHICLE_VELOCITY_THRESHOLD_FOR_COMPLETE_STOP_mps)&&!stoppedAtTarget){

			manual_override = 1;
			BehaviorTertiary::Maneuver_Temporal Maneuver_STOP(0,0,0,-999,STOP);
			Maneuver_STOP.speedTargetDistanceActual = TargetState.DAR_m-vehicle.DTAR;
			//Maneuver_STOP.BehaviorAccelRequestType = SUPER_AGGRESSIVE_ACCEL_REQUEST_BEHAVIOR;
			AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);
			return Maneuver_STOP;
			//	}
		}
		else if(vehicle.curSpeed_mps<VEHICLE_VELOCITY_THRESHOLD_FOR_COMPLETE_STOP_mps){
			StoppedAtStopZone[StopZoneIdx] = true;
		}


	}
	else{
		StoppedAtStopZone[StopZoneIdx] = false;
		manual_override = -1;
	}

	bool eval_b  = (env_s<DIST_THRSH_FOR_FLOW_RESTRICTION_CONSIDERATION)&&(!StoppedAtStopZone[StopZoneIdx]);
	BehaviorTertiary::Maneuver_Temporal Maneuver_STOP;

	if(eval_b)
	{
		if(env_s > DIST_THRSH_FOR_FLOW_RESTRICTION_COST_INFLUENCE){
			Maneuver_STOP = BehaviorTertiary::Maneuver_Temporal(-999,env_v,fmax(0,env_s),0,STOP);//Don't really want the stop behavior to win if there are competing behaviors/maneuvers
			My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_STOP;
		}
		else{
			auto StopAccel = BehaviorUtils::targetAcceleration(env_s,vehicle.curSpeed_mps,env_v);
			Maneuver_STOP = BehaviorTertiary::Maneuver_Temporal(StopAccel,env_v,fmax(0,env_s),StopAccel,STOP);
			My_Maneuver_Temporal =  My_Maneuver_Temporal + Maneuver_STOP;
		}
	}


	AgentPool::StoreAgentInPool(My_Maneuver_Temporal,TargetState,__FUNCTION__);




	return My_Maneuver_Temporal;
}






