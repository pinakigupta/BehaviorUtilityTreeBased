#pragma once


#include "../PCH/pch.hpp"


#include "BehaviorUtils.hpp"
#include "behaviorHorizon.hpp"
#include "BehaviorElementary.hpp"
#include "shared_enums.h"


namespace BehaviorSecondary
{
typedef enum TrafficLightType{ UNKNOWN_TYPE, RED_TYPE, GREEN_TYPE, YELLOW_TYPE,FLASHING_RED_TYPE, FLASHING_YELLOW_TYPE, FLASHING_GREEN_TYPE, PEDESTRIAN_TYPE}eTrafficLightType;

BehaviorTertiary::Maneuver_Temporal YieldToPedestrian(Env::PolicyState &closestState,std::pair<ObjUidType,exlcm::objprediction_t> &SAobj, bool InParkingAisle = false);
BehaviorTertiary::Maneuver_Temporal YieldToCrossTrafficObject(Env::PolicyState &TargetState,std::pair<ObjUidType,exlcm::objprediction_t> &SAobj,\
		int  HvInMajor= LN_YIELD_DUE_TO_STATIC_INFERIORITY, bool NOYieldToDistCrossLanes =false);
BehaviorTertiary::Maneuver_Temporal CrossWalkBehavior(Env::PolicyState &TargetState,double speedTargetDistance);
BehaviorTertiary::Maneuver_Temporal CrossWalkBehavior(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal NonCrossWalkPedestrianBehavior(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal holdStillUntillIntersectionIsClear();
BehaviorTertiary::Maneuver_Temporal stopAtStopBar(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal stopAndStandBy(Env::PolicyState &TargetState,bool StopZone = false);
BehaviorTertiary::Maneuver_Temporal HandleObservedTrafficSigns(Env::PolicyState &TargetState);

BehaviorTertiary::Maneuver_Temporal StopAtTrafficLight(Env::PolicyState &TargetState, bool StopZone = false);
BehaviorTertiary::Maneuver_Temporal YieldAtTrafficLight(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal HandleTrafficSignals(Env::PolicyState &TargetState,bool StopZone = false, bool UnmappedTrafficLight = false);

extern std::ostream& PrintEnum(std::ostream &os, const eTrafficLightType  );
};


