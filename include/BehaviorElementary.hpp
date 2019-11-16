#pragma once

#include "../PCH/pch.hpp"

#include "BehaviorUtils.hpp"
#include "behaviorHorizon.hpp"
#include "shared_enums.h"
#include "Env.hpp"
#include "lcmprint.hpp"



namespace BehaviorTertiary{
class Maneuver_Temporal;
class Maneuver_Spatial;
class Behavior;
}

namespace BehaviorElementary
{
BehaviorTertiary::Maneuver_Temporal AdaptiveCruise(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal AdaptiveCruiseObj(Env::PolicyState &TargetState,double , ObjUidType, int HypIdx = 0 );
BehaviorTertiary::Maneuver_Temporal AdaptiveCruiseObj(Env::PolicyState &TargetState,double , double , double, ObjUidType objID = -1);
BehaviorTertiary::Maneuver_Temporal SharedLaneACCObj( Env::PolicyState TargetState,ObjUidType objID, int ExpectedStaticObjLoc=-1);

extern BehaviorTertiary::Behavior PlannedlaneChange(Env::PolicyState &TargetState);
extern BehaviorTertiary::Behavior turn(Env::PolicyState &TargetState,double TurSpeedTarget = -1);
extern BehaviorTertiary::Behavior StaticlaneKeeping(Env::PolicyState &TargetState,double SpdFactor=1);
extern BehaviorTertiary::Behavior SimplelaneKeepBehavior(Env::PolicyState &TargetState);
BehaviorTertiary::Behavior laneKeepBehavior(Env::PolicyState &TargetState);
BehaviorTertiary::Behavior AdjacentlaneKeepBehavior(Env::PolicyState &,int,int );
extern BehaviorTertiary::Behavior  GetMaxSpeedBehavior(const Env::PolicyState &TargetState, double SpdFactor = 1);
extern BehaviorTertiary::Behavior ExitBehavior(Env::PolicyState &TargetState);
BehaviorTertiary::Maneuver_Temporal CollisionImminentBehavior();

BehaviorTertiary::Behavior& CreateAndAddDefaultManeuvers(BehaviorTertiary::Behavior& BehaviorDesired);
};


