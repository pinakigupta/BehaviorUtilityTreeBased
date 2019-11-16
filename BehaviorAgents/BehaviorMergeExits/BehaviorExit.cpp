#include "../../PCH/pch.hpp"

#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"

#include "../../include/BehaviorObject.hpp"
#include "../../include/Env.hpp"
#include "../../include/shared_enums.h"
#include "../../include/AgentPool.hpp"


namespace BU=BehaviorUtils;
struct  vehicle;



BehaviorTertiary::Behavior BehaviorElementary::ExitBehavior(Env::PolicyState &TargetState){
	// At this point relying on the fact that the hypothesis will localize with some probability if it is uncertain which branch to localize, on both branches. Right now we would respond with a 100% probability
	// ACC assumpotion.
	BehaviorTertiary::Behavior My_Behavior = BehaviorElementary::laneKeepBehavior(TargetState);

	if(AgentPool::SeekAgentFromPool(My_Behavior,__FUNCTION__))
		return My_Behavior;

	My_Behavior.TemporalRule = EXIT;
	My_Behavior.CalculateBehavior();

	const double SLOWDOWN_FACTOR = 0.95;
	My_Behavior.speedTarget = My_Behavior.speedTarget*SLOWDOWN_FACTOR;
	auto timenow = std::chrono::high_resolution_clock::now();



	auto ParentLaneSegUID = TargetState.CurrentGrpLanes.back();
	auto ParentLaneSeg = BehaviorUtils::getLaneSeg(ParentLaneSegUID);



	if(TargetState.step.direction==TURN_LEFT){
		My_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_LEFT;
		My_Behavior.turn_indicator_target_distance = TargetState.DAR_m-vehicle.DTAR;
	}
	else if(TargetState.step.direction==TURN_RIGHT){
		My_Behavior.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_RIGHT;
		My_Behavior.turn_indicator_target_distance = TargetState.DAR_m-vehicle.DTAR;
	}

	AgentPool::StoreAgentInPool(My_Behavior,__FUNCTION__);

	return My_Behavior;
}
