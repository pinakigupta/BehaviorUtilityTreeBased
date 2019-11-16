#include "../include/AgentPool.hpp"
#include "../include/BehaviorUtils.hpp"

size_t HashAgentKey(const Env::PolicyState& MyPolicy){
	size_t seed = 0;
	for(auto& lane:MyPolicy.CurrentGrpLanes)
		Hash_combine(seed,(int64_t)lane);
	for(auto& lane:MyPolicy.TargetGrpLanes)
		Hash_combine(seed,(int64_t)lane);
	return seed;
}
std::map<AgentKeyType, BehaviorTertiary::Behavior > AgentPool::LTBehaviorAgents;
std::map<AgentKeyType, BehaviorTertiary::Maneuver_Temporal > AgentPool::LTTemporalAgents;
std::map<AgentKeyType ,BehaviorTertiary::Behavior > AgentPool::AllActiveBehaviorAgents;
std::map<AgentKeyType, std::pair<int,Env::PolicyState> > AgentPool::AllActiveBehaviorAgentInfo; // Key -> {AgentName ,HashAgentKey}   value ->(Agent use count}
bool AgentPool::SeekAgentFromPool(BehaviorTertiary::Behavior& ReturnBehavior, std::string name){
	std::pair<std::string,int> ThisPair = std::make_pair(name,HashAgentKey(ReturnBehavior.TargetState));
	stringstream os;
	os<<ThisPair;
	std::string ThisPairStr = os.str();
	AgentPool::AllActiveBehaviorAgentInfo[ThisPair].second=ReturnBehavior.TargetState;
	if(BehaviorUtils::find(AgentPool::AllActiveBehaviorAgents,ThisPair)){
		ReturnBehavior =  AgentPool::AllActiveBehaviorAgents[ThisPair];
		/*	if(BU::codeGreen(_CF_ +"Behavior: "+ThisPairStr+" @(s)",ExecTime,1)){
			lcmprint::PrintEnum(cout,(behavior_type)ReturnBehavior.behaviorType);
		} */
		AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first++;
		return true;
	}
	else{
		AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first=0;
		if(BehaviorUtils::find(AgentPool::LTBehaviorAgents,ThisPair)){
			ReturnBehavior =  AgentPool::LTBehaviorAgents[ThisPair];
			AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first++;
			return true;
		}
	}
	return false;
}

void AgentPool::StoreAgentInPool( BehaviorTertiary::Behavior& StoreBehavior,std::string name,bool IsThisaLTAgent ){
	std::pair<std::string,int> ThisPair = std::make_pair(name,HashAgentKey(StoreBehavior.TargetState));
	stringstream os;
	os<<ThisPair;
	std::string ThisPairStr = os.str();
	/*	BU::codeYellow(_CF_ + "StoreAgentInPool:ExecTime " + ThisPairStr,ExecTime,1);
	BU::codeYellow(_CF_ + "StoreAgentInPool:[name,StepIdx] " + ThisPairStr,ThisPair,1);
	BU::codeYellow(_CF_ + "StoreAgentInPool:AllActiveAgents[name,StepIdx] " + ThisPairStr,StoreBehavior.behaviorType,1); */
	//if(BehaviorUtils::find(AgentPool::AllActiveBehaviorAgents,ThisPair))
	//	exit(1);
	AgentPool::AllActiveBehaviorAgents[ThisPair] = StoreBehavior;
	if(IsThisaLTAgent)
		AgentPool::LTBehaviorAgents[ThisPair] = StoreBehavior;
}

std::map<AgentKeyType ,BehaviorTertiary::Maneuver_Temporal > AgentPool::AllActiveTemporalAgents;
std::map<AgentKeyType, std::pair<int,Env::PolicyState> > AgentPool::AllActiveTemporalAgentInfo;
bool AgentPool::SeekAgentFromPool(BehaviorTertiary::Maneuver_Temporal& ReturnManeuverTemporal,const Env::PolicyState& TargetState, std::string name){
	std::pair<std::string,int> ThisPair = std::make_pair(name,HashAgentKey(TargetState));
	stringstream os;
	os<<ThisPair;
	std::string ThisPairStr = os.str();
	if(BehaviorUtils::find(AgentPool::AllActiveTemporalAgents,ThisPair)){ // Found in current Temporal pool
		ReturnManeuverTemporal =  AgentPool::AllActiveTemporalAgents[ThisPair];
		/*	if(BU::codeGreen(_CF_ +"Temporal Maneuver: "+ThisPairStr+" @(s)",ExecTime,1)){
			BehaviorTertiary::Print(cout,(TempRules)ReturnManeuverTemporal.TemporalRule);
		}*/

		AgentPool::AllActiveTemporalAgentInfo[ThisPair].first++;
		return true;
	}
	else if(BehaviorUtils::find(AgentPool::AllActiveBehaviorAgents,ThisPair)){ // Found in current  Behavior pool
		ReturnManeuverTemporal =  AgentPool::AllActiveBehaviorAgents[ThisPair];
		AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first++;
		return true;
	}
	else{
		AgentPool::AllActiveTemporalAgentInfo[ThisPair].first=0;
		if(BehaviorUtils::find(AgentPool::LTTemporalAgents,ThisPair)){ // Found in LT Temporal pool
			ReturnManeuverTemporal =  AgentPool::LTTemporalAgents[ThisPair];
			AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first++;
			return true;
		}
		else if(BehaviorUtils::find(AgentPool::LTBehaviorAgents,ThisPair)){ // Found in LT Behavior pool
			ReturnManeuverTemporal =  AgentPool::LTBehaviorAgents[ThisPair];
			AgentPool::AllActiveBehaviorAgentInfo[ThisPair].first++;
			return true;
		}
	}
	AgentPool::AllActiveTemporalAgentInfo[ThisPair].second=TargetState;
	return false;
}

void AgentPool::StoreAgentInPool( BehaviorTertiary::Maneuver_Temporal& StoreTemporalManuever,const Env::PolicyState& TargetState,std::string name,bool IsThisaLTAgent){
	std::pair<std::string,int> ThisPair = std::make_pair(name,HashAgentKey(TargetState));
	stringstream os;
	os<<ThisPair;
	std::string ThisPairStr = os.str();
	/*	BU::codeYellow(_CF_ + "StoreAgentInPool:ExecTime " + ThisPairStr,ExecTime,1);
	BU::codeYellow(_CF_ + "StoreAgentInPool:[name,StepIdx] " + ThisPairStr,ThisPair,1);
	BU::codeYellow(_CF_ + "StoreAgentInPool:AllActiveAgents[name,StepIdx] " + ThisPairStr,StoreBehavior.behaviorType,1); */
	//if(BehaviorUtils::find(AgentPool::AllActiveTemporalAgents,ThisPair))
	//	exit(1);
	AgentPool::AllActiveTemporalAgents[ThisPair] = StoreTemporalManuever;
	if(IsThisaLTAgent)
		AgentPool::LTTemporalAgents[ThisPair] = StoreTemporalManuever;
}


void AgentPool::EmptyAgentPool(){
	AgentPool::AllActiveBehaviorAgents.clear();
	AgentPool::AllActiveTemporalAgents.clear();
	AgentPool::AllActiveTemporalAgentInfo.clear();
	AgentPool::AllActiveTemporalAgentInfo.clear();
	AgentPool::AllActiveBehaviorAgents.clear();
}

void AgentPool::PrintAllBehaviorAgentsInPool(){
	if(AgentPool::AllActiveBehaviorAgentInfo.empty()){
		cout<<blue_on<<" Behavior Agent Pool Empty"<<color_off<<endl;
		return;
	}
	for(auto &agent:AgentPool::AllActiveBehaviorAgentInfo){
		std::vector<LaneSegUidType> CurrentGrpLanes = agent.second.second.CurrentGrpLanes;
		std::vector<LaneSegUidType> TargetGrpLanes = agent.second.second.TargetGrpLanes;
		auto step = agent.second.second.step;
		cout<<blue_on<<" Agent Details = "<<agent.first<<" Agent count = "<<agent.second.first<<\
				" CurrentLanes(Grp) "<<CurrentGrpLanes<<"("<<step.current_segment_group<<")"<<\
				" TgtLanes(Grp) "<<TargetGrpLanes<<"("<<step.target_segment_group<<")"<<color_off<<endl;
	}
}

void AgentPool::PrintAllTemporalAgentsInPool(){
	if(AgentPool::AllActiveTemporalAgentInfo.empty()){
		cout<<green_on<<" Temporal Agent Pool Empty"<<color_off<<endl;
		return;
	}
	for(auto &agent:AgentPool::AllActiveTemporalAgentInfo){
		std::vector<LaneSegUidType> CurrentGrpLanes = agent.second.second.CurrentGrpLanes;
		std::vector<LaneSegUidType> TargetGrpLanes = agent.second.second.TargetGrpLanes;
		auto step = agent.second.second.step;
		cout<<green_on<<" Agent Details = "<<agent.first<<" Agent count = "<<agent.second.first<<\
				" CurrentLanes(Grp) "<<CurrentGrpLanes<<"("<<step.current_segment_group<<")"<<\
				" TgtLanes(Grp) "<<TargetGrpLanes<<"("<<step.target_segment_group<<")"<<color_off<<endl;
	}
}
