#include "../PCH/pch.hpp"
#include "BehaviorTertiary.hpp"
typedef std::pair<std::string,int> AgentKeyType;
namespace AgentPool{

extern std::map<AgentKeyType, BehaviorTertiary::Behavior > AllActiveBehaviorAgents;
extern std::map<AgentKeyType, std::pair<int,Env::PolicyState> > AllActiveBehaviorAgentInfo;
extern std::map<AgentKeyType, BehaviorTertiary::Behavior > LTBehaviorAgents;
extern std::map<AgentKeyType, BehaviorTertiary::Maneuver_Temporal > AllActiveTemporalAgents;
extern std::map<AgentKeyType, std::pair<int,Env::PolicyState> > AllActiveTemporalAgentInfo;
extern std::map<AgentKeyType, BehaviorTertiary::Maneuver_Temporal > LTTemporalAgents;

bool SeekAgentFromPool(BehaviorTertiary::Behavior& ReturnBehavior,std::string name);
void StoreAgentInPool( BehaviorTertiary::Behavior& StoreBehavior,std::string name,bool IsThisaLTAgent = false);


bool SeekAgentFromPool(BehaviorTertiary::Maneuver_Temporal& ReturnBehavior,const Env::PolicyState& TargetState,std::string name);
void StoreAgentInPool( BehaviorTertiary::Maneuver_Temporal& StoreBehavior,const Env::PolicyState& TargetState,std::string name,bool IsThisaLTAgent = false);

void PrintAllBehaviorAgentsInPool();
void PrintAllTemporalAgentsInPool();

void EmptyLTAgentPool();
void EmptyAgentPool();
}


