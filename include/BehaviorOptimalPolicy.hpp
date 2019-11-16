#pragma once
#include "lcmprint.hpp"
#include "BehaviorCals.hpp"
#include "BehaviorStatic.hpp"
#include "BehaviorObject.hpp"
#include "BehaviorTertiary.hpp"

namespace Policy{
extern BehaviorTertiary::Behavior& CreateBehaviorFromPolicyDeque(BehaviorTertiary::Behavior& BehaviorDesired, PolicyDequeType &MyPolicyDQ, \
		int alternaterouteNeeded, int FronIdxInMyPolicyDQ,  int StartStep = 0, bool ActivePolicyFlag = false , bool AlternatePolicyFlag = true);

bool CreateNextStepBehavior(BehaviorTertiary::Behavior& BehaviorDesired, PolicyDequeType &MyPolicyDQ,int FronIdxInMyPolicyDQ,int alternaterouteNeeded, \
		int& step,double& EndOfStepDistanceFromHV, bool ActivePolicyFlag,bool AlternatePolicyFlag);

void ReEvaluateActivePolicyAndSeekOptimalPolicy(BehaviorTertiary::Behavior & BehaviorDesired);

void EvaluateAlternatePoliciesAndSeekOptimalPolicy(BehaviorTertiary::Behavior & BehaviorDesired);

void ClearPolicyMemory();

double GetPolicyTrafficJamTime(const BehaviorTertiary::Behavior & BehaviorDesired, int LANE_SITUATION, ObjUidType& ImpatienceObj  );


inline double CalculateCost(exlcm::policycosts_t& my_PolicyCost){
	my_PolicyCost.TotalCost = my_PolicyCost.BehaviorPolicyCost.PolicyCost+my_PolicyCost.TrafficEfficiencyCost+my_PolicyCost.PolicyTransitionCost+my_PolicyCost.routeCost;
	return my_PolicyCost.TotalCost;
}



class DetailedPolicy{

	std::map<int,double> POLICY_TRANSITION_COSTS = { {HV_LANE_SITUATION,0 }, {STRIAGHT_LANE_SITUATION,0.75 }, {LEFT_ADJACENT_LANE_SITUATION,LEFT_ADJACENT_LANE_POLICY_SWITCH_COST}, \
		{RIGHT_ADJACENT_LANE_SITUATION,RIGHT_ADJACENT_LANE_POLICY_SWITCH_COST}, {LEFT_OPPOSING_LANE_SITUATION,LEFT_OPPOSING_LANE_POLICY_SWITCH_COST},\
		{RIGHT_OPPOSING_LANE_SITUATION,RIGHT_OPPOSING_LANE_POLICY_SWITCH_COST} };


public:
	reroute_type_enum alternaterouteNeeded;
	lane_situation_enum rootLane;
	double TrafficJamTime=0;
	ObjUidType TrafficJamObjID = -1;
	BehaviorTertiary::Behavior PolicyOptimalBehavior;
	Policy::InputPolicyType Policy;
	PolicyDequeType PolicyDQ;
	std::map<int,double> SegGrpOriginDAR;
	int Front=-1;
	exlcm::policycosts_t Cost;
	//double DTAR=-1;
	void CalculateCost();
	DetailedPolicy(const int route,const int lane,int FrontIdx, const BehaviorTertiary::Behavior& MyBehavior, const Policy::InputPolicyType& MyPolicy,const PolicyDequeType& MyPolicyDQ);
	DetailedPolicy(){};

};

extern Policy::DetailedPolicy  SearchAlternatePolicy(const Env::PolicyState& TargetState, int alternaterouteNeeded);


bool SoughtAndFoundAlternatePolicy(BehaviorTertiary::Behavior & ,std::map<int,Policy::DetailedPolicy>& ,int alternaterouteNeeded,int LanePolicyKey, bool LaneSAExists );

bool AlternatePolicyBehavior(const Env::PolicyState& TargetState, int alternaterouteNeeded, int rootLane, std::map<int,Policy::DetailedPolicy>& AllCandidatePolicies);

void PrintBehaviorPolicyCostTree(std::ostream& os,const BehaviorTertiary::BehaviorPolicyCostTree *root,int indent=80);

std::ostream& operator<<(std::ostream& os,const Policy::DetailedPolicy& );
std::ostream& operator<<(std::ostream& os,const std::map<int,Policy::DetailedPolicy>& );
std::ostream& operator<<(std::ostream& os, const BehaviorTertiary::BehaviorPolicyCostTree *root);

extern DetailedPolicy ActiveDetailedPolicy,BackEndDetailedPolicy;
extern std::map<int,Policy::DetailedPolicy> AllCandidatePolicies;
}



