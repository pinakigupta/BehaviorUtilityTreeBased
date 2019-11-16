#include "../include/BehaviorUtils.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"
#include "../include/BehaviorGapUtils.hpp"
#include "../include/lcmprint.hpp"
#include "../include/AgentPool.hpp"

extern std::map<int, PolicyDequeType  > Prev_ALL_AlternatePolicyDQ;
extern std::map<int, Policy::InputPolicyType  > Prev_ALL_AlternatePolicies;
Policy::DetailedPolicy Policy::ActiveDetailedPolicy,Policy::BackEndDetailedPolicy;
std::map<int,Policy::DetailedPolicy> Policy::AllCandidatePolicies;

void AgentPool::EmptyLTAgentPool(){

	for (auto it = AgentPool::AllActiveBehaviorAgents.cbegin(); it != AgentPool::AllActiveBehaviorAgents.cend() /* not hoisted */; /* no increment */)
	{
		//  if(BehaviorUtils::find(Policy::ActiveDetailedPolicy.Policy.routeplan.segment_uid,it->second.TargetState.CurrentGrpLanes))
		if(it->second.TargetState.DAR_m>vehicle.DTAR)
		{
			++it;
		}
		else
		{
			AgentPool::AllActiveBehaviorAgents.erase(it++);    // or "it = m.erase(it)" since C++11
		}
	}
}



Policy::DetailedPolicy::DetailedPolicy(const int route,const int lane,int FrontIdx,const BehaviorTertiary::Behavior& MyBehavior, const Policy::InputPolicyType& MyPolicy,const PolicyDequeType& MyPolicyDQ):\
		alternaterouteNeeded(route),rootLane(lane),Front(FrontIdx),PolicyOptimalBehavior(MyBehavior),Policy(MyPolicy),PolicyDQ(MyPolicyDQ){
	this->CalculateCost();

}


void Policy::ClearPolicyMemory(){
	Policy::AllCandidatePolicies.clear();
	Prev_ALL_AlternatePolicyDQ.clear();Prev_ALL_AlternatePolicies.clear();
	AgentPool::EmptyLTAgentPool();
	/*pthread_mutex_lock(&Mutex::_alternateplan_mutex);
	behaviorinput::AllaltpathPlan_IN.clear();
	behaviorinput::AllaltrouteSegmentList_IN.clear();
	pthread_mutex_unlock(&Mutex::_alternateplan_mutex);*/
}

void Policy::EvaluateAlternatePoliciesAndSeekOptimalPolicy(BehaviorTertiary::Behavior & BehaviorDesired)
{

	const double ShowTmThrsh = 1.0;
	vehicle.PredAccel_mpss = vehicle.curAccel_mpss;
	static auto LastExectTime = chrono::high_resolution_clock::now();


	int StepIdx = BehaviorDesired.TargetState.step.step_index_n;
	std::string CurrGrpStr = "<StepIdx:"+std::to_string(StepIdx)+">";

	bool HVlaneExists = SA.LaneSituationExists(HV_LANE_SITUATION);
	bool leftlaneExists = SA.LaneSituationExists(LEFT_ADJACENT_LANE_SITUATION);
	bool rightlaneExists = SA.LaneSituationExists(RIGHT_ADJACENT_LANE_SITUATION);
	bool leftOpposinglaneExists = SA.LaneSituationExists(LEFT_OPPOSING_LANE_SITUATION);
	bool rightOpposinglaneExists = SA.LaneSituationExists(RIGHT_OPPOSING_LANE_SITUATION);
	bool StraightlaneExists = SA.LaneSituationExists(STRIAGHT_LANE_SITUATION);

	int LanePolicyKey;


	/*	if(BehaviorDiagNostics::StateDequeMismatch==SPATIAL_STATE_MISMATCH){
		BU::codeBlue(_CF_ + "BehaviorDiagNostics::StateDequeMismatch !!! ",BehaviorDiagNostics::StateDequeMismatch,0.1);
		BehaviorDesired.alternaterouteNeeded.insert(STRAIGHT_ROUTE);
		BehaviorDesired.reRouteNeeded=STRAIGHT_ROUTE;
		return;
	} */

	Policy::ActiveDetailedPolicy.CalculateCost();

	int MyTask = BehaviorDesired.TargetState.step.step_task;
	int MyDirection = BehaviorDesired.TargetState.step.direction;
	int MyIdx =  Policy::ActiveDetailedPolicy.Front;
	int MyRule;

	auto LCXTargetState = Policy::ActiveDetailedPolicy.PolicyDQ[MyIdx];
	double LCXDAR = BehaviorDesired.TargetState.GrpEnd_DAR_m;
/*
	do{
		LCXDAR = LCXTargetState.DAR_m;
		MyIdx++;
		if(MyIdx>=Policy::ActiveDetailedPolicy.PolicyDQ.size())
			break;
		LCXTargetState = Policy::ActiveDetailedPolicy.PolicyDQ[MyIdx];
		if(LCXTargetState.step.current_segment_station_m==-1)
			break;
	}while(true); */

	bool JustClearPolicyMemWoUpdatingDetailedPolicies = false;
	bool NeighBorHoodTask = (MyTask == PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK) || (MyTask == PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK);
	bool AlreadyInDLCX = (MyTask == CHANGE_LANES_TASK) && ((MyDirection==BEAR_LEFT_OPPOSING_LANE)||(MyDirection==BEAR_RIGHT_OPPOSING_LANE));

	if( AlreadyInDLCX ||NeighBorHoodTask || DLCX_ALLWD_GLOBALLY){ // This is a temporary condition
		if((LCXDAR-vehicle.DTAR)<25){
			JustClearPolicyMemWoUpdatingDetailedPolicies = true;
		}
	}else{
		JustClearPolicyMemWoUpdatingDetailedPolicies = true;
	}

	if(Policy::ActiveDetailedPolicy.Cost.TotalCost<1.5){
		BehaviorUtils::codeBlue(_CF_ + "Not performing policy search. Active Policy cost is low ",BehaviorDesired.PolicyCost,ShowTmThrsh);
		JustClearPolicyMemWoUpdatingDetailedPolicies = true;
	}


	if(1e-3*(chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now()-LastExectTime)).count()>BEHAVIORPOLICY_EXEC_WAIT_TIME_S){
		LastExectTime = chrono::high_resolution_clock::now();

		//BU::codeGreen(_CF_ + "PolicyBehaviors ",PolicyBehaviors,ShowTmThrsh);

		if(Policy::AllCandidatePolicies.empty()){
			BehaviorUtils::codeBlue(_CF_ + "AllCandidatePolicies is empty. Returning "," ",ShowTmThrsh);
			return;
		}

		if(JustClearPolicyMemWoUpdatingDetailedPolicies){
			Policy::ClearPolicyMemory();
			return;
		}

		auto it=Policy::AllCandidatePolicies.begin();

		std::vector<exlcm::policycosts_t> AllPolicyCosts;
		double MinCost = 1e7;
		behavioroutput::CurrPolicy.Policy_costs.clear();
		for(auto itr=Policy::AllCandidatePolicies.begin();itr!=Policy::AllCandidatePolicies.end();itr++){
			if(Policy::CalculateCost(itr->second.Cost)<MinCost){
				MinCost = Policy::CalculateCost(itr->second.Cost);
				it = itr;
			}
			AllPolicyCosts.push_back(itr->second.Cost);
		}
		behavioroutput::CurrPolicy.Policy_costs = AllPolicyCosts;
		behavioroutput::CurrPolicy.Policy_count = AllPolicyCosts.size();

		//if(!behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause && BEHAVIORPOLICY_CONT_PUB_ENABLED)
			//behavioroutput::behavioroutput.publish("BEHAVIORPOLICY",&behavioroutput::CurrPolicy);

		std::string state;
		if(BehaviorDesired.reRouteNeeded!=NO_REROUTE){
			//BehaviorDesired.alternaterouteNeeded.insert(NO_REROUTE);
			BehaviorUtils::codeYellow(_CF_ + "Optimal policy not executed as reRouteNeeded ",BehaviorDesired.reRouteNeeded,1);
			state = "if";
		}
		else if((BehaviorTertiary::SpatialTaskGroup[it->second.PolicyOptimalBehavior.SpatialTask]==BehaviorTertiary::SpatialTaskGroup[BehaviorDesired.SpatialTask])&&\
				(BehaviorDesired.direction==it->second.PolicyOptimalBehavior.direction)) { // Same direction of lane shift
			if(BehaviorUtils::codeBlue(_CF_ + "Minimum policy task is same as current spatial policy "," ",ShowTmThrsh)){
				lcmprint::PrintEnum(cout,(route_step_task_enum)BehaviorDesired.SpatialTask);
			}
			state = "else if 1";

		}
		else if(BehaviorUtils::find(Policy::AllCandidatePolicies,HV_LANE_SITUATION)){
			if(Policy::CalculateCost(it->second.Cost)+ACTIVE_POLICY_CHANGE_COST_THRESH<Policy::CalculateCost(Policy::ActiveDetailedPolicy.Cost)){
				BehaviorUtils::codeYellow(_CF_ + " Policy changing. Optimal policy ",it->first,1);
				BehaviorUtils::codeYellow(_CF_ + " Policy changing. {Optimal policy , Active Policy  } ",std::make_pair(it->second,Policy::AllCandidatePolicies[HV_LANE_SITUATION]),1);
				//BehaviorDesired.alternaterouteNeeded.insert(NO_REROUTE);
				BehaviorDesired.reRouteNeeded = it->second.alternaterouteNeeded;
				state = "if else if 2";
			}
			state = "else if 2";
		}



		if(!behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause){
			if(ALTBEHAVIORVISUALIZATION_PUB_ENABLED){
				//behavioroutput::behavioroutput.publish("ALTERNATEBEHAVIORVISUALIZATION",&behavioroutput::_altbehaviorviz);
				behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks.clear();
				behavioroutput::_altbehaviorviz.PolicyCnt = 0;
			}
		}

		Policy::ClearPolicyMemory();
		return;

	}
	else if(JustClearPolicyMemWoUpdatingDetailedPolicies){
		return;
	}


	//	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,LANE_CHANGE_ROUTE_DLCX_ALLWD,LEFT_ADJACENT_LANE_SITUATION,leftlaneExists);

	//	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,LANE_CHANGE_ROUTE_DLCX_ALLWD,RIGHT_ADJACENT_LANE_SITUATION,rightlaneExists);

	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,LANE_CHANGE_ROUTE_DLCX_RQRD,LEFT_OPPOSING_LANE_SITUATION,leftOpposinglaneExists);

	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,LANE_CHANGE_ROUTE_DLCX_RQRD,RIGHT_OPPOSING_LANE_SITUATION,rightOpposinglaneExists);

	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,OPTIMAL_REROUTE,HV_LANE_SITUATION,HVlaneExists);

	Policy::SoughtAndFoundAlternatePolicy(BehaviorDesired,Policy::AllCandidatePolicies,STRAIGHT_ROUTE,STRIAGHT_LANE_SITUATION,HVlaneExists);



	return;


}

std::ostream& Policy::operator<<(std::ostream& os,const Policy::DetailedPolicy& MyPolicy){
	os<<" Policy Root Lane ";
	lcmprint::PrintEnum(os,(lane_situation_enum)MyPolicy.rootLane);
	os<<" Policy route ";
	lcmprint::PrintEnum(os,(reroute_type_enum)MyPolicy.alternaterouteNeeded);
	os<<" Policy Traffic jam time "<<MyPolicy.TrafficJamTime<<endl;
	os<<" Policy cost: ";
	lcmprint::operator <<(os,MyPolicy.Cost);
	os<<" routePlan ";
	lcmprint::operator <<(os,MyPolicy.Policy.routeplan);
	os<<"Front state index = "<<MyPolicy.Front<<endl;

	return os;
}

std::ostream& Policy::operator<<(std::ostream& os,const std::map<int,Policy::DetailedPolicy>& MyPolicyMap){
	for(auto MyPolicy:MyPolicyMap){
		os<<MyPolicy.second;
	}
	return os;
}



