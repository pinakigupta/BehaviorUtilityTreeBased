#pragma once

#include "../PCH/pch.hpp"

#include "Vehicle.hpp"
#include "BehaviorUtils.hpp"
#include "behaviorHorizon.hpp"
#include "BehaviorSecondary.hpp"
#include "BehaviorStatic.hpp"
#include "BehaviorObject.hpp"
#include "shared_enums.h"
#include "lcmprint.hpp"
namespace BU = BehaviorUtils;

typedef std::pair<int,int> GapIntType;
typedef std::pair<double,double> GapType;
typedef std::function<GapType (double )> GapPredFuncType;
typedef std::function<double (double )> GapPredFuncFracType;
typedef std::pair<ObjUidType,ObjUidType> GapObjUidType;
typedef std::vector<GapType> GapTypeVec;
typedef std::vector<GapObjUidType> GapObjUidTypeVec;
typedef std::vector<GapPredFuncType> GapPredFuncTypeVec;

namespace BehaviorTertiary
{
const double MaxAccelLimit = 9.8; //m/sec^2
extern std::map<double,double> AccelFactorDefault;



class BehaviorPolicyCostTree{
public:
	exlcm::behaviorpolicycosts_t RootBehaviorPolicyCost;
	std::vector<BehaviorPolicyCostTree*> ChildBehaviorPolicyCosts = {};
};



const map<decltype(exlcm::pathstep_t::step_task),int> SpatialTaskGroup = { {SHUTDOWN_TASK,LANE_SHUTDOWN}, {STANDBY_TASK,LANE_PROPAGATE}, {PROCEED_TASK,LANE_PROPAGATE}, \
		{CHANGE_LANES_TASK,LANE_SHIFT},{MERGE_TASK,LANE_PROPAGATE}, {EXIT_TASK,LANE_PROPAGATE}, {TURN_TASK,LANE_PROPAGATE}, {REVERSE_TASK,LANE_REVERSE_PROPAGATE}, \
		{REVERSE_TURN_TASK,LANE_REVERSE_SHIFT},{PROCEED_AFTER_LANE_CHANGE_TASK,LANE_PROPAGATE},{PROCEED_WITH_CAUTION_PARK_ZONE_RIGHT_SIDE_TASK,LANE_PROPAGATE},\
		{PROCEED_WITH_CAUTION_PARK_ZONE_LEFT_SIDE_TASK,LANE_PROPAGATE},\
		{PROCEED_WITH_CAUTION_PARK_ZONE_BOTH_SIDES_TASK,LANE_PROPAGATE},{PROCEED_THROUGH_PARKING_AISLE_TASK,LANE_PROPAGATE}};

const map<int, int> BehaviorPreference = {
		{ACC_BEHAVIOR,ACC_BEHAVIOR_ORDER},
		{LANE_CHANGE_BEHAVIOR,LANE_CHANGE_BEHAVIOR_ORDER},
		{LANE_KEEPING_BEHAVIOR,LANE_KEEPING_BEHAVIOR_ORDER},
		{TURN_LEFT_BEHAVIOR,TURN_LEFT_BEHAVIOR_ORDER},
		{TURN_RIGHT_BEHAVIOR,TURN_RIGHT_BEHAVIOR_ORDER},
		{TURN_ALONG_CURVY_ROAD_BEHAVIOR,TURN_ALONG_CURVY_ROAD_BEHAVIOR_ORDER},
		{YIELD_TO_PEDESTRIAN_BEHAVIOR,YIELD_TO_PEDESTRIAN_BEHAVIOR_ORDER},
		{YIELD_TO_CROSS_TRAFFIC_BEHAVIOR,YIELD_TO_CROSS_TRAFFIC_BEHAVIOR_ORDER},
		{HANDLE_INTERSECTION_BEHAVIOR,HANDLE_INTERSECTION_BEHAVIOR_ORDER},
		{STOP_AT_STOP_BAR_BEHAVIOR,STOP_AT_STOP_BAR_BEHAVIOR_ORDER},
		{ENTERING_ROUNDABOUT_BEHAVIOR,ENTERING_ROUNDABOUT_BEHAVIOR_ORDER},
		{EXITING_ROUNDABOUT_BEHAVIOR,EXITING_ROUNDABOUT_BEHAVIOR_ORDER},
		{MERGING_BEHAVIOR,MERGING_BEHAVIOR_ORDER},
		{EXITING_FLOW_BEHAVIOR,EXITING_FLOW_BEHAVIOR_ORDER},
		{STOP_AT_TRAFFIC_LIGHT_BEHAVIOR,STOP_AT_TRAFFIC_LIGHT_BEHAVIOR_ORDER},
		{UNKNOWN_BEHAVIOR,UNKNOWN_BEHAVIOR_ORDER},
		{SAFETY_FIRST_BEHAVIOR,SAFETY_FIRST_BEHAVIOR_ORDER},
		{COLLISION_IMMINENT_BEHAVIOR,COLLISION_IMMINENT_BEHAVIOR_ORDER},
		{EVASIVE_LANE_CHANGE_BEHAVIOR,EVASIVE_LANE_CHANGE_BEHAVIOR_ORDER}
};


class Maneuver_Spatial{
public:

	//Behavior output
	active_maneuver_enum activeManeuver ;
	int reRouteNeeded = NO_REROUTE  ;
	std::set<int> alternaterouteNeeded = {}  ;
	int direction= STRAIGHT_DIRECTION;
	lane_intention_enum leftLaneChangeIntention ;
	lane_intention_enum rightLaneChangeIntention ;
	double laneChangeZoneStart_m = 0.0;
	double laneChangeZoneEnd_m = 0.0;
	double TgtlaneChangeZoneStart_m = 0.0 ;
	double TgtlaneChangeZoneEnd_m = 0.0;
	exlcm::behaviorlanegapsviz_t TargetLaneChangeGapPred;
	exlcm::behaviorlanegapsviz_t CurrentLaneChangeGapPred;
	exlcm::behaviorlanegapsviz_t AllCurrentLaneGaps;
	exlcm::behaviorlanegapsviz_t AllTargetLaneGaps;
	map<tuple<ObjUidType,int,int>,exlcm::objdetails_t> relevant_objs = {}; // These are spatial relevant objects  Key = tuple(ObjUidType,Lane SA idx, Obj Pred Idx)

	//Internal variables
	double Maneuver_Spatial_cost_wt = 0.5;
	route_step_task_enum SpatialTask = STANDBY_TASK;
	short turn_indicator_command = TURN_INDICATOR_NONE; // 0 - unknown 1- left 2 - right 3 - Emergency Flash 4- No turn
	double turn_indicator_target_distance;
	bool SpatialTargetsSet = false;
	double Spatial_cost = std::numeric_limits<double>::max();
	double PolicyCost = std::numeric_limits<double>::max();

	//functions
	virtual void CalculateCost();
	virtual void inline InsertObj(exlcm::objdetails_t obj){
		if(obj.obj_ID>0){
			relevant_objs[make_tuple(obj.obj_ID,obj.lanesituation_index,obj.objpred_idx)]=obj;
		}
	}
	virtual void inline InsertObj(vector<exlcm::objdetails_t> objs){
		for(auto &obj:objs)
			this->InsertObj(obj);
	}
	virtual void inline InsertObj(pair<exlcm::objdetails_t,exlcm::objdetails_t> objs){
		this->InsertObj(objs.first);
		this->InsertObj(objs.second);
	}
	virtual void inline InsertObj(vector<pair<exlcm::objdetails_t,exlcm::objdetails_t> > objID){
		for(auto &obj:objID)
			this->InsertObj(obj);
	}
	virtual void inline EmptyObj(){
		this->relevant_objs.clear();
	}

	//Constructors
	Maneuver_Spatial();
	Maneuver_Spatial(Env::PolicyState &TargetState);
	Maneuver_Spatial(active_maneuver_enum my_active_maneuver, lane_intention_enum my_LeftLCInt, lane_intention_enum  my_rightLCInt, \
			double my_LCZ_Start_m, double my_LCZ_End_m, decltype(exlcm::pathstep_t::step_task) my_SpatialTask , short  my_TI_Cmnd );
};

class Maneuver_Temporal{
public:

	//Behavior output
	double speedTarget ;
	double AccelTarget;
	double speedTargetDistance;
	double speedTargetDistanceActual=-1 ;
	int8_t horn_request = 0;
	int primary_target_object = -1; // This is temporal relevant object
	behavior_accel_request BehaviorAccelRequestType = COMFORT_ACCEL_REQUEST_BEHAVIOR;
	std::vector<Maneuver_Temporal> LeftAdjacentLane = {};
	std::vector<Maneuver_Temporal> RightAdjacentLane = {};


	//Internal variables
	double Maneuver_Temporal_cost_wt = 0.5;
	double ProjectedAcceleration,EstimatedTimeToTarget ;
	double Temporal_cost = std::numeric_limits<double>::max();
	double PolicyCost = std::numeric_limits<double>::max();
	double Temporal_cost_bias = 0;
	bool   TemporalTargetsSet = false;
	TempRules TemporalRule = NOT_DEFINED_YET;

	//functions
	virtual void CalculateCost();


	//Constructors
	Maneuver_Temporal();
	Maneuver_Temporal(Env::PolicyState &TargetState,std::pair<int,int> lane = {HV_LANE_SITUATION,0});
	//	Maneuver_Temporal(double my_accelTarget,double my_speedTarget,double my_speedTargetDistance,double my_ProjectedAcceleration); // Not encouraged to use this constructor vs the next one
	Maneuver_Temporal::Maneuver_Temporal(double my_accelTarget,double my_speedTarget,double my_speedTargetDistance,double my_ProjectedAcceleration,\
			decltype(exlcm::pathstep_t::step_rule) my_TemporalRule);

};

class Behavior:public Maneuver_Spatial, public Maneuver_Temporal
{
public:

	//Behavior output
	behavior_type behaviorType ;
	set<behavior_type> ConsideredBehaviorAgents = {};

	//Internal variables
	double Behavior_cost = std::numeric_limits<double>::max();
	double PolicyCost = std::numeric_limits<double>::max();
	Env::PolicyState TargetState;
	BehaviorPolicyCostTree *BehaviorPolicyCosts = nullptr;

	//functions
	void CalculateCost();
	void CalculateBehavior();
	void SetBehaviorPolicyCost(){
		this->BehaviorPolicyCosts = new BehaviorPolicyCostTree;
		BehaviorPolicyCosts->RootBehaviorPolicyCost.PolicyCost = this->Behavior::PolicyCost;
		BehaviorPolicyCosts->RootBehaviorPolicyCost.SpatialManeuverCost = this->Spatial_cost;
		BehaviorPolicyCosts->RootBehaviorPolicyCost.TemporalManeuverCost = this->Temporal_cost;
		BehaviorPolicyCosts->RootBehaviorPolicyCost.BehaviorType = BU::to_string(this->behaviorType);
		BehaviorPolicyCosts->RootBehaviorPolicyCost.SpatialTask = BU::to_string(this->SpatialTask);
		BehaviorPolicyCosts->RootBehaviorPolicyCost.TemporalRule = BU::to_string(this->TemporalRule);
	}

	void SetBehaviorPolicyCost(Behavior& BehaviorA,Behavior& BehaviorB){

		this->Behavior::PolicyCost = BehaviorA.Behavior::PolicyCost + BehaviorB.Behavior::PolicyCost;
		SetBehaviorPolicyCost();
		this->BehaviorPolicyCosts->ChildBehaviorPolicyCosts.push_back(std::move(BehaviorA.BehaviorPolicyCosts));
		this->BehaviorPolicyCosts->ChildBehaviorPolicyCosts.push_back(std::move(BehaviorB.BehaviorPolicyCosts));
		BehaviorA.BehaviorPolicyCosts = nullptr;
		BehaviorB.BehaviorPolicyCosts = nullptr;
	}

	exlcm::behaviorpolicycosts_t GetBehaviorPolicyCost(){
		return BehaviorPolicyCosts->RootBehaviorPolicyCost;
	}


	//Constructors
	Behavior();
	Behavior(Env::PolicyState &TargetState, std::pair<int,int> lane = {HV_LANE_SITUATION,0});
	Behavior(BehaviorTertiary::Maneuver_Spatial&, BehaviorTertiary::Maneuver_Temporal&);
	Behavior(Behavior& Root,BehaviorTertiary::Maneuver_Spatial&, BehaviorTertiary::Maneuver_Temporal&);

	//destructor
	//	~Behavior(){
	//		DeleteTree(this->BehaviorPolicyCosts);
	//	}


};

behavior_type CalculateBehavior(decltype(exlcm::pathstep_t::step_task) SpatialTask,decltype(exlcm::pathstep_t::step_rule) TemporalRule );


BehaviorTertiary::Behavior operator+(const BehaviorTertiary::Behavior&, const Maneuver_Temporal&);

BehaviorTertiary::Maneuver_Spatial operator*(const BehaviorTertiary::Maneuver_Spatial& myClass, double Factor);
BehaviorTertiary::Maneuver_Temporal operator*(const BehaviorTertiary::Maneuver_Temporal& myClass, double Factor);
BehaviorTertiary::Behavior operator*(const BehaviorTertiary::Behavior& myClass, double Factor);

Maneuver_Temporal operator+(const Maneuver_Temporal& , const Maneuver_Temporal& );
//Maneuver_Spatial operator+(const Maneuver_Spatial& , const Maneuver_Spatial& );
Behavior operator+( Behavior& ,  Behavior& );

bool operator<(const BehaviorTertiary::Maneuver_Temporal& , const Maneuver_Temporal& );


Maneuver_Temporal VNM_RationalizedAgent(std::vector<Maneuver_Temporal>& myClass, std::vector<double>);
Behavior VNM_RationalizedAgent(std::vector<Behavior>& myClass, std::vector<double>);

inline void PrintSpatialTaskGroup(std::ostream &os, int SpatialTask ){
	os<<" Behavior SpatialTaskGroup = ";
	switch(BehaviorTertiary::SpatialTaskGroup[SpatialTask])
	{
	case LANE_SHUTDOWN: {os<<green_on<<"LANE_SHUTDOWN"<<color_off<<endl;break;}
	case LANE_PROPAGATE: {os<<green_on<<"LANE_PROPAGATE"<<color_off<<endl;break;}
	case LANE_SHIFT: {os<<green_on<<"LANE_SHIFT"<<color_off<<endl;break;}
	case LANE_REVERSE_PROPAGATE: {os<<green_on<<"LANE_REVERSE_PROPAGATE"<<color_off<<endl;break;}
	case LANE_REVERSE_SHIFT: {os<<green_on<<"LANE_REVERSE_SHIFT"<<color_off<<endl;break;}
	default: {os<<blue_on<<"Not sure. Behavior SpatialTaskGroup Value is "<<BehaviorTertiary::SpatialTaskGroup[SpatialTask]<<endl;break;}
	}
}
std::ostream& operator<<(std::ostream &os, const set<behavior_type>& );
std::ostream& operator<<(std::ostream &os, const Behavior &);
std::ostream& operator<<(std::ostream &os, const Maneuver_Temporal& );
std::ostream& operator<<(std::ostream &os, const Maneuver_Spatial& );
//void PrintBehaviorPolicyCostTree(std::ostream& os, const BehaviorPolicyCostTree *root, int level = 0 );


extern BehaviorTertiary::Maneuver_Temporal YieldToTraffic(Env::PolicyState& TargetState, int MajorBranchYieldStatus = LN_YIELD_DUE_TO_STATIC_INFERIORITY, \
		bool CreepAllowed = false, bool NOYieldToDistCrossLanes = false);


void PostProcessForLcmPublish( BehaviorTertiary::Behavior &);
BehaviorTertiary::Behavior ControlComplexBehavior(Env::PolicyState& ,bool MinorConfirmed);
BehaviorTertiary::Behavior InterSectionManeuver(Env::PolicyState& TargetState,bool MinorConfirmed = false);
BehaviorTertiary::Maneuver_Temporal InterSectionCreepManeuver(Env::PolicyState& TargetState);

extern Maneuver_Temporal Default_Maneuver_Temporal;
std::ostream& Print(std::ostream &os, const TempRules My_rule);
};



