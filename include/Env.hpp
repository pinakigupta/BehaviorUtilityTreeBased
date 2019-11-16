# pragma once

#include "../PCH/pch.hpp"
#include "BehaviorUtils.hpp"
#include "BehaviorStatic.hpp"
#include "BehaviorEnums.h"



namespace Env
{

class PolicyState
{
public:
	double DAR_m;//Distance Along Route in meters
	double GrpOrigin_DAR_m;//Distance Along Route in meters
	double GrpEnd_DAR_m;//Distance Along Route in meters
	//Only for stateType=STEP
	exlcm::pathstep_t step;
	std::set<right_of_way_rule_enum> observed_step_rule = {};
	//double current_segment_station_lcmin::pathPlan_m;
	bool UserConfirmReceived = false;
	std::vector<LaneSegUidType> CurrentGrpLanes;
	std::vector<LaneSegUidType> TargetGrpLanes;
	double CurrentToTgtGrpConnLen = 0;
	double CurrentGrpLen = 0;
	double TgtGrpLen = 0;
	double MinTravelTm;
	bool SuspectedOpposingGrp = false;
	InterSectionUidType InterSectionAhead;
	InterSectionUidType InterSectionBehind;
	PolicyState* Prev_State = nullptr;
	PolicyState* Next_State = nullptr;
};
};

inline size_t HashPolicyState(const Env::PolicyState& MyPolicy){
	size_t seed = 0;
	for(auto& lane:MyPolicy.CurrentGrpLanes)
		Hash_combine(seed,(int64_t)lane);
	for(auto& lane:MyPolicy.TargetGrpLanes)
		Hash_combine(seed,(int64_t)lane);
	Hash_combine(seed,(int64_t)MyPolicy.step.step_rule);
	Hash_combine(seed,(int64_t)MyPolicy.step.step_task);
	return seed;
}

typedef std::deque<Env::PolicyState> PolicyDequeType;

namespace Policy{
class DetailedPolicy;
}

namespace Env
{
enum stateType{ STEP, OBSTACLE};
extern double MaxDTAR;
extern Vehicle::vehicleState MaxDTAR_VehicleState;
extern bool initDone ;
extern int ReInitCompleted ;
extern int ReInitRequest ;
extern vector<int> SortedRouteSegmentGrpList;
extern map<int,double> SegmentOriginDAR;
extern std::pair<InterSectionUidType, double> _ClosestInterSectionInfo;

std::ostream &operator <<(std::ostream &os, const exlcm::pathstep_t &MyStep);
std::ostream &operator <<(std::ostream &os, const Env::PolicyState &MyState);
std::ostream &operator <<(std::ostream &os, const PolicyDequeType &MyDeque);

bool operator ==(const Env::PolicyState& a, const Env::PolicyState& b);
bool operator !=(const Env::PolicyState& a, const Env::PolicyState& b); // Almost equal, DAR_m allowed to be different
extern PolicyDequeType & operator+(const PolicyDequeType & , const PolicyDequeType & );


double LaneSegOriginDAR(LaneSegUidType,const Policy::DetailedPolicy &);
double LaneSegEndDAR(LaneSegUidType ,const Policy::DetailedPolicy & );
extern PolicyDequeType ActivePolicyDQ;
void initStates(Policy::DetailedPolicy &MyDetailedPolicy, bool ReInitRequest = false, double DAR_m=0.0, bool ActivePolicy = false);
//extern int Front;
bool initVehicleState(Policy::DetailedPolicy &, bool ReInitRequest= false);
void updateVehicleState(Policy::DetailedPolicy&);
double SegGrpOriginDAR(int segUID, const Policy::InputPolicyType &, map<int,double>&, double DAR_m_Origin = 0.0);
double SegGrpOriginDAR(int segUID, const Policy::DetailedPolicy & );
double SegGrpEndDAR(int segUID, const Policy::DetailedPolicy &);
std::pair<InterSectionUidType,double> FindClosestInterSectionInfo(int CurrentGrp);
std::pair<InterSectionUidType,double> FindClosestInterSectionInfo(Env::PolicyState& TargetState);

void PrintSortedDAR();
LaneSegUidType FindNextLaneinRoute(LaneSegUidType lanesegUID, const Policy::InputPolicyType &);
LaneSegUidType FindPrevLaneinRoute(LaneSegUidType lanesegUID, const Policy::InputPolicyType &);
int FindNextGrpinRoute(int segUID,const Policy::InputPolicyType &);
int FindPrevGrpinRoute(int segUID,const Policy::InputPolicyType &);

Env::PolicyState prev( const Env::PolicyState& CurrentState);
Env::PolicyState next( const Env::PolicyState& CurrentState);



double CalculateTargetDistanceAlongSeg(const exlcm::pathstep_t&,const Policy::InputPolicyType & MyPolicy );
int SearchState(const Env::PolicyState &TargetState,const PolicyDequeType &);
bool CheckAndPopCompletedStates();
void InitStatesAndVehicleStates(const Policy::InputPolicyType & );
void FindFrontState( Policy::DetailedPolicy & );
int FindClosestState(Policy::DetailedPolicy & );
Env::PolicyState FindClosestState(Env::PolicyState &TargetState);
};
extern bool initDone;

