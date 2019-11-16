#pragma once
#include "../PCH/pch.hpp"


#include "Vehicle.hpp"
#include "BehaviorTypeDef.hpp"

namespace behavioroutput{
extern exlcm::behaviorplan_t behaviorPlan;
extern exlcm::useroutputs_t userout;
extern exlcm::behaviorvisualization_t _behaviorviz;
extern exlcm::behaviorvisualization_t _altbehaviorviz;
extern exlcm::lanegapvisualization_t _lanegapviz;
extern exlcm::behaviorpolicy_t initPolicy,CurrPolicy;
//extern lcm::LCM behavioroutput,behavioroutputfast;
extern std::vector< exlcm::policybehaviorstring_t > AllPolicyBehaviorStacks;
}
using namespace  std;
namespace Mutex{
extern pthread_mutex_t _obj_SA_mutex,_SA_mutex,_traffic_signal_mutex, _traffic_sign_mutex, _IntersectionList_mutex,\
_routeSegmentList_mutex,_visualizerstates_mutex,_lanechangeplot_mutex,_alternateplan_mutex,_behavioroutputfast_mutex;
}

namespace behaviorinput{
extern exlcm::visualizerstates_t _visualizerstates;
extern exlcm::trajectoryplan_t _trajectoryplan;
extern map< LaneSegUidType , exlcm::lanesegment_t> lanesegments;
extern exlcm::localization_t hvLoc;
extern std::map< std::pair<LaneSegUidType,int>, std::pair<exlcm::connectiontrafficsignal_t,double> > TrafficSignalStatesList,TrafficSignalStatesList_IN;
extern std::map< std::pair<LaneSegUidType,int>, std::pair<exlcm::connectiontrafficsign_t,double> > TrafficSignStatesList,TrafficSignStatesList_IN;
extern std::map<InterSectionUidType,exlcm::intersection_t> IntersectionList, IntersectionList_IN;
extern std::map<int, exlcm::routesegmentlist_t> AllaltrouteSegmentList,AllaltrouteSegmentList_IN;
extern std::map<int, exlcm::pathplan_t> AllaltpathPlan,AllaltpathPlan_IN;
extern std::unordered_map<int,ClockType> AllaltpathPlan_IN_LastUpdateTime,AllaltrouteSegmentList_IN_LastUpdateTime;
extern exlcm::pathplan_t pathPlan, pathPlan_IN;//To store a local copy of the behaviorinput::pathPlan
extern exlcm::routesegmentlist_t routeSegmentList,routeSegmentList_IN ;
extern unique_ptr<exlcm::locsegcoords_t>  curPosInfo;
inline void PrintAllLaneSeg(){
	std::set<ObjUidType> MySortedObjectList;
	for(auto &obj:lanesegments){
		MySortedObjectList.insert(obj.first);
	}
	cout<<MySortedObjectList<<endl;
}

}



extern Vehicle::vehicleState vehicle;



struct pairhash {
public:
	template <typename T, typename U>
	std::size_t operator()(const std::pair<T, U> &x) const
	{
		return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
	}
};


namespace BehaviorTertiary{
class Behavior;
}



extern ClockType SysTimer,LastPauseClock;
extern std::vector<InterSectionUidType> _InterSectionUIDList;;
// polling LCM fd
extern struct pollfd fds;
//extern lcm::LCM lcm1;
extern bool validpathPlanAvailable;// Flag is set when a valid behaviorinput::pathPlan is available
extern bool validRouteSegListAvailable;
extern bool allMapletlanesegmentsAvailable;
extern bool ValidFirstLocalization;
extern bool ValidFirstSegcoordsLocalization;
extern bool SIMFREEZEONSET;
extern bool abortCurPlan;// Cancel current behaviorinput::pathPlan if set
extern int8_t _tplan_lcx_status;

extern std::map<InterSectionUidType, std::tuple<bool, bool,bool> >  _InterSectionEntryExitLog;



extern double start_clk;
extern  double clk;
extern double Behavior_time ;




extern double ExecTime;



