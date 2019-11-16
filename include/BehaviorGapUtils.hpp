#pragma once
#include "../PCH/pch.hpp"

#include "BehaviorTertiary.hpp"
#include "behaviorHorizon.hpp"
#include "BehaviorObject.hpp"

namespace BU=BehaviorUtils;
#include "shared_enums.h"

inline exlcm::objdetails_t CreateObjDetails(ObjUidType  obj_ID,float obj_dist,int8_t lanesituation_index,int8_t  objpred_idx){
	exlcm::objdetails_t MyObjDetails;
	MyObjDetails.obj_ID = obj_ID;
	MyObjDetails.obj_dist = obj_dist;
	MyObjDetails.lanesituation_index = lanesituation_index;
	MyObjDetails.objpred_idx = objpred_idx;
	return MyObjDetails;
}

inline std::vector<exlcm::objdetails_t> CreateObjDetails(GapObjUidType obj_ID,GapType obj_dist,GapIntType lanesituation_index,GapIntType  objpred_idx){
	std::vector<exlcm::objdetails_t> MyObjDetails;
	MyObjDetails.push_back(CreateObjDetails(obj_ID.first,obj_dist.first,lanesituation_index.first,objpred_idx.first));
	MyObjDetails.push_back(CreateObjDetails(obj_ID.second,obj_dist.second,lanesituation_index.second,objpred_idx.second));
	return MyObjDetails;
}

inline std::vector<exlcm::objdetails_t> CreateObjDetails(GapObjUidType obj_ID,GapType obj_dist,int8_t lanesituation_index,GapIntType  objpred_idx){
	std::vector<exlcm::objdetails_t> MyObjDetails;
	MyObjDetails.push_back(CreateObjDetails(obj_ID.first,obj_dist.first,lanesituation_index,objpred_idx.first));
	MyObjDetails.push_back(CreateObjDetails(obj_ID.second,obj_dist.second,lanesituation_index,objpred_idx.second));
	return MyObjDetails;
}
extern pthread_mutex_t _lanechangeplot_mutex;
extern GapType IDEAL_LC_ZONE;
extern GapType BLIND_ZONE;

namespace BehaviorGapUtils{

GapType operator&(GapType MyGap, GapType OtherGap);
double GapOverLapFraction(GapType MyGap, GapType OtherGap);
double InstantGapOverLapFraction(GapType MyGap, GapPredFuncType GapOverLapPredfunc, double Tm);
GapPredFuncFracType GapOverLapFraction(GapType MyGap, GapPredFuncType GapOverLapPredfunc);
GapType GapOverLapPred(GapPredFuncType MyGapfunc, GapPredFuncType OtherGapfunc,double LkAhdTm);
GapPredFuncType operator&(GapPredFuncType MyGapfunc, GapPredFuncType OtherGapfunc);
GapType ConstGap(GapType ConstantGap,double LkAhdTm);
GapPredFuncType ConstGapFunc(GapType ConstantGap);
GapPredFuncType operator &(GapPredFuncType MyGapfunc,  GapType ConstantGap);
GapPredFuncType operator &(GapType OtherGap,GapPredFuncType MyGapfunc);
GapType PredictedGap(GapType CurrentGap,GapType CurrentGapVel,GapType CurrentAccel, GapType GapIDs, double LookAheadTime ,  bool IsthisHV );
double AssessGapCost(GapPredFuncType GapOverLapPredfunc, GapPredFuncType GapPredfunc);
std::vector<double> AssessGapCost(GapPredFuncTypeVec GapOverLapPredfunc, GapPredFuncTypeVec GapPredfunc);
void ExtractGapParam(GapObjUidType GapObjIDs, GapType & GapObjVel, GapType & GapObjAccel);
void AdjustGapOffset(const GapObjUidType ObjIDs ,GapType& offset);
GapType operator+(GapType A,GapType B);
std::tuple<GapType,GapType,GapType,GapObjUidType,GapPredFuncType,GapPredFuncType,GapIntType> CreateGapFuncs(GapPredFuncType center_ahead_gap_pred, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc,int gap_idx = 1);


std::ostream& operator<<(std::ostream& os,GapPredFuncType MyGapFunc);
std::ostream& operator<<(std::ostream& os,GapPredFuncFracType MyGapFuncFrac);
std::ostream& operator<<(std::ostream& os,GapPredFuncTypeVec MyGapFuncVec);

bool operator==(const GapType lhs,const GapType rhs);
bool operator==(const GapObjUidType lhs,const int Val);
bool operator==(const GapObjUidTypeVec lhs,const int Val);
void PlotPredictedGap(GapPredFuncTypeVec & MyGapfuncs,GapPredFuncTypeVec & MyGapOverLapfuncs, GapPredFuncType & CurrentLaneGap, std::vector<std::string> Mylabels);

struct PlotPredictedGap_arg_struct {
	GapPredFuncTypeVec MyGapfuncs;
	GapPredFuncTypeVec MyGapOverLapfuncs;
	GapPredFuncType CurrentLaneGap;
	std::vector<std::string> Mylabels;
};

class GapObject{
public:
	size_t uid = 0 ;
	GapIntType objpred_idx;
	GapType Gap,GapVel,GapAccel;
	GapObjUidType GapObjIDs;
	GapPredFuncType GapPredFunc, GapOverlapPredFunc;
	double GapCost=std::numeric_limits<double>::max();
	int lane = -1;
	double ObsrvdTime;

	// Constructors
	inline GapObject(){ObsrvdTime = BU::TimeNow();};
	GapObject(GapType x,GapType v,GapType a,GapObjUidType objUID, int LANE_SITUATION, GapIntType ObjPredIfx,GapPredFuncType LaneStaticGapPredFunc);
	GapObject(GapType x,GapType v,GapType a,GapObjUidType UID, int LANE_SITUATION, GapIntType ObjPredIfx,const GapObject& Center_Ahead, GapPredFuncType LaneStaticGapPredFunc);
	GapObject(const GapObject& Other,const GapObject& Center_Ahead,GapPredFuncType LaneStaticGapPredFunc);
	GapObject(const GapObject& Center_Ahead, int LANE_SITUATION,GapPredFuncType LaneStaticGapPredFunc, int gap_idx = 1);

	//Functions

	void UpdateOverLapPredFunc(GapPredFuncType center_ahead_gap_pred);
	void UpdatePredFunc();
	void Hash(size_t& seed);


};

GapObject operator&(const GapObject& MyGapobj, const GapObject& OtherGapObj);

typedef std::vector<GapObject> GapObjectVec;

GapObjectVec LaneGapObjVec(const GapObject& Center_Ahead, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc, int sign =1);

double AssessGapCost(GapObject& MyGapObject);
std::vector<double> AssessGapCost(GapObjectVec& MyGapObjectVec);

std::ostream& operator<<(std::ostream& os,const BehaviorGapUtils::GapObject& MyGapObject);
std::ostream& operator<<(std::ostream& os,const BehaviorGapUtils::GapObjectVec& MyGapObjectVec);



void PlotGaps(GapObjectVec& ,GapObject& ,GapObjectVec& ,GapObject& );

exlcm::behaviorgapviz_t construct_behaviorgapviz(float gap_pred_time,float gap_start,float gap_end);
exlcm::behaviorgapviz_t construct_behaviorgapviz(float gap_pred_time,const GapObject& MyGap);

void *PlotPredictedGapHandler(void *);

}

BehaviorTertiary::Behavior& operator<<(BehaviorTertiary::Behavior& MyBehavior,const BehaviorGapUtils::GapObject& MyGap);
BehaviorTertiary::Behavior& operator<(BehaviorTertiary::Behavior& MyBehavior,const BehaviorGapUtils::GapObject& MyGap);

//extern BehaviorGapUtils::GapObjectVec TargetLane_Ahead,TargetLane_Behind;
//extern BehaviorGapUtils::GapObject TargetLane_Adjacent;
//extern BehaviorGapUtils::GapObject Center_Ahead,Center_Behind;

extern BehaviorTertiary::Behavior GetSimpleLaneBehavior(Env::PolicyState &TargetState, std::pair<int,int> lane);
extern void PlotGaps(BehaviorGapUtils::GapObjectVec& Behind,BehaviorGapUtils::GapObject& Adjacent,BehaviorGapUtils::GapObjectVec& Ahead,BehaviorGapUtils::GapObject& Center_Ahead);
extern void FilterSmallLCZ(BehaviorTertiary::Behavior& MyBehavior,double laneChangeZoneEnd_Thresh = 0);
extern BehaviorTertiary::Maneuver_Temporal CreateGap(BehaviorTertiary::Behavior& Current_Lane_Behavior,const BehaviorGapUtils::GapObject& Ahead , const BehaviorGapUtils::GapObject& Adjacent,\
		int LANE_SITUATION, GapPredFuncType );

extern void ExecuteLaneChange(int TARGET_LANE_SITUATION, double env_s0, double env_s1 , 
						BehaviorTertiary::Behavior& Current_Lane_Behavior, BehaviorTertiary::Behavior& AdjacentLaneBehaviorDesired,
						Env::PolicyState &TargetState, int LaneChangeIntentionOvrrd, bool TrajCommitted);
extern int GetAlternateLane(int LANE_SITUATION,Env::PolicyState &TargetState);

