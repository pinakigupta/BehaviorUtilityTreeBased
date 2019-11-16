#pragma once
#include "../PCH/pch.hpp"


#include "shared_enums.h"
#include "BehaviorTypeDef.hpp"


const double DEFAULT_VIEW_RANGE = 200;
extern class SituationAwareness
{
public:
	exlcm::situation_t HVData;
	std::unordered_map<int,exlcm::objsituation_t> objData;
	std::unordered_map<int,ClockType> objDataUpdateClock;

	sa_lane_yield_priority_enum HVLaneYieldPriority(InterSectionUidType intersection_uid);
	bool IsObjYieldingToHV(ObjUidType AdjacentBehindID,int Adjbehindhypidx);
	exlcm::objsituation_t GetObj(int uid);
	int min_cross_range_lane_idx(exlcm::intersectioncrosslane_t& LaneData);
    double OptCreepStation(InterSectionUidType intersection_uid);
    double cross_lane_dist();
    double cross_lane_SpdLmt();
    double cross_view_range();
    std::vector<LaneSegUidType> AllIntersectingLaneIDs(InterSectionUidType intersection_uid = -1);
    std::vector<int8_t> AllIntersectingLaneYieldStates(InterSectionUidType intersection_uid);
    int8_t FindCrossLaneIntIdx(InterSectionUidType intersection_uid);
	int FindAuxlane_idx(int SITUATION );
	double GetLaneAvgSpd(int LANE_SITUATION,int ZONE=0);
	ObjUidType behind_obj_id(int LANE_SITUATION, int rel_idx_Tgt = 0, int *Myidx = 0, int* Hypidx = 0);
	double clear_dist_behind(int LANE_SITUATION,int rel_idx_Tgt = 0);
	ObjUidType ahead_obj_id(int LANE_SITUATION,int rel_idx_Tgt = 0,int* Myidx=0, int* Hypidx=0);
	ObjUidType ahead_roadside_obj_id(int LANE_SITUATION,int rel_idx_Tgt = 0,int* Myidx=0, int* Hypidx=0);
	double clear_dist_ahead(int LANE_SITUATION = HV_LANE_SITUATION,int rel_idx_Tgt = 0);
	double clear_dist_ahead_by_ID(int LANE_SITUATION,ObjUidType objID );
	double roadside_dist_ahead(int LANE_SITUATION = HV_LANE_SITUATION,int rel_idx_Tgt = 0);
	double clear_dist_ahead_by_UID( ObjUidType objID, int LANE_SITUATION= HV_LANE_SITUATION) ;
	double roadside_dist_ahead_by_UID( ObjUidType objID, int LANE_SITUATION= HV_LANE_SITUATION) ;
	bool LaneSituationExists(int LANE_SITUATION);
	ObjUidType merge_obj_uid(int LANE_SITUATION, int rel_idx_Tgt = 0, int* Hypidx=0);
	double clear_dist_ahead_merge(int LANE_SITUATION, int rel_idx_Tgt = 0);
	ObjUidType parallel_obj_uid(int LANE_SITUATION, int rel_idx_Tgt = 0, int* Hypidx=0);
	double clear_dist_ahead_parallel(int LANE_SITUATION, int rel_idx_Tgt = 0);
	ObjUidType cross_obj_uid(int LANE_SITUATION, int rel_idx_Tgt = 0, int* Hypidx=0);
	double clear_dist_ahead_cross(int LANE_SITUATION, int rel_idx_Tgt = 0);
	void PrintSituation();
	std::set<ObjUidType> PrintObjectList(bool display = true);
	void PrintObjTimeDetails(ObjUidType uid);
	void PrintObjDetails(ObjUidType uid);
	double sensor_range_ahead(int LANE_SITUATION = HV_LANE_SITUATION);
	double sensor_range_behind(int LANE_SITUATION = HV_LANE_SITUATION);


}Situations;
extern SituationAwareness SA,SA_IN,SA_IN_HV;

std::ostream& operator<<(std::ostream& cout, const SituationAwareness& MySA );
void TimeDiffFromPause(ClockType Timer);
