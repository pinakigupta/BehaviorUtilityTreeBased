#include "../../include/BehaviorGapUtils.hpp"
#include "../../include/BehaviorUtils.hpp"

using namespace BehaviorGapUtils;
namespace BU = BehaviorUtils;

void SetCorrectLaneChangeIntention(int TARGET_LANE_SITUATION,BehaviorTertiary::Behavior& Current_Lane_Behavior,int LaneChangeIntentionVal ){
	if(TARGET_LANE_SITUATION==LEFT_ADJACENT_LANE_SITUATION)
		Current_Lane_Behavior.leftLaneChangeIntention = LaneChangeIntentionVal;
	else if(TARGET_LANE_SITUATION==RIGHT_ADJACENT_LANE_SITUATION)
		Current_Lane_Behavior.rightLaneChangeIntention = LaneChangeIntentionVal;
	else if(TARGET_LANE_SITUATION==LEFT_OPPOSING_LANE_SITUATION)
		Current_Lane_Behavior.leftLaneChangeIntention = LaneChangeIntentionVal;
	else if(TARGET_LANE_SITUATION==RIGHT_OPPOSING_LANE_SITUATION)
		Current_Lane_Behavior.rightLaneChangeIntention = LaneChangeIntentionVal;

}

void ExecuteLaneChange(int TARGET_LANE_SITUATION, double env_s0, double env_s1 , 
						BehaviorTertiary::Behavior& Current_Lane_Behavior, BehaviorTertiary::Behavior& AdjacentLaneBehaviorDesired,
						Env::PolicyState &TargetState, int LaneChangeIntentionOvrrd, bool TrajCommitted){
	Current_Lane_Behavior.laneChangeZoneStart_m= max(env_s0,0.0);//TODO:Tune based on clrDAhd
	Current_Lane_Behavior.laneChangeZoneEnd_m=env_s1;//LCZ valid only when on this curLaneSeg
	Current_Lane_Behavior.TgtlaneChangeZoneStart_m = max(env_s0,0.0);
	Current_Lane_Behavior.TgtlaneChangeZoneEnd_m = env_s1;
	Current_Lane_Behavior.leftLaneChangeIntention=LANE_CHANGE_IS_NOT_DESIRED;
	Current_Lane_Behavior.rightLaneChangeIntention=LANE_CHANGE_IS_NOT_DESIRED;

	double SpdLmt = Current_Lane_Behavior.TargetState.step.posted_speed_lim_mps;
	double SpdRef = SpdLmt;
	//SpdRef = fmin(SpdRef,vehicle.curSpeed_mps);
	double LCX_NOT_DESIRED_ZONE = LCX_TM_LEFT_FOR_LCX_ABRT_s*SpdRef;
	double LCX_NOT_COMMITTED_NOT_DESIRED_ZONE = LCX_TM_LEFT_FOR_LCX_ABRT_TRAJ_COMMITED_s*SpdRef;
	double GAP_CREATION_DIST_THRSH = LCX_TM_LEFT_FOR_GAP_CREATION_THRSH_s*SpdRef;
	double GAP_CREATION_NOT_DESIRED_DIST_THRSH = LCX_TM_LEFT_FOR_GAP_CREATION_ABRT_THRSH_s*SpdRef;


	BehaviorGapUtils::GapObjectVec TargetLane_Ahead,TargetLane_Behind;
	BehaviorGapUtils::GapObject TargetLane_Adjacent;
	BehaviorGapUtils::GapObject Center_Ahead,Center_Behind;

	GapType offset = {VEHICLE_LENGTH/2 , -VEHICLE_LENGTH/2};



	//pthread_mutex_lock(&Mutex::_lanechangeplot_mutex);


	int LaneObjIdx,AhdhypIdx,BhndhypIdx;
	ObjUidType CenterAheadObjUID,CenterBehindObjUID;
	CenterAheadObjUID = SA.ahead_obj_id(HV_LANE_SITUATION,0,&LaneObjIdx,&AhdhypIdx);
	GapType center_ahead_gap = std::make_pair(0 , SA.clear_dist_ahead(HV_LANE_SITUATION)) + std::make_pair(0, -VEHICLE_LENGTH/2);
	//Center gap ahead starts from 0 as the HV can occupy (0 to VEHICLE_LENGTH/2 ) space also in future time.
	GapObjUidType CenterAheadObjIDs = std::make_pair(-1,CenterAheadObjUID);
	GapType center_ahead_gap_vel = std::make_pair(0,BU::ExtractParamFromObjSA(CenterAheadObjIDs.second,"vel")[AhdhypIdx]-vehicle.curSpeed_mps); // First element is zero
	GapType center_ahead_gap_accel = std::make_pair(0,BU::ExtractParamFromObjSA(CenterAheadObjIDs.second,"accel")[AhdhypIdx]-vehicle.PredAccel_mpss); // First element is zero


	CenterBehindObjUID = SA.behind_obj_id(HV_LANE_SITUATION,0,&LaneObjIdx,&BhndhypIdx);
	GapType center_behind_gap = std::make_pair(SA.clear_dist_behind(HV_LANE_SITUATION),0 ) + offset;//SA maybe already doing the vehicle length subtraction.
	GapObjUidType CenterBehindObjIDs = std::make_pair(CenterBehindObjUID,-1);
	GapType center_behind_gap_vel = std::make_pair(BU::ExtractParamFromObjSA(CenterBehindObjIDs.second,"vel")[BhndhypIdx]-vehicle.curSpeed_mps,0); // Second element is zero
	GapType center_behind_gap_accel = std::make_pair(BU::ExtractParamFromObjSA(CenterBehindObjIDs.second,"accel")[BhndhypIdx]-vehicle.PredAccel_mpss,0); // Second element is zero



	GapPredFuncType LaneStaticGapPredFunc = std::bind(BehaviorGapUtils::PredictedGap,make_pair(-500,env_s1),make_pair(vehicle.curSpeed_mps,-vehicle.curSpeed_mps),\
			make_pair(vehicle.PredAccel_mpss,-vehicle.PredAccel_mpss),make_pair(-1,-1),std::placeholders::_1,false);

	GapPredFuncType LaneSnsrRngGapPredFunc = std::bind(BehaviorGapUtils::PredictedGap,make_pair(SA.sensor_range_behind(TARGET_LANE_SITUATION),\
			SA.sensor_range_ahead(TARGET_LANE_SITUATION)), make_pair(0,0),make_pair(0,0),make_pair(-1,-1),std::placeholders::_1,false);

	LaneStaticGapPredFunc = LaneStaticGapPredFunc & LaneSnsrRngGapPredFunc;

	GapIntType AhdGapHyp = std::make_pair(0,AhdhypIdx);
	GapIntType BhndGapHyp = std::make_pair(BhndhypIdx,0);
	Center_Ahead = BehaviorGapUtils::GapObject(center_ahead_gap,center_ahead_gap_vel,center_ahead_gap_accel,CenterAheadObjIDs,HV_LANE_SITUATION,AhdGapHyp,LaneStaticGapPredFunc);
	Center_Behind = BehaviorGapUtils::GapObject(center_behind_gap,center_behind_gap_vel,center_behind_gap_accel,CenterBehindObjIDs,HV_LANE_SITUATION,BhndGapHyp,LaneStaticGapPredFunc);



	int hypidx,garbage;
	int aheadID = SA.ahead_obj_id(HV_LANE_SITUATION,0,&garbage,&hypidx);
	auto AheadObjDetails = CreateObjDetails(aheadID,SA.clear_dist_ahead(HV_LANE_SITUATION),HV_LANE_SITUATION,hypidx);
	int behindID = SA.behind_obj_id(HV_LANE_SITUATION,0,&garbage,&hypidx);
	auto BehindObjDetails = CreateObjDetails(behindID,SA.clear_dist_behind(HV_LANE_SITUATION),HV_LANE_SITUATION,hypidx);
	Current_Lane_Behavior.InsertObj(AheadObjDetails);Current_Lane_Behavior.InsertObj(BehindObjDetails);


	static std::map<int,int> PrevLaneChangeActiveGapIdx;
	BehaviorGapUtils::GapObject PrevLaneChangeActiveGapObj;
	lane_change_feedback_enum traj_fb = behaviorinput::_trajectoryplan.lane_change_feedback;

	AdjacentLaneBehaviorDesired.speedTarget =  TargetState.step.posted_speed_lim_mps;  //SA.HVData.Aux_lanesituation[ADJACENT_TARGET_LANE_SITUATION].avg_speed_limit_mps[0];
	AdjacentLaneBehaviorDesired.speedTargetDistance = -999;

	BU::codeGreen( _CF_ +"lane change at lane ", vehicle.curSegmentUid);
	GapType TargetAdjacentGapVel,TargetAdjacentGapAccel;

	TargetLane_Ahead = BehaviorGapUtils::LaneGapObjVec(Center_Ahead,TARGET_LANE_SITUATION,LaneStaticGapPredFunc,1);
	TargetLane_Behind = BehaviorGapUtils::LaneGapObjVec(Center_Ahead,TARGET_LANE_SITUATION,LaneStaticGapPredFunc,-1);


	// pairs of range (min,max)
	int Adjaheadhypidx,Adjbehindhypidx;
	int AdjacentAheadID = SA.ahead_obj_id(TARGET_LANE_SITUATION,0,&garbage,&Adjaheadhypidx);
	int AdjacentBehindID = SA.behind_obj_id(TARGET_LANE_SITUATION,0,&garbage,&Adjbehindhypidx);
	GapObjUidType TargetAdjacentObjIDs = std::make_pair(AdjacentBehindID,AdjacentAheadID);
	BehaviorGapUtils::AdjustGapOffset(TargetAdjacentObjIDs,offset);
	GapType TargetAdjacentGap = std::make_pair(SA.clear_dist_behind(TARGET_LANE_SITUATION), SA.clear_dist_ahead(TARGET_LANE_SITUATION) )+offset;
	BehaviorGapUtils::ExtractGapParam(TargetAdjacentObjIDs,TargetAdjacentGapVel,TargetAdjacentGapAccel);
	GapIntType Adjhypidx = std::make_pair(Adjbehindhypidx,Adjaheadhypidx);


	bool IsAdjBhndObjYieldingToHV = SA.IsObjYieldingToHV(AdjacentBehindID,Adjbehindhypidx);


	TargetLane_Adjacent = BehaviorGapUtils::GapObject(TargetAdjacentGap,TargetAdjacentGapVel,TargetAdjacentGapAccel,TargetAdjacentObjIDs,TARGET_LANE_SITUATION, Adjhypidx,Center_Ahead,LaneStaticGapPredFunc);
	if(IsAdjBhndObjYieldingToHV)
		TargetLane_Adjacent.GapCost-=0.5;



	auto MinGapIdx = [] (BehaviorGapUtils::GapObjectVec Gaps) -> int {
		int Idx = 0;
		for (int i =0; i< Gaps.size(); i++){
			if(Gaps[i].GapCost<Gaps[Idx].GapCost)
				Idx = i;
		}
		return  Idx;
	}; // lambda here
	int MyMinGapIdx;


	double MinGapCost = std::min({  TargetLane_Behind[MinGapIdx(TargetLane_Behind)].GapCost  ,  TargetLane_Adjacent.GapCost  ,  TargetLane_Ahead[MinGapIdx(TargetLane_Ahead)].GapCost  });

	bool NonAbrtLCX = (traj_fb!=ABORTED)&&(MinGapCost<MIN_GAP_COST_THRESH);
	bool AbrtLCX = (traj_fb==ABORTED)&&(MinGapCost<ABRT_MIN_GAP_COST_THRESH);
	bool traj_committed = TrajCommitted;

//	cout<<" traj_committed "<<traj_committed<<" TARGET_LANE_SITUATION "<<TARGET_LANE_SITUATION<<" traj_fb "<<traj_fb<<endl;

	static bool prev_commit;



	if(traj_committed &&(PrevLaneChangeActiveGapIdx.find(TARGET_LANE_SITUATION)!=PrevLaneChangeActiveGapIdx.end())){ // At this point we have an agreement with trajectory about which gap we should be targetting.
		SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IN_PROGRESS);
		if(PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] == 0){
			Current_Lane_Behavior<<TargetLane_Adjacent; // Assign this gap to Current Behavior
			Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
			PrevLaneChangeActiveGapObj = TargetLane_Adjacent;
			AdjacentLaneBehaviorDesired = GetSimpleLaneBehavior(TargetState,{TARGET_LANE_SITUATION,0});
		}
		else if(PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] > 0){
			MyMinGapIdx = PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION]-1;
			if(MyMinGapIdx>=TargetLane_Ahead.size())
				goto OPTIMAL_GAP_SEARCH;
			Current_Lane_Behavior<<TargetLane_Ahead[MyMinGapIdx];
			Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
			PrevLaneChangeActiveGapObj = TargetLane_Ahead[MyMinGapIdx];
			AdjacentLaneBehaviorDesired = GetSimpleLaneBehavior(TargetState,{TARGET_LANE_SITUATION,PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION]});


		}
		else if(PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] < 0 ){
			MyMinGapIdx = -PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION]-1;
			if(MyMinGapIdx>=TargetLane_Behind.size())
				goto OPTIMAL_GAP_SEARCH;
			Current_Lane_Behavior<<TargetLane_Behind[MyMinGapIdx];
			Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
			PrevLaneChangeActiveGapObj = TargetLane_Behind[MyMinGapIdx];
		}
		MinGapCost = PrevLaneChangeActiveGapObj.GapCost;
		NonAbrtLCX = (traj_fb!=ABORTED)&&(MinGapCost<MIN_GAP_COST_THRESH);
	} // END if traj_fb==COMMITED
	else { // traj_fb!=COMMITED
		OPTIMAL_GAP_SEARCH:
		{
			if(MinGapCost==TargetLane_Adjacent.GapCost){
				PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] = 0;
				Current_Lane_Behavior<<TargetLane_Adjacent;
				Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
				PrevLaneChangeActiveGapObj = TargetLane_Adjacent;
				AdjacentLaneBehaviorDesired = GetSimpleLaneBehavior(TargetState,{TARGET_LANE_SITUATION,0});

			}
			else if(MinGapCost== TargetLane_Ahead[MinGapIdx(TargetLane_Ahead)].GapCost){
				MyMinGapIdx = MinGapIdx(TargetLane_Ahead);
				PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] = MyMinGapIdx+1;
				Current_Lane_Behavior<<TargetLane_Ahead[MyMinGapIdx];
				Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
				PrevLaneChangeActiveGapObj = TargetLane_Ahead[MyMinGapIdx];
				AdjacentLaneBehaviorDesired = GetSimpleLaneBehavior(TargetState,{TARGET_LANE_SITUATION,PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION]});
			}
			else if(MinGapCost==TargetLane_Behind[MinGapIdx(TargetLane_Behind)].GapCost){
				MyMinGapIdx = MinGapIdx(TargetLane_Behind);
				PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] = -(1+MyMinGapIdx);
				Current_Lane_Behavior<<TargetLane_Behind[MyMinGapIdx];
				PrevLaneChangeActiveGapObj = TargetLane_Behind[MyMinGapIdx];
				Current_Lane_Behavior<Center_Ahead; // Always Assign center ahead gap to Current Behavior
			}


		}

	}  // END Else traj_fb==COMMITED




	BU::codeYellow("PrevLaneChangeActiveGapObj",PrevLaneChangeActiveGapObj,0.25);


	if(NonAbrtLCX||AbrtLCX){
		if((Current_Lane_Behavior.leftLaneChangeIntention == LANE_CHANGE_IN_PROGRESS)||(Current_Lane_Behavior.rightLaneChangeIntention == LANE_CHANGE_IN_PROGRESS)){
			// Do Nothing.
		}
		else if (env_s0>0)
			SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_NOT_DESIRED);
		else if (env_s1<LCX_TM_LEFT_FOR_URGENT_ZONE_s*SpdRef)
			SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_URGENT);
		else if(env_s1<LCX_TM_LEFT_FOR_RECOMMENDED_ZONE_s*SpdRef)
			SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_RECOMMENDED);
		else
			SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_DESIRED);

	}
	else{
		PrevLaneChangeActiveGapIdx.erase(TARGET_LANE_SITUATION);
		SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_NOT_DESIRED);
		Current_Lane_Behavior.EmptyObj();
	}

	if(traj_committed)
		SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IN_PROGRESS);

	BU::codeBlue(_CF_ + " {NonAbrtLCX, AbrtLCX} ",std::make_pair(NonAbrtLCX,AbrtLCX),1);
	//	BU::codeBlue(_CF_ + " Current_Lane_Behavior",Current_Lane_Behavior,1);


	if(Current_Lane_Behavior.TgtlaneChangeZoneStart_m>0)  // Ahead gaps can be more desirable and we can target them, but we will allow trajectory to proceed ONLY once we have reached that gap.
		SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_NOT_DESIRED);

	//BU::codePurple( _CF_ + " Current_Lane_Behavior ",Current_Lane_Behavior,0.5);

	bool TriggerForGapCreation = ((Current_Lane_Behavior.TgtlaneChangeZoneStart_m > 0)|| (MinGapCost>MIN_GAP_CREATION_COST_THRESH)||(traj_fb == REJECTED )||(env_s1<GAP_CREATION_DIST_THRSH));

	std::map<double,double> GAP_CREATION_HV_SPD_THRSH_FACTOR_TBL = { {0.3, 0.0} , {0.3, 10.0}, {0.4, 20}, {0.5, 30}, {0.6, 40}, {0.8, 70} ,{1.0, 100.0}, {1.0, 120.0}   };
	// input - dist to end LCZ. output ratio of the max speeed
	double GAP_CREATION_HV_SPD_LMT = BehaviorUtils::LookUp1DMap(GAP_CREATION_HV_SPD_THRSH_FACTOR_TBL,env_s1) * SpdLmt;

	BehaviorTertiary::Maneuver_Temporal CreateGapManueverTemporal;
	auto AdjacentLaneBehaviorDesired1 = AdjacentLaneBehaviorDesired;
	if((env_s1>LCX_NOT_DESIRED_ZONE)&& (env_s1>GAP_CREATION_NOT_DESIRED_DIST_THRSH) && (vehicle.curSpeed_mps > GAP_CREATION_HV_SPD_LMT) &&(env_s1<GAP_CREATION_DIST_THRSH) && TriggerForGapCreation){
		BehaviorGapUtils::GapObjectVec GapCreationObjVec;
		BehaviorGapUtils::GapObject GapCreationObj;
		double TargetDist = -999;
		double TargetSpd = -1;
		double MinGapCreationGapCost = -1000;
		if(PrevLaneChangeActiveGapIdx.find(TARGET_LANE_SITUATION)!=PrevLaneChangeActiveGapIdx.end()){
			if(PrevLaneChangeActiveGapIdx[TARGET_LANE_SITUATION] >=0){ // We might have to create gap at this point (if we are targeting front gap at this point)
				BehaviorGapUtils::GapObject FakeGap;
				CreateGapManueverTemporal = CreateGap(Current_Lane_Behavior,FakeGap /*Center_Ahead*/ , TargetLane_Adjacent,	TARGET_LANE_SITUATION,LaneStaticGapPredFunc);
				//BehaviorTertiary::Maneuver_Temporal AdjacentLaneManueverTemporal;
				//AdjacentLaneManueverTemporal = AdjacentLaneBehaviorDesired;
				AdjacentLaneBehaviorDesired = AdjacentLaneBehaviorDesired + CreateGapManueverTemporal;
				//Current_Lane_Behavior = Current_Lane_Behavior + AdjacentLaneManueverTemporal;

			}
		}

	}


	FilterSmallLCZ(Current_Lane_Behavior);


	if((LaneChangeIntentionOvrrd>0)&&(Current_Lane_Behavior.laneChangeZoneEnd_m<OPPORTUNISTIC_LCX_URGENCY_OVRD_DIST_THRSH)){
		if((Current_Lane_Behavior.leftLaneChangeIntention==LANE_CHANGE_IS_DESIRED)||(Current_Lane_Behavior.leftLaneChangeIntention==LANE_CHANGE_IS_RECOMMENDED)\
				||(Current_Lane_Behavior.leftLaneChangeIntention==LANE_CHANGE_IS_URGENT)||(Current_Lane_Behavior.leftLaneChangeIntention==LANE_CHANGE_IN_PROGRESS)){
			if(Current_Lane_Behavior.leftLaneChangeIntention!=LANE_CHANGE_IN_PROGRESS)
				Current_Lane_Behavior.leftLaneChangeIntention = LaneChangeIntentionOvrrd;
		}
		if((Current_Lane_Behavior.rightLaneChangeIntention==LANE_CHANGE_IS_DESIRED)||(Current_Lane_Behavior.rightLaneChangeIntention==LANE_CHANGE_IS_RECOMMENDED)\
				||(Current_Lane_Behavior.rightLaneChangeIntention==LANE_CHANGE_IS_URGENT)||(Current_Lane_Behavior.rightLaneChangeIntention==LANE_CHANGE_IN_PROGRESS)){
			if(Current_Lane_Behavior.rightLaneChangeIntention!=LANE_CHANGE_IN_PROGRESS)
				Current_Lane_Behavior.rightLaneChangeIntention = LaneChangeIntentionOvrrd;
		}
	}

	if(Current_Lane_Behavior.leftLaneChangeIntention==LANE_CHANGE_IN_PROGRESS){
		if(BU::find(TargetState.TargetGrpLanes,vehicle.curSegmentUid)){
			Current_Lane_Behavior.leftLaneChangeIntention = LANE_CHANGE_NEAR_COMPLETION;
		}
	}
	else if(Current_Lane_Behavior.rightLaneChangeIntention==LANE_CHANGE_IN_PROGRESS){
		if(BU::find(TargetState.TargetGrpLanes,vehicle.curSegmentUid)){
			Current_Lane_Behavior.rightLaneChangeIntention = LANE_CHANGE_NEAR_COMPLETION;
		}
	}



	Current_Lane_Behavior.TargetLaneChangeGapPred.gap_count = 0;
	Current_Lane_Behavior.CurrentLaneChangeGapPred.gap_count = 0;
	if(Current_Lane_Behavior.laneChangeZoneEnd_m!=0){
		for(double Tm=0;Tm<=7;Tm+=0.5){
			Current_Lane_Behavior.TargetLaneChangeGapPred.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)Tm,PrevLaneChangeActiveGapObj));
			Current_Lane_Behavior.CurrentLaneChangeGapPred.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)Tm,Center_Ahead));
		}
		Current_Lane_Behavior.TargetLaneChangeGapPred.gap_count = Current_Lane_Behavior.TargetLaneChangeGapPred.LaneGaps.size();
		Current_Lane_Behavior.CurrentLaneChangeGapPred.gap_count = Current_Lane_Behavior.CurrentLaneChangeGapPred.LaneGaps.size();
	}

	Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.clear();
	Current_Lane_Behavior.AllCurrentLaneGaps.LaneGaps.clear();
	for(int i=0;i<TargetLane_Behind.size();i++)
		Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)0,TargetLane_Behind[i]));
	std::reverse(Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.begin(), Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.end());
	Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)0,TargetLane_Adjacent));
	for(int i=0;i<TargetLane_Ahead.size();i++)
		Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)0,TargetLane_Ahead[i]));
	Current_Lane_Behavior.AllCurrentLaneGaps.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)0,Center_Behind));
	Current_Lane_Behavior.AllCurrentLaneGaps.LaneGaps.push_back(BehaviorGapUtils::construct_behaviorgapviz((float)0,Center_Ahead));
	Current_Lane_Behavior.AllCurrentLaneGaps.gap_count = Current_Lane_Behavior.AllCurrentLaneGaps.LaneGaps.size();
	Current_Lane_Behavior.AllTargetLaneGaps.gap_count = Current_Lane_Behavior.AllTargetLaneGaps.LaneGaps.size();

	bool traj_committed_lcx_not_desired = (env_s1<LCX_NOT_COMMITTED_NOT_DESIRED_ZONE)&&(traj_committed);
	bool traj_not_committed_lcx_not_desired= (env_s1<LCX_NOT_DESIRED_ZONE)&&(!traj_committed);


	if(traj_committed_lcx_not_desired||traj_not_committed_lcx_not_desired){
		SetCorrectLaneChangeIntention(TARGET_LANE_SITUATION,Current_Lane_Behavior,LANE_CHANGE_IS_NOT_DESIRED);
		Current_Lane_Behavior.reRouteNeeded = STRAIGHT_ROUTE;
		Current_Lane_Behavior.alternaterouteNeeded.insert(STRAIGHT_ROUTE);
		BU::codeBlue( _CF_ +" SECONDARY_ROUTE plan Requested ",Current_Lane_Behavior.reRouteNeeded,0.1);
	}

	prev_commit = (traj_committed);




	if(SIMFREEZEONSET){
		cout<<"MinGapCost "<<MinGapCost<<endl;
		//cout<<" LeftAdjacentGapCost "<<TargetLane_Adjacent.GapCost<<" LeftAheadGapCost "<<LeftAheadGapCost<<" LeftBehindGapCost "<<LeftBehindGapCost<<endl;
		cout<<"TargetLane_Adjacent "<<TargetLane_Adjacent<<endl;
		//	cout<<" TargetLane_Adjacent IDEAL_ZONE_OVERLAP "<<(TargetLane_Adjacent.GapOverlapPredFunc & IDEAL_LC_ZONE)<<endl;
		//	cout<<" PrevLaneChangeActiveGapObj IDEAL_ZONE_OVERLAP "<<(PrevLaneChangeActiveGapObj.GapOverlapPredFunc & IDEAL_LC_ZONE)<<endl;
		//cout<< " OVERLAP Fraction "<<GapOverLapFraction(IDEAL_LC_ZONE,TargetLane_Adjacent.GapOverlapPredFunc)<<endl;
		//cout<< " OVERLAP Fraction "<<GapOverLapFraction(IDEAL_LC_ZONE,TargetLane_Adjacent.GapOverlapPredFunc)<<endl;
		cout<<" LaneSnsrRngGapPredFunc "<<LaneSnsrRngGapPredFunc<<endl;
		cout<<"TargetLane_Ahead "<<TargetLane_Ahead<<endl;
		cout<<"TargetLane_Behind "<<TargetLane_Behind<<endl;
		cout<<"center_ahead_gap "<<Center_Ahead<<endl;
		cout<<"center_behind_gap "<<Center_Behind<<endl;
		cout<<" traj_fb ";
		lcmprint::PrintEnum(cout,traj_fb);
		cout<<" Current_Lane_Behavior.leftLaneChangeIntention ";
		lcmprint::PrintEnum(cout,(lane_intention_enum)Current_Lane_Behavior.leftLaneChangeIntention) ;
		cout<<" Current_Lane_Behavior.rightLaneChangeIntention ";
		cout<<"LaneChangeIntentionOvrrd "<<LaneChangeIntentionOvrrd<<endl;
		lcmprint::PrintEnum(cout,(lane_intention_enum)Current_Lane_Behavior.rightLaneChangeIntention) ;
		cout<<" Current_Lane_Behavior.laneChangeZoneStart_m "<<\
				Current_Lane_Behavior.laneChangeZoneStart_m<<" Current_Lane_Behavior.laneChangeZoneEnd_m "<<Current_Lane_Behavior.laneChangeZoneEnd_m<<endl;
		cout<<" Current_Lane_Behavior.TgtlaneChangeZoneStart_m "<<\
				Current_Lane_Behavior.TgtlaneChangeZoneStart_m<<" Current_Lane_Behavior.TgtlaneChangeZoneEnd_m "<<Current_Lane_Behavior.TgtlaneChangeZoneEnd_m<<endl;
		cout<<" PrevLaneChangeActiveGapIdx "<<PrevLaneChangeActiveGapIdx<<" env_s "<<env_s1<< endl;
		//	SA.PrintSituation();
		//cout<<purple_on<<" ---------------------Current_Lane_Behavior--------------"<<endl<<Current_Lane_Behavior<<endl;
	}

	//pthread_mutex_unlock(&Mutex::_lanechangeplot_mutex);


}
