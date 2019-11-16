#include "../../include/BehaviorGapUtils.hpp"

double GAP_CREATION_DECEL_LIMIT_LOCAL;
double GAP_CREATION_ACCEL_LIMIT_LOCAL;

BehaviorTertiary::Maneuver_Temporal CreateGap(BehaviorTertiary::Behavior& Current_Lane_Behavior,const BehaviorGapUtils::GapObject& Ahead , \
		const BehaviorGapUtils::GapObject& Adjacent, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc){
	double TargetDist = -999;
	double TargetSpd = -1;
	int MyMinGapIdx = -1;
	double MinGapCreationGapCost;
	BehaviorGapUtils::GapObject Center_Ahead = Ahead;
	BehaviorGapUtils::GapObject GapCreationObj;
	BehaviorTertiary::Maneuver_Temporal My_Create_Gap_Maneuver;
	double env_s1 = Current_Lane_Behavior.TargetState.DAR_m - vehicle.DTAR;
	double GAP_CREATION_DIST_THRSH = LCX_TM_LEFT_FOR_GAP_CREATION_THRSH_s*Current_Lane_Behavior.TargetState.step.posted_speed_lim_mps;
	std::map<double,double> GAP_CREATION_DECEL_FACTOR_TBL = { {1.0, 0.0} , {1.0, 10.0}, {0.8, 20}, {0.7, 30}, {0.7, 40}, {0.6, 70} ,{0.5, 100.0}, {0.5, 120.0}   };
	GAP_CREATION_DECEL_LIMIT_LOCAL = BehaviorUtils::LookUp1DMap(GAP_CREATION_DECEL_FACTOR_TBL,GAP_CREATION_DIST_THRSH) * GAP_CREATION_DECEL_LIMIT;
	GAP_CREATION_ACCEL_LIMIT_LOCAL = GAP_CREATION_ACCEL_LIMIT;

	auto MinGapIdx = [] (std::vector<double> GapCosts) {
		return std::distance(std::begin(GapCosts),  std::min_element(std::begin(GapCosts), std::end(GapCosts)));
	}; // lambda here

	do{
		vehicle.PredAccel_mpss-=GAP_CREATION_DECEL_STEP;
		Center_Ahead.GapAccel = std::make_pair(0,Current_Lane_Behavior.AccelTarget-vehicle.PredAccel_mpss); // First element is zero
		Center_Ahead.UpdatePredFunc();
		BehaviorGapUtils::GapObjectVec GapCreationObjVec = LaneGapObjVec(Center_Ahead,LANE_SITUATION,LaneStaticGapPredFunc,-1);
		auto GapCreationGapCost = AssessGapCost(GapCreationObjVec);

		MyMinGapIdx = MinGapIdx(GapCreationGapCost);
		MinGapCreationGapCost = GapCreationGapCost[MyMinGapIdx];
		GapCreationObj = GapCreationObjVec[MyMinGapIdx];
		using namespace BehaviorGapUtils;
		if(GapCreationObj.GapObjIDs==-1){ // There is no object in the Behind gap
			GapCreationObj = GapObject(Adjacent,Center_Ahead,LaneStaticGapPredFunc);
			MinGapCreationGapCost = AssessGapCost(GapCreationObj); // Only one gap
		}
		if(MinGapCreationGapCost<MIN_GAP_CREATION_COST_THRESH){
			TargetDist = min(env_s1,GAP_CREATION_TGT_DIST);
			Current_Lane_Behavior<<GapCreationObj;
			TargetSpd = BU::targetSpeed(TargetDist,vehicle.curSpeed_mps,vehicle.PredAccel_mpss);
			if(std::isnan(TargetSpd)){ // Need to reduce the distance
				TargetSpd = 0.0;
				TargetDist = BU::targetDistance(vehicle.curSpeed_mps,TargetSpd,vehicle.PredAccel_mpss);
			}
			My_Create_Gap_Maneuver = BehaviorTertiary::Maneuver_Temporal(vehicle.PredAccel_mpss,TargetSpd,TargetDist,vehicle.PredAccel_mpss,CHANGE_LANES);
			Current_Lane_Behavior = Current_Lane_Behavior + My_Create_Gap_Maneuver;
			FilterSmallLCZ(Current_Lane_Behavior,LCX_GAP_FILTER_PASS_THRESH);
			break;
		}

	}while(vehicle.PredAccel_mpss>-GAP_CREATION_DECEL_LIMIT_LOCAL);


	if((vehicle.PredAccel_mpss<=-GAP_CREATION_DECEL_LIMIT_LOCAL)){
		vehicle.PredAccel_mpss = vehicle.curAccel_mpss ;
		do{

			vehicle.PredAccel_mpss+=GAP_CREATION_ACCEL_STEP;
			Center_Ahead.GapAccel = std::make_pair(0,Current_Lane_Behavior.AccelTarget-vehicle.PredAccel_mpss); // First element is zero
			Center_Ahead.UpdatePredFunc();
			BehaviorGapUtils::GapObjectVec GapCreationObjVec = LaneGapObjVec(Center_Ahead,LANE_SITUATION,LaneStaticGapPredFunc,1);
			auto GapCreationGapCost = AssessGapCost(GapCreationObjVec);

			MyMinGapIdx = MinGapIdx(GapCreationGapCost);
			MinGapCreationGapCost = GapCreationGapCost[MyMinGapIdx];
			GapCreationObj = GapCreationObjVec[MyMinGapIdx];
			using namespace BehaviorGapUtils;
			if(GapCreationObj.GapObjIDs==-1){ // There is no object in the  gap
				GapCreationObj = GapObject(Adjacent,Center_Ahead,LaneStaticGapPredFunc);
				MinGapCreationGapCost = AssessGapCost(GapCreationObj); // Only one gap
			}
			if(MinGapCreationGapCost<MIN_GAP_CREATION_COST_THRESH){
				TargetDist = min(env_s1,GAP_CREATION_TGT_DIST);
				Current_Lane_Behavior<<GapCreationObj;
				TargetSpd = BU::targetSpeed(TargetDist,vehicle.curSpeed_mps,vehicle.PredAccel_mpss);
				if(std::isnan(TargetSpd)){ // Need to reduce the distance
					TargetSpd = 0.0;
					TargetDist = BU::targetDistance(vehicle.curSpeed_mps,TargetSpd,vehicle.PredAccel_mpss);
				}
				My_Create_Gap_Maneuver = BehaviorTertiary::Maneuver_Temporal(vehicle.PredAccel_mpss,TargetSpd,TargetDist,vehicle.PredAccel_mpss,CHANGE_LANES);
				Current_Lane_Behavior = Current_Lane_Behavior + My_Create_Gap_Maneuver;
				FilterSmallLCZ(Current_Lane_Behavior,LCX_GAP_FILTER_PASS_THRESH);
				break;
			}

		}while((vehicle.PredAccel_mpss<GAP_CREATION_ACCEL_LIMIT_LOCAL)&&((vehicle.PredAccel_mpss<Current_Lane_Behavior.AccelTarget)));
	}

	//	if(vehicle.PredAccel_mpss<-GAP_CREATION_DECEL_LIMIT){
	/*	if(BU::codeRed( _CF_  + "GAP Creation failed. MinGapIdx = ",MyMinGapIdx,1)){
			cout<<" TargetDist "<<TargetDist<<" MinGapCreationGapCost " <<MinGapCreationGapCost<<endl;
			cout<<" Failed gap overlap "<<GapCreationObj.GapOverlapPredFunc<<endl;
		} */
	//	}

	/*	if(BU::codeBlue( _CF_ +"Trying to CREATE GAP.  "," ",1)){
		cout<<" vehicle.PredAccel_mpss "<<vehicle.PredAccel_mpss<<" vehicle.curSpeed_mps "<<vehicle.curSpeed_mps<<endl;
		cout<<" TargetDist "<<TargetDist<<" TargetSpd "<<TargetSpd<<" MinGapCreationGapCost " <<MinGapCreationGapCost<<endl;
		cout<<"center_ahead_gap : "<<Center_Ahead<<endl;
		cout<<" GapCreationObj : "<<GapCreationObj<<endl;
	}
	 */

	if(env_s1<GAP_CREATION_DIST_THRSH){ // Try to create gap, using a proportional control. This is required when we can't trust the output of fusion to use a more predictive model as we have used above.
		//Trying to slowdown the vehicle.
		if((Current_Lane_Behavior.laneChangeZoneStart_m>0)||(Current_Lane_Behavior.laneChangeZoneEnd_m <=0)|| (Adjacent.GapCost > MIN_GAP_CREATION_COST_THRESH)){
			TargetDist = min(env_s1,GAP_CREATION_TGT_DIST);
			double LCZ_StartDistFracOfDistRemain = Current_Lane_Behavior.laneChangeZoneStart_m/env_s1;
			vehicle.PredAccel_mpss = vehicle.curAccel_mpss ;
			vehicle.PredAccel_mpss -= LCZ_StartDistFracOfDistRemain*GAP_CREATION_DECEL_LIMIT_LOCAL;
			vehicle.PredAccel_mpss = max(vehicle.PredAccel_mpss,-GAP_CREATION_DECEL_LIMIT_LOCAL);
			BU::codeYellow( _CF_ +"Aggressive proportional gap creation at the tail end of lane change zone. distance remaining = ",env_s1,1);
			BU::codeYellow( _CF_ +"Aggressive proportional gap creation. vehicle.PredAccel_mpss = ",vehicle.PredAccel_mpss,1);
			TargetSpd = BU::targetSpeed(TargetDist,vehicle.curSpeed_mps,vehicle.PredAccel_mpss);
			if(std::isnan(TargetSpd)){ // Need to reduce the distance
				TargetSpd = 0.0;
				TargetDist = BU::targetDistance(vehicle.curSpeed_mps,TargetSpd,vehicle.PredAccel_mpss);
			}
			My_Create_Gap_Maneuver = BehaviorTertiary::Maneuver_Temporal(vehicle.PredAccel_mpss,TargetSpd,TargetDist,vehicle.PredAccel_mpss,CHANGE_LANES);
			Current_Lane_Behavior = Current_Lane_Behavior + My_Create_Gap_Maneuver;
			FilterSmallLCZ(Current_Lane_Behavior,LCX_GAP_FILTER_PASS_THRESH);
		}
	}



	return My_Create_Gap_Maneuver;
}

