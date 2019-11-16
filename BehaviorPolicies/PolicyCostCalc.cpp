#include "../include/BehaviorOptimalPolicy.hpp"



void Policy::DetailedPolicy::CalculateCost(){
	PolicyOptimalBehavior.SetBehaviorPolicyCost();
	this->Cost.BehaviorPolicyCost = PolicyOptimalBehavior.GetBehaviorPolicyCost();
	this->Cost.alternaterouteNeeded = BU::to_string(this->alternaterouteNeeded);
	this->Cost.routeCost = 0.0;
	if(Policy::ActiveDetailedPolicy.alternaterouteNeeded==this->alternaterouteNeeded)
		this->Cost.PolicyTransitionCost = 0;
	else
		this->Cost.PolicyTransitionCost = POLICY_TRANSITION_COSTS[this->rootLane];

	const std::map<double,double> WaitCostLookUp = 
	{ {0.0, 0.0} , {1.5, 0.0}, {2.5, 0.2}, {3.5, 0.5}, {5.0, 1.0} ,{7.5, 1.5}, {10.0, 2.5}, {20.0, 4.5} ,{60.0, 5.5} }; // { wait time , wait cost }

	this->TrafficJamTime = Policy::GetPolicyTrafficJamTime(this->PolicyOptimalBehavior,this->rootLane, this->TrafficJamObjID);
	this->Cost.TrafficEfficiencyCost = BehaviorUtils::LookUp1DMap(WaitCostLookUp,this->TrafficJamTime);
	if(this->alternaterouteNeeded==LANE_CHANGE_ROUTE_DLCX_RQRD){
		BehaviorTertiary::Behavior GarbageBehavior;
		Policy::GetPolicyTrafficJamTime(GarbageBehavior,HV_LANE_SITUATION, this->TrafficJamObjID); // root lane is left/right. But the impatience obj associated is with the HV lane
		if(SA.clear_dist_ahead_by_ID(HV_LANE_SITUATION,this->TrafficJamObjID)>OPPORTUNISTIC_LCX_DLCX_ALLWD_DIST_THRSH){
			cout<<" rootLane "<<this->rootLane<<" TrafficJamObjID "<<this->TrafficJamObjID<<endl;
			this->Cost.PolicyTransitionCost+=1000;
			BU::codeBlue(_CF_+" SA.clear_dist_ahead() too much for DLCX ",SA.clear_dist_ahead(HV_LANE_SITUATION,0),0.25);
			if(BU::codeBlue(_CF_+" SA.ahead ob ID  ",SA.ahead_obj_id(HV_LANE_SITUATION,0),0.25)){
		//		cout<<"HVData.Aux_lanesituation ";
		//		lcmprint::operator <<(cout,SA.HVData.Aux_lanesituation);
			}
		}
	}
	this->Cost.routeTime = this->PolicyDQ[this->Front].step.min_time_to_destination_s;
	//	for(auto &step:PolicyDQ)
	//		this->Cost.routeTime+=step.MinTravelTm;
	double ROUTE_TIME_SCALING_FACTOR =-1.0;
	if(ROUTE_TIME_SCALING_FACTOR>0){
		this->Cost.routeCost = this->Cost.routeTime/ROUTE_TIME_SCALING_FACTOR;
	}
	else{
		ROUTE_TIME_SCALING_FACTOR = this->Cost.routeTime;
		this->Cost.routeCost /=  this->Cost.routeTime;
	}
	this->Cost.TotalCost = Policy::CalculateCost(this->Cost);
	this->Cost.alternaterouteNeeded = BU::to_string(this->alternaterouteNeeded);
	this->Cost.segment_count = this->Policy.routeplan.segment_count;
	this->Cost.segment_uid = this->Policy.routeplan.segment_uid;
	this->Cost.segment_group = this->Policy.routeplan.segment_group;
	PolicyOptimalBehavior.SetBehaviorPolicyCost();
	this->Cost.BehaviorPolicyCost = PolicyOptimalBehavior.GetBehaviorPolicyCost();
}


double Policy::GetPolicyTrafficJamTime(const BehaviorTertiary::Behavior & BehaviorDesired, int LANE_SITUATION , ObjUidType& ImpatienceObj ){
	if(!ENABLE_OPTIMAL_POLICY_SEARCH)
		return 0;

	double WaitCost = 0;
	const double ShowTmThrsh = 1;

	double Timenow = BehaviorUtils::TimeNow();


	static map<int,ObjUidType> impatience_obj_ID= \
			{{HV_LANE_SITUATION,-1},{STRIAGHT_LANE_SITUATION,-1},{LEFT_ADJACENT_LANE_SITUATION,-1},{RIGHT_ADJACENT_LANE_SITUATION,-1},{LEFT_OPPOSING_LANE_SITUATION,-1},{RIGHT_OPPOSING_LANE_SITUATION,-1}, \
		{OPPOSING_SHARED_LANE_SITUATION,-1}};
	static map<int,LaneSegUidType> impatience_Origin_Grp = \
			{{HV_LANE_SITUATION,-1},{STRIAGHT_LANE_SITUATION,-1},{LEFT_ADJACENT_LANE_SITUATION,-1},{RIGHT_ADJACENT_LANE_SITUATION,-1},{LEFT_OPPOSING_LANE_SITUATION,-1},\
		{RIGHT_OPPOSING_LANE_SITUATION,-1}, {OPPOSING_SHARED_LANE_SITUATION,-1}};
	static map<int,double> last_tracked_obj_time = \
			{{HV_LANE_SITUATION,Timenow},{STRIAGHT_LANE_SITUATION,Timenow},{LEFT_ADJACENT_LANE_SITUATION,Timenow},{RIGHT_ADJACENT_LANE_SITUATION,Timenow},{LEFT_OPPOSING_LANE_SITUATION,Timenow},\
		{RIGHT_OPPOSING_LANE_SITUATION,Timenow}, {OPPOSING_SHARED_LANE_SITUATION,Timenow}};
	static map<int,double> impatience_time = last_tracked_obj_time;
	static map<int,exlcm::objprediction_t> impatience_hypotheses;

	int LaneObjIdx,hypothesisIdx;
	int ObjIdx = 0;
	ObjUidType Ahead_obj_ID;
	exlcm::objsituation_t objSA ;
	do{
		Ahead_obj_ID = SA.ahead_obj_id(LANE_SITUATION,ObjIdx,&LaneObjIdx,&hypothesisIdx);
		ObjIdx++;

		if(SA.objData.find(Ahead_obj_ID)!=SA.objData.end()){
			objSA =  SA.objData[Ahead_obj_ID];

			if ((objSA.objtype==VEHICLE_OBSTACLE_TYPE)||(objSA.objtype==TRUCK_OBSTACLE_TYPE)) // There could be pedestrian in front of a vehicle
				break;
		}
	}
	while(Ahead_obj_ID>0);


	exlcm::objsituation_t ImpatienceObjSA;
	exlcm::objprediction_t ImpatienceObjPred;
	bool IsImpatienceObjStillValid = false;

	if(impatience_obj_ID[LANE_SITUATION]>0){
		if(BU::find(impatience_obj_ID,LANE_SITUATION)){
			ImpatienceObjSA = SA.objData[impatience_obj_ID[LANE_SITUATION]];
			for(auto pred:ImpatienceObjSA.pred_obj_SA){
				if((pred.relative_location == SA_AHEAD|| pred.relative_location ==  SA_BEHIND)&&(pred.in_lane_speed_mps <OPPORTUNISTIC_LCX_IMPATIENCE_SPD_THRESHOLD)){
					ImpatienceObjPred = pred;
					IsImpatienceObjStillValid = true;
					break;
				}
			}
		}
	}

	ImpatienceObj=impatience_obj_ID[LANE_SITUATION];

	if(SA.objData.find(Ahead_obj_ID)==SA.objData.end()){
		BU::codeRed(_CF_ +" something is really wrong ... Could not find ahead object in the SA object list. Ahead_obj_ID ",Ahead_obj_ID,ShowTmThrsh);
		return 0;
	}


	if (!((objSA.objtype==VEHICLE_OBSTACLE_TYPE)||(objSA.objtype==TRUCK_OBSTACLE_TYPE)))
		return 0;

	if(Ahead_obj_ID<=0)
		return 0;


	if((hypothesisIdx>=objSA.valid_pred_count)||(hypothesisIdx<0)||(objSA.valid_pred_count<=0)){
		BU::codeRed(_CF_ +" something is really wrong ... { hypothesisIdx ,valid_pred_count } = ",std::make_pair(hypothesisIdx,objSA.valid_pred_count),ShowTmThrsh);
		return 0;
	}

	exlcm::objprediction_t objPred = objSA.pred_obj_SA[hypothesisIdx];

	if((objPred.relative_location == SA_ONCOMING_LEFT)||(objPred.relative_location == SA_ONCOMING_FAR_LEFT)){
		BU::codeRed(_CF_ +" something is  wrong ... relative_location == SA_ONCOMING_LEFT || SA_ONCOMING_FAR_LEFT ",Ahead_obj_ID,ShowTmThrsh);
		return 0;
	}

	if((objPred.relative_location == SA_ONCOMING)&&(objPred.Stationary_Status==STATIONARY_STATUS_MOVING)){ // Really dont want to do this, Should be for all stationary status. But looks like fusion is flaky
		BU::codeRed(_CF_ +" something is  wrong ... relative_location == SA_ONCOMING && Stationary_Status == STATIONARY_STATUS_MOVING ",Ahead_obj_ID,ShowTmThrsh);
		return 0;
	}


	double Ahead_obj_Distance = SA.clear_dist_ahead(LANE_SITUATION);



	int StepIdx = BehaviorDesired.TargetState.step.step_index_n;
	std::string CurrGrpStr = "<StepIdx:"+std::to_string(StepIdx)+"::LANE:"+std::to_string(LANE_SITUATION)+">";
	std::string objIDStr;
	objIDStr = CurrGrpStr+"<ID:"+std::to_string(impatience_obj_ID[LANE_SITUATION])+">";




	if(Ahead_obj_ID>0){
		last_tracked_obj_time[LANE_SITUATION] = Timenow;
		double Ahead_obj_vel = objPred.in_lane_speed_mps;
		int Ahead_Obj_Grp = BU::LaneSegmentToRouteSegmentGrp(objPred.lane_segment_uid,Policy::BackEndDetailedPolicy.Policy);
		bool IsAheadObjSlow = (Ahead_obj_vel<OPPORTUNISTIC_LCX_IMPATIENCE_SPD_THRESHOLD);
		bool IsHVSlow = (vehicle.curSpeed_mps<OPPORTUNISTIC_LCX_IMPATIENCE_HV_SPD_THRESHOLD);
		bool ISAheadObjSame = (impatience_obj_ID[LANE_SITUATION] !=-1) && (impatience_obj_ID[LANE_SITUATION]==Ahead_obj_ID) ;
		bool IsAheadObjNear = (Ahead_obj_Distance<OPPORTUNISTIC_LCX_IMPATIENCE_DIST_THRESHOLD);
		bool IsAheadObjInSameGrp = (impatience_Origin_Grp[LANE_SITUATION] !=-1)&&(Ahead_Obj_Grp==impatience_Origin_Grp[LANE_SITUATION]);
		int obj_intersection_uid = objPred.intersection_uid;
		bool IsAheadObjNotNearIntersection = (obj_intersection_uid<0); // Trying to make traffic cost 0 when RV is near intersection
		int stnry_stat = objPred.Stationary_Status;
		bool IsAheadObjNotParked = (stnry_stat!=STATIONARY_STATUS_PARKED_ROADSIDE)||(stnry_stat!=STATIONARY_STATUS_PARKED_PARKING_AISLE); // Trajectory will handle these
		IsAheadObjInSameGrp = IsAheadObjInSameGrp || (Ahead_Obj_Grp<=0); // Unfortunately SA is not working well all the time.

		objIDStr = CurrGrpStr+"<ID:"+std::to_string(impatience_obj_ID[LANE_SITUATION])+">";




		if(IsImpatienceObjStillValid){

		}
		else if(IsAheadObjSlow && IsHVSlow && ISAheadObjSame && IsAheadObjNear && IsAheadObjInSameGrp && IsAheadObjNotNearIntersection && IsAheadObjNotParked){
			//Do Nothing
		}
		else{


			if(!ISAheadObjSame)
				BU::codeYellow( _CF_ +" impatience_obj and Ahead Obj not equal. Ahead_obj_ID "+objIDStr,Ahead_obj_ID,ShowTmThrsh);

			if(!IsAheadObjInSameGrp)
				BU::codeYellow( _CF_ +" impatience_obj and Ahead Obj lane group not equal. {impatience_Origin_Grp,Ahead_Obj_Grp} "+objIDStr,std::make_pair(impatience_Origin_Grp[LANE_SITUATION],Ahead_Obj_Grp),ShowTmThrsh);

			if(!IsHVSlow)
				BU::codeYellow( _CF_ +" HV not slow enough for Opportunistic LCX.  curSpeed_mps "+objIDStr,vehicle.curSpeed_mps,ShowTmThrsh);

			if(!IsAheadObjSlow)
				BU::codeYellow( _CF_ +" Ahead Obj not slow enough for Opportunistic LCX. Ahead_obj_vel "+objIDStr,Ahead_obj_vel,ShowTmThrsh);

			if(!IsAheadObjNear)
				BU::codeYellow( _CF_ +" Ahead Obj not close enough for Opportunistic LCX. Ahead_obj_Distance "+objIDStr,Ahead_obj_Distance,ShowTmThrsh);

			if(!IsAheadObjNotNearIntersection)
				BU::codeYellow( _CF_ +" Ahead Obj near following intersection "+objIDStr,obj_intersection_uid,ShowTmThrsh);

			if(!IsAheadObjNotParked)
				BU::codeYellow( _CF_ +" Ahead Obj stationary status "+objIDStr,stnry_stat,ShowTmThrsh);

			if(Ahead_Obj_Grp>0)
				impatience_Origin_Grp[LANE_SITUATION] =Ahead_Obj_Grp;

			impatience_obj_ID[LANE_SITUATION] =Ahead_obj_ID;
			impatience_time[LANE_SITUATION] = Timenow;
			impatience_hypotheses[LANE_SITUATION]=objPred;

			return 0;

		}
	}
	else{ // Ahead_obj_ID = -1
		if((Timenow - last_tracked_obj_time[LANE_SITUATION])>10){
			impatience_obj_ID[LANE_SITUATION] =-1;
			impatience_Origin_Grp[LANE_SITUATION] = -1;
			impatience_time[LANE_SITUATION] = Timenow;
			BU::codeRed( _CF_  + " Ahead object NOT found for more than (s) " +objIDStr,(Timenow - last_tracked_obj_time[LANE_SITUATION]),ShowTmThrsh);
			return 0;
		}
	}

	bool IsImpatienceEnough = ((Timenow - impatience_time[LANE_SITUATION]) >OPPORTUNISTIC_LCX_IMPATIENCE_TM_THRESHOLD);
	if(!IsImpatienceEnough){
		BU::codeRed( _CF_ +" Ahead Obj impatience not enough for Opportunistic LCX. wait time(s) " + objIDStr,(Timenow - impatience_time[LANE_SITUATION]),ShowTmThrsh);
		return 0;
	}
	else{
		BU::codeGreen( _CF_ +" Object tracked  "+ objIDStr,impatience_obj_ID[LANE_SITUATION],ShowTmThrsh);
		BU::codeGreen( _CF_ +" wait time  "+objIDStr,(Timenow - impatience_time[LANE_SITUATION]),ShowTmThrsh);

	}

	if(Env::_ClosestInterSectionInfo.second<20){ // When HV is near an intersection don't use high traffic occupancy cost
		BU::codeYellow( _CF_ +" Too close intersection distance =  "+objIDStr,Env::_ClosestInterSectionInfo.second,ShowTmThrsh);
		return 0;
	}



//	cout<<yellow_on<<" rootLane "<<LANE_SITUATION<<" TrafficJamObjID "<<ImpatienceObj<<color_off<<endl;

	return (Timenow - impatience_time[LANE_SITUATION]);
}


