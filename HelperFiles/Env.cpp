#include "../include/Env.hpp"
#include "../include/BehaviorUtils.hpp"
#include "../include/io_handler.hpp"
#include "../include/lcmprint.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"



namespace BU=BehaviorUtils;

struct  vehicle;

std::pair<InterSectionUidType, double> Env::_ClosestInterSectionInfo;
double Env::MaxDTAR = 0.0;
bool Env::initDone = false;
Vehicle::vehicleState Env::MaxDTAR_VehicleState;

extern Policy::DetailedPolicy Policy::ActiveDetailedPolicy,Policy::BackEndDetailedPolicy;


double Env::LaneSegOriginDAR(LaneSegUidType LaneSegUID, const Policy::DetailedPolicy &MyDetailedPolicy){

	if(LaneSegUID<0)
		BU::codeRed( _CF_  + " Inside LaneSegOriginDAR possible error as sought LaneSegUID is ",LaneSegUID,1);

	int GrpID = BU::LaneSegmentToRouteSegmentGrp(LaneSegUID,MyDetailedPolicy.Policy);
	double GrpOriginDAR;
	if(BU::find(MyDetailedPolicy.SegGrpOriginDAR,GrpID)){
		GrpOriginDAR = MyDetailedPolicy.SegGrpOriginDAR[GrpID];
	}
	else{
		map<int,double> AllGrpSegOriginDAR;
		Env::SegGrpOriginDAR(MyDetailedPolicy.Policy.routeplan.segment_group.back(),MyDetailedPolicy.Policy,AllGrpSegOriginDAR,0);
		GrpOriginDAR = AllGrpSegOriginDAR[GrpID];
	}

	auto AllLaneSeginGrp = BU::routeSegmentGrpTolanesegments(GrpID,MyDetailedPolicy.Policy);
	int CurrLaneSegIdx = 0;
	double SegOriginDAR = GrpOriginDAR;
	while(AllLaneSeginGrp[CurrLaneSegIdx]!=LaneSegUID){
		SegOriginDAR += BU::getSegLen(AllLaneSeginGrp[CurrLaneSegIdx]);
		SegOriginDAR += BU::getConnLen(AllLaneSeginGrp[CurrLaneSegIdx],AllLaneSeginGrp[CurrLaneSegIdx+1]);
		CurrLaneSegIdx++;
	}
	return SegOriginDAR;
}

void Env::initStates(Policy::DetailedPolicy &MyDetailedPolicy, bool ReInitRequest, double DAR_m_Origin, bool ActivePolicy )
{

	BehaviorUtils::codeBlue( _CF_ +"Initializing states...");
	double prev_valid_Station_m = 0;
	// Build the states queue.
	double TargetDistance, Prev_TargetDistance;
	double DAR_m = DAR_m_Origin;


	//	Env::SegOriginDAR(Env::Sortedlcmin::routeSegmentList.back());
	MyDetailedPolicy.SegGrpOriginDAR.clear();
	Env::SegGrpOriginDAR(MyDetailedPolicy.Policy.routeplan.segment_group.back(),MyDetailedPolicy.Policy,MyDetailedPolicy.SegGrpOriginDAR,DAR_m_Origin);
	std::string routestr = "<route uid:"+std::to_string(MyDetailedPolicy.Policy.uid)+">";
	//BU::codeYellow(_CF_ + " DAR_m_Origin ",DAR_m_Origin,-1);
	if(ActivePolicy)
		BU::codeYellow(_CF_ + " MyDetailedPolicy.SegGrpOriginDAR "+routestr,MyDetailedPolicy.SegGrpOriginDAR,1);



	//Env::PrintSortedDAR();

	for(int i=0;i<MyDetailedPolicy.Policy.pathPlan.included_step_count;i++)
	{//TODO: Handle the case to catch cases when DAR_m isnt't updated at least once in every iter
		exlcm::pathstep_t pathStep=MyDetailedPolicy.Policy.pathPlan.path_steps[i];

		if(pathStep.current_segment_station_m==0)
			pathStep.current_segment_station_m = 0.001;

		if(pathStep.step_rule==YIELD_UNTIL_CREEP_DISTANCE_REACHED){
			pathStep.current_segment_station_m = -0.1;
			//pathStep.step_rule = ASSUME_RIGHT_OF_WAY_RULE;
		}

		Env::PolicyState state;
		//state.type=Env::STEP;
		//	state.current_segment_station_lcmin::pathPlan_m = pathStep.current_segment_station_m;

		TargetDistance = Env::CalculateTargetDistanceAlongSeg(pathStep,MyDetailedPolicy.Policy);



		if(!MyDetailedPolicy.PolicyDQ.empty()){
			Prev_TargetDistance = Env::CalculateTargetDistanceAlongSeg(MyDetailedPolicy.PolicyDQ.back().step,MyDetailedPolicy.Policy);
		}

	//	if(pathStep.step_rule==OBEY_POSTED_SIGN_RULE)
	//		state.observed_step_rule = OBEY_POSTED_SIGN_RULE;

		if(pathStep.current_segment_station_m>0)
		{
			bool IsThisALaneChangeCompleteTask = (pathStep.step_task==PROCEED_AFTER_LANE_CHANGE_TASK);

			if(IsThisALaneChangeCompleteTask){
				MyDetailedPolicy.PolicyDQ.back().step.step_rule = CHANGE_LANES_RULE;
				pathStep.target_segment_station_m = pathStep.current_segment_station_m  + MyDetailedPolicy.PolicyDQ.back().step.target_segment_station_m -MyDetailedPolicy.PolicyDQ.back().step.current_segment_station_m;
			}


			//If the previous step was in the same lane seg
			if(i>=1){

				if(!MyDetailedPolicy.PolicyDQ.empty()){
					if(pathStep.current_segment_group==MyDetailedPolicy.PolicyDQ.back().step.current_segment_group){ // previous step is same as current step segment
						auto delta_dist = (TargetDistance-Prev_TargetDistance);//behaviorinput::pathPlan.path_steps[i-1].current_segment_station_m);
						if ((delta_dist<0)&& (!IsThisALaneChangeCompleteTask)){
							std::string stepstr = std::to_string(pathStep.step_index_n);
							stepstr = "<step_index:"+stepstr+">";
							const double TimeGap = 1;
							BU::codeRed(_CF_+ "delta_dist -ve. Something might be wrong.... {TargetDistance, Prev_TargetDistance}  = " + stepstr, std::make_pair(TargetDistance,Prev_TargetDistance),TimeGap);
							BU::codePurple(_CF_+ "behaviorinput::pathPlan previous step " + stepstr, MyDetailedPolicy.Policy.pathPlan.path_steps[i-1],TimeGap);
							BU::codeBlue(_CF_+ " ------------------------------------ "+ stepstr," ",TimeGap);
							BU::codePurple(_CF_+  "behaviorinput::pathPlan step " + stepstr, pathStep,TimeGap);
							BU::codeBlue(_CF_+  " ++++++++++++++++++++++++++++++++++++ "+ stepstr," ",TimeGap);
						}
						state.DAR_m=DAR_m+delta_dist;
						if((ActivePolicy)&&(pathStep.step_task==CHANGE_LANES_TASK))
							cout<<yellow_on<<" pathStep.current_segment_group "<<pathStep.current_segment_group<<" TargetDistance "<<TargetDistance<<" MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] "<<\
							MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group]<<" delta_dist "<<delta_dist<<color_off<<endl;
						//Add the incremental dist from prev station on the same seg
					}
					else//If this is the only station on the segment (so far)
					{
						//state.DAR_m=DAR_m+TargetDistance;
						state.DAR_m = MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] + TargetDistance;
					}
				}
				else//Policy deque is empty
				{
					//state.DAR_m=DAR_m+TargetDistance;
					state.DAR_m = MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] + TargetDistance;
				}
			}
			else
			{
				//state.DAR_m=DAR_m+TargetDistance;
				state.DAR_m = MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] + TargetDistance;
			}



			if(IsThisALaneChangeCompleteTask) // Not double lane change
				state.DAR_m = DAR_m;
			else
				DAR_m=state.DAR_m;
			//	prev_valid_Station_m = pathStep.current_segment_station_m;


		}
		else if((pathStep.current_segment_station_m < 0)&&(pathStep.current_segment_station_m!=-1)){

			if(MyDetailedPolicy.PolicyDQ.empty()){ // First state
				state.DAR_m=BehaviorUtils::getGrpLen(pathStep.current_segment_group,MyDetailedPolicy.Policy) + 	fabs(pathStep.current_segment_station_m);
			}
			else if(pathStep.current_segment_group==MyDetailedPolicy.PolicyDQ.back().step.current_segment_group){ // previous step is same as current step segment
				auto delta_dist = (TargetDistance-Prev_TargetDistance);
				state.DAR_m=DAR_m+delta_dist;
			}
			else //If this is the only station on the segment (so far)
			{
				state.DAR_m = MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] +  BehaviorUtils::getGrpLen(pathStep.current_segment_group,MyDetailedPolicy.Policy) + \
						fabs(pathStep.current_segment_station_m);
			}
			DAR_m = state.DAR_m;

			//	pathStep.current_segment_station_m = -1;
		}
		else if(pathStep.current_segment_station_m ==-1){

			if(pathStep.step_rule==ASSUME_RIGHT_OF_WAY_RULE){
				std::vector<LaneSegUidType> AllLanesInThisGrp = BU::routeSegmentGrpTolanesegments(pathStep.current_segment_group,MyDetailedPolicy.Policy);
				pathStep.step_rule = FOLLOW_INTERSECTION_RULE;
			}

		//	if(MyDetailedPolicy.PolicyDQ.back().observed_step_rule==OBEY_POSTED_SIGN_RULE)
		//		state.observed_step_rule = OBEY_POSTED_SIGN_RULE;

			DAR_m = MyDetailedPolicy.SegGrpOriginDAR[pathStep.current_segment_group] +  BehaviorUtils::getGrpLen(pathStep.current_segment_group,MyDetailedPolicy.Policy)+ \
					BU::getGrpConnLen(pathStep.current_segment_group,pathStep.target_segment_group,MyDetailedPolicy.Policy);
			state.DAR_m=DAR_m;


		}
		else{
			if(BehaviorUtils::codeRed( _CF_  + "Something is wrong in the behaviorinput::pathPlan information. Trouble in initstates() function. Check current_segment_station_m = ",pathStep.current_segment_station_m,1)){
				cout<<" pahtstep "<<pathStep<<endl;
			}
		}


		if(pathStep.step_rule==FOLLOW_INTERSECTION_RULE){
			//search for direction @ intersection
			int direction = pathStep.direction;
			if(direction!=STRAIGHT_DIRECTION){
				for(auto it = MyDetailedPolicy.PolicyDQ.rbegin();it !=MyDetailedPolicy.PolicyDQ.rend();it++){
					if((state.DAR_m-it->DAR_m)>40)
						break;
					if(it->step.step_rule==FOLLOW_INTERSECTION_RULE)//previous intersection
						break;
					if(it->step.direction!=STRAIGHT_DIRECTION)
						break;
					it->step.direction = direction;
				}
			}
		}

		state.step=pathStep;



		if(state.step.step_rule==-1)
			state.step.step_rule = ASSUME_RIGHT_OF_WAY_RULE; // Only to deal with lane change states now
		/*	if((state.step.step_rule==RIGHT_OF_WAY_AFTER_SIGNAL)&&(MyDetailedPolicy.PolicyDQ.back().step.step_rule==RIGHT_OF_WAY_AFTER_SIGNAL)){
			if((state.step.current_segment_group==MyDetailedPolicy.PolicyDQ.back().step.current_segment_group)){
				state.step.step_rule = RIGHT_OF_WAY_AFTER_SIGNAL_STOP_ZONE;
			}
		}*/

		state.GrpOrigin_DAR_m = Env::SegGrpOriginDAR(pathStep.current_segment_group,MyDetailedPolicy);
		state.GrpEnd_DAR_m = Env::SegGrpEndDAR(pathStep.current_segment_group,MyDetailedPolicy);
		state.CurrentGrpLanes = BehaviorUtils::routeSegmentGrpTolanesegments(state.step.current_segment_group,MyDetailedPolicy.Policy);
		state.TargetGrpLanes = BehaviorUtils::routeSegmentGrpTolanesegments(state.step.target_segment_group,MyDetailedPolicy.Policy);
		state.CurrentGrpLen = BehaviorUtils::getGrpLen(state.step.current_segment_group,MyDetailedPolicy.Policy);
		state.TgtGrpLen = BehaviorUtils::getGrpLen(state.step.target_segment_group,MyDetailedPolicy.Policy);
		state.CurrentToTgtGrpConnLen = BehaviorUtils::getGrpConnLen(state.step.current_segment_group,state.step.target_segment_group,MyDetailedPolicy.Policy);
		state.SuspectedOpposingGrp = BehaviorUtils::SuspectedOpposingLaneGrp(state.step.current_segment_group,MyDetailedPolicy.Policy);
		state.InterSectionAhead = MyDetailedPolicy.Policy.intersectionplan[state.step.current_segment_group];


		if(!MyDetailedPolicy.PolicyDQ.empty())
			state.MinTravelTm = (state.DAR_m -MyDetailedPolicy.PolicyDQ.back().DAR_m)/state.step.posted_speed_lim_mps ;
		else
			state.MinTravelTm = state.DAR_m/state.step.posted_speed_lim_mps;

		if(!MyDetailedPolicy.PolicyDQ.empty())
			state.Prev_State = &MyDetailedPolicy.PolicyDQ.back();

		MyDetailedPolicy.PolicyDQ.push_back(state);

		if(MyDetailedPolicy.PolicyDQ.size()>1)
			MyDetailedPolicy.PolicyDQ.rbegin()[1].Next_State = &MyDetailedPolicy.PolicyDQ.back();


	}


	return ;

}//initiState()



// Init Vehicle States not completed
bool Env::initVehicleState(Policy::DetailedPolicy &MyDetailedPolicy, bool ReInitRequest)
{

	return true; // disabled this function.
	bool initVehicleStateSuccess = false;
	const double TimeGap = 1;
	if(true)
	{
		usleep(1000);

		LaneSegUidType segID;

		if(behaviorinput::hvLoc.matched_laneseg_uid ==0){
			BehaviorUtils::codeRed( _CF_  + "initVehicle State NOT complete as Segcoords UID = 0 "," ",2.5);
			return false;
		}
		else{

			behaviorinput::curPosInfo->seg_fr = behaviorinput::hvLoc.matched_seg_fr;
			behaviorinput::curPosInfo->along_m = behaviorinput::hvLoc.matched_lane_station_m;
			behaviorinput::curPosInfo->conn_n = behaviorinput::hvLoc.matched_conn_n;
			behaviorinput::curPosInfo->point_fr = behaviorinput::hvLoc.matched_point_fr;
			behaviorinput::curPosInfo->offset_m = behaviorinput::hvLoc.matched_lane_lateral_offset_m;
			segID = behaviorinput::hvLoc.matched_laneseg_uid;
		}

		vehicle.curSegmentUid=segID;
		vehicle.DTACS=behaviorinput::curPosInfo->along_m;
		vehicle.prevDTACS=vehicle.DTACS;


		bool segExists = BehaviorUtils::SegExists(MyDetailedPolicy.Policy,segID);
		if(!segExists){
			BehaviorUtils::codeRed( _CF_  + " Can't find the current Segment in route. Seg ID = ",segID,TimeGap);
			return false;
		}

		bool vehicleInPlannedRoute=false;
		for(auto SegGrp:MyDetailedPolicy.Policy.routeplan.segment_group)
		{
			if(BehaviorUtils::SegExistsInThisGroup(MyDetailedPolicy.Policy,vehicle.curSegmentUid,SegGrp))
			{
				//Vehicle is somewhere ON the planned route
				vehicleInPlannedRoute=true;
				break;
			}


		}


		//Pop out states in the policy until the state just before/at the current vehicle state.
		if(vehicleInPlannedRoute)
		{
			bool foundInitState=false;
			if(BU::find(MyDetailedPolicy.PolicyDQ[0].CurrentGrpLanes,vehicle.curSegmentUid))
			{
				foundInitState=true;
				vehicle.DTAR= behaviorinput::hvLoc.matched_lane_station_m + Env::LaneSegOriginDAR(behaviorinput::hvLoc.matched_laneseg_uid,MyDetailedPolicy) ;
				BehaviorUtils::codeGreen( _CF_ +"Vehicle's state init near planned origin. DTAR = ",vehicle.DTAR);
				//return (true);
			}
			else
			{
				for(size_t i=1;i<MyDetailedPolicy.PolicyDQ.size();i++)
				{
					if(BU::find(MyDetailedPolicy.PolicyDQ[i].CurrentGrpLanes,vehicle.curSegmentUid))
					{
						foundInitState=true;
						//DTAR~DAR till the previous segment + the len(conn)+DTACS
						try{
							vehicle.DTAR = behaviorinput::hvLoc.matched_lane_station_m + Env::LaneSegOriginDAR(behaviorinput::hvLoc.matched_laneseg_uid,MyDetailedPolicy) ;
							BehaviorUtils::codeBlue( _CF_ +"Found initial state of vehicle to have completed DTAR ",vehicle.DTAR);
							//Erase the previous/unused states
							MyDetailedPolicy.PolicyDQ.erase(MyDetailedPolicy.PolicyDQ.begin(),MyDetailedPolicy.PolicyDQ.begin()+i);
							//return (true);
						}catch(...){
							BehaviorUtils::codeRed( _CF_  + " Error in function . Exiting");
							exit(0);
						}
					}
				}
			}
			if(!foundInitState)
			{
				BehaviorUtils::codeRed( _CF_  + "Failed to associate vehicle's state within the current planned environment.\nAborting... curSegmentUid = ",vehicle.curSegmentUid);
				if(!ReInitRequest)
					exit(1);
			}
		}
		else//Vehicle not in Planned route
		{
			//Vehicle not found on the route. Request a new behaviorinput::pathPlan from here to the goal/destination.
			BehaviorUtils::codeRed( _CF_  + "Vehicle is being reported to be in segID = ", vehicle.curSegmentUid, TimeGap);
			BehaviorUtils::codeRed( _CF_  + " segment_count = ",behaviorinput::routeSegmentList.segment_count, TimeGap);
			BehaviorUtils::codeRed( _CF_  + "vehicle does not seem to be on the planned route.\n\
					 Please abort the current plan and generate a new valid behaviorinput::pathPlan from the current vehicle's location.\n Exiting...\n"," ",TimeGap);
			return (false);
			//exit(0);
		}

	}

	Env::updateVehicleState(MyDetailedPolicy); // Needed to update some additional variables.
	return(true);
}


void Env::updateVehicleState(Policy::DetailedPolicy &MyDetailedPolicy)
{

	bool badObservation = true;
	LaneSegUidType segID;
	const double DTAR_Thresh = 5;
	const double TimeGap = 1.0 ;
	double Current_DTAR;


	behaviorinput::curPosInfo->seg_fr = behaviorinput::hvLoc.matched_seg_fr;
	behaviorinput::curPosInfo->along_m = behaviorinput::hvLoc.matched_lane_station_m;
	behaviorinput::curPosInfo->conn_n = behaviorinput::hvLoc.matched_conn_n;
	behaviorinput::curPosInfo->point_fr = behaviorinput::hvLoc.matched_point_fr;
	behaviorinput::curPosInfo->offset_m = behaviorinput::hvLoc.matched_lane_lateral_offset_m;

	badObservation = false;
	segID = behaviorinput::hvLoc.matched_laneseg_uid;
	bool segExistsNew =BehaviorUtils::SegExists(MyDetailedPolicy.Policy,segID);


	if(!segExistsNew){
		badObservation = true;
		std::string segstr = std::to_string(segID);
		if(BehaviorUtils::codeRed( _CF_  + "Possible localization error. In function updateVehicleState() couldn't find segID " + segstr," ",1)){
			std::cout<<" Input policy route plan "<<MyDetailedPolicy.Policy.routeplan.segment_uid<<std::endl;
		}
	}


	/*if(Env::MaxDTAR<vehicle.DTAR){
		Env::MaxDTAR = vehicle.DTAR;
		Env::MaxDTAR_VehicleState = vehicle;
	}
	 */



	if(!badObservation)//If its a good observation of the vehicle's state
	{
		vehicle.curSegCompletionRatio=behaviorinput::curPosInfo->seg_fr;
		vehicle.curSegmentUid=segID;
		vehicle.prevDTACS=vehicle.DTACS;
		vehicle.DTACS=behaviorinput::curPosInfo->along_m;
		vehicle.c0 = behaviorinput::curPosInfo->offset_m;
		vehicle.c1 = behaviorinput::hvLoc.matched_lane_misalign_rad;

		vehicle.targetSegmentUid = Env::FindNextLaneinRoute(segID,MyDetailedPolicy.Policy);



		double ConnLength;
		if(vehicle.curSegCompletionRatio>1.0){
			vehicle.curConnOriginSegUid = vehicle.curSegmentUid;
			vehicle.curConnDestinationSegUid = vehicle.targetSegmentUid;
			ConnLength = BehaviorUtils::getConnLen(vehicle.curConnOriginSegUid,vehicle.curConnDestinationSegUid);
			vehicle.curConnCompletionRatio = (vehicle.DTACS-BehaviorUtils::getSegLen(vehicle.curConnOriginSegUid))/ConnLength;
		}
		else if(vehicle.curSegCompletionRatio<0.0){
			vehicle.curConnDestinationSegUid = vehicle.curSegmentUid;
			//vehicle.curConnDestinationSegUid = Env::FindNextLaneinRoute(vehicle.curConnOriginSegUid,MyPolicy);
			vehicle.curConnOriginSegUid = Env::FindPrevLaneinRoute(vehicle.curConnDestinationSegUid,MyDetailedPolicy.Policy);
			if(vehicle.curConnOriginSegUid ==-1)
				cout<<" FindPrevLaneinRoute gives -1 for "<<vehicle.curConnOriginSegUid<<endl;
			ConnLength = BehaviorUtils::getConnLen(vehicle.curConnOriginSegUid,vehicle.curConnDestinationSegUid);
			vehicle.curConnCompletionRatio = (vehicle.DTACS+ConnLength)/ConnLength;
		}
		else{
			vehicle.curConnCompletionRatio = -1;
			if((vehicle.curConnOriginSegUid==0)||(vehicle.curConnDestinationSegUid==0)){
				vehicle.curConnDestinationSegUid = segID;
				int64_t prev_segID = Env::FindPrevLaneinRoute(segID,MyDetailedPolicy.Policy);
				if(!BU::SegExists(MyDetailedPolicy.Policy,prev_segID)){
					BU::codeRed( _CF_  + " Can't find (in route) lane segment ",prev_segID,1);
					BU::codeRed( _CF_  + " Current value of vehicle.curConnOriginSegUid ",vehicle.curConnOriginSegUid,1);
					prev_segID = -1;
				}

				if(prev_segID!=-1){
					vehicle.curConnOriginSegUid = prev_segID;
					BU::codeBlue( _CF_ +"prev_segID",prev_segID,1);

				}
				else{
					auto CurrSeg = BehaviorUtils::getLaneSeg(segID);
					if(behaviorinput::lanesegments.find(segID)!=behaviorinput::lanesegments.end()){
						int64_t PrevLaneSegID  = CurrSeg.connection[CurrSeg.primary_prev_connection_n].next_laneseg_uid ;
						if(behaviorinput::lanesegments.find(PrevLaneSegID)!=behaviorinput::lanesegments.end())
							vehicle.curConnOriginSegUid = PrevLaneSegID;
						BU::codeBlue( _CF_ +" CurrSeg.primary_prev_connection_n ",CurrSeg.primary_prev_connection_n,1);
					}
					else
						BU::codeRed( _CF_  + " Can't find lane segment ",segID,1);
				}
			}
			else{
				BU::codeBlue( _CF_ +" vehicle.curConnOriginSegUid ",vehicle.curConnOriginSegUid,1);
			}
		}

		double DTAR = 0;
		int CurrentGrp = BU::LaneSegmentToRouteSegmentGrp(vehicle.curSegmentUid,MyDetailedPolicy.Policy);
		double LaneSegOriginDAR = Env::LaneSegOriginDAR(segID,MyDetailedPolicy);
		if(BU::SuspectedOpposingLaneGrp(CurrentGrp,MyDetailedPolicy.Policy))
			DTAR = BU::getSegLen(segID)-behaviorinput::hvLoc.matched_lane_station_m + LaneSegOriginDAR ;
		else
			DTAR = behaviorinput::hvLoc.matched_lane_station_m + LaneSegOriginDAR ;


		vehicle.DTAR = DTAR;


		Env::FindFrontState(MyDetailedPolicy);
		Env::_ClosestInterSectionInfo = Env::FindClosestInterSectionInfo(CurrentGrp);

	}
	else//If the observation is bad
	{
		const double badObservationTimeGap = 1;
		BehaviorUtils::codeRed( _CF_  + "!!!!!!!!!!!!!!!!!Bad observation of vehicle's state detected. Skipping the observation until a good observation is available.!!!!!!"," ",badObservationTimeGap);
		BehaviorUtils::codeRed( _CF_  + " vehicle.curSegmentUid = ",vehicle.curSegmentUid,badObservationTimeGap);
		BehaviorUtils::codeRed( _CF_  + " Localization segID = ",segID,badObservationTimeGap);
	}


}



bool Env::CheckAndPopCompletedStates(){
	bool ReachedNextLevel = false;
	auto PolicyDQ = Policy::ActiveDetailedPolicy.PolicyDQ;
	if((Policy::ActiveDetailedPolicy.Front<0)||(Policy::ActiveDetailedPolicy.Front>=PolicyDQ.size()))
		return false;
	auto env_s=PolicyDQ[Policy::ActiveDetailedPolicy.Front].DAR_m-vehicle.DTAR;

	static int level = 0;
	bool FrontStepCompleted = false;

	if ((MAP_INCONSISTENCY_THRESHOLD_m>=env_s))//TODO:THe DAR seems to be more than the DTAR calculated by 1.xx meters.Should fix it or use a threshold window.
	{


		static int Prev_Front;
		if(PolicyDQ.empty())
			BehaviorUtils::codeGreen( _CF_ +"Agent has reached the goal");
		else if(Policy::ActiveDetailedPolicy.Front!=Prev_Front)
		{
			Prev_Front = Policy::ActiveDetailedPolicy.Front;
			const double TimeGap = -1;
			level++;
			clk=BehaviorUtils::TimeNow();
			Behavior_time = clk-start_clk;
			BehaviorUtils::codeGreen( _CF_ +"-------------------------------------------------------------------------------"," ",TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"Current time is (s) ",Behavior_time,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"Agent has advanced to the Level ", level,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"curSegmentUid ",vehicle.curSegmentUid,TimeGap);
			BehaviorUtils::codeBlue( _CF_ +"Localization curSegmentUid ",behaviorinput::hvLoc.matched_laneseg_uid,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +" curSegCompletionRatio = ",vehicle.curSegCompletionRatio,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"TargetSegmentUid ",vehicle.targetSegmentUid,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"vehicle DTAR ",vehicle.DTAR,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"Next Step DAR is  ",PolicyDQ[Policy::ActiveDetailedPolicy.Front].DAR_m,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"behaviorinput::pathPlan's next group step segment  is ",PolicyDQ[Policy::ActiveDetailedPolicy.Front].step.current_segment_group,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"Transition to next Level is away by (m) ",PolicyDQ[Policy::ActiveDetailedPolicy.Front].DAR_m-vehicle.DTAR,TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"Next State is: ",PolicyDQ[Policy::ActiveDetailedPolicy.Front],TimeGap);
			BehaviorUtils::codeGreen( _CF_ +"*****************************************************************************"," ",TimeGap);
			ReachedNextLevel = true;
		}
	}


	try{
		Env::updateVehicleState(Policy::ActiveDetailedPolicy);
		Policy::ActiveDetailedPolicy.rootLane = HV_LANE_SITUATION;
		Policy::ActiveDetailedPolicy.alternaterouteNeeded = Policy::ActiveDetailedPolicy.Policy.routeplan.reroute_status;
	}catch(...){
		BehaviorUtils::codeRed( _CF_  + "Caught updateVehicleState error. "," ",0.5);
	}

	return ReachedNextLevel;
}

void Env::InitStatesAndVehicleStates(const Policy::InputPolicyType & MyPolicy){
	static bool allMapletlanesegmentsAvailableOnce,validRouteSegListAvailableOnce,validpathPlanAvailableOnce,ValidFirstLocalizationOnce,ValidFirstSegcoordsLocalizationOnce;
	std::vector<LaneSegUidType> MissingLaneSegmentData;
	if(!allMapletlanesegmentsAvailableOnce&&allMapletlanesegmentsAvailable){
		BehaviorUtils::codeGreen( _CF_ +"allMapletlanesegmentsAvailable check passed "," ",1);
		allMapletlanesegmentsAvailableOnce = true;
	}
	else if(!allMapletlanesegmentsAvailable){
		if(BehaviorUtils::codeRed( _CF_  + "allMapletlanesegmentsAvailable check NOT passed "," ",5)){
			for(auto it =std::begin(MyPolicy.routeplan.segment_uid);it < std::begin(MyPolicy.routeplan.segment_uid) + MyPolicy.routeplan.segment_count;it++){
				if ( behaviorinput::lanesegments.find(*it)!=behaviorinput::lanesegments.end() ){
					continue;
				}
				else{
					MissingLaneSegmentData.push_back(*it);
				}
			}
			cout<<" MissingLaneSegmentData : "<<MissingLaneSegmentData<<endl;
			cout<<blue_on<<" Received lane segments: ";
			for(const auto &lanesg:behaviorinput::lanesegments)
				cout<<"  " <<lanesg.second.uid;
			cout<<color_off<<endl;

		}
	}


	if(!validRouteSegListAvailableOnce&&validRouteSegListAvailable){
		BehaviorUtils::codeGreen( _CF_ +"validRouteSegListAvailableOnce check passed "," ",1);
		validRouteSegListAvailableOnce = true;
	}
	else if (!validRouteSegListAvailable){
		BehaviorUtils::codeRed( _CF_  + "validRouteSegListAvailable check NOT passed "," ",1);
	}

	if(!validpathPlanAvailableOnce&&validpathPlanAvailable){
		BehaviorUtils::codeGreen( _CF_ +"validpathPlanAvailable check passed "," ",1);
		validpathPlanAvailableOnce = true;
	}
	else if(!validpathPlanAvailable){
		BehaviorUtils::codeRed( _CF_  + "validpathPlanAvailable check NOT passed "," ",1);
	}

	if(!ValidFirstLocalizationOnce && ValidFirstLocalization){
		BehaviorUtils::codeGreen( _CF_ +"ValidFirstLocalizationOnce check passed "," ",1);
		ValidFirstLocalizationOnce = true;
	}
	else if(!ValidFirstLocalization){
		BehaviorUtils::codeRed( _CF_  + "ValidFirstLocalization check NOT passed "," ",1);
	}

	if(!ValidFirstSegcoordsLocalizationOnce&&ValidFirstSegcoordsLocalization){
		BehaviorUtils::codeGreen( _CF_ +"ValidFirstSegcoordsLocalizationOnce check passed "," ",1);
		ValidFirstSegcoordsLocalizationOnce = true;
	}
	else if(!ValidFirstSegcoordsLocalization){
		BehaviorUtils::codeRed( _CF_  + "ValidFirstSegcoordsLocalization check NOT passed "," ",1);
	}

	if(!Env::initDone)
	{
		if(validpathPlanAvailable && allMapletlanesegmentsAvailable && validRouteSegListAvailable && ValidFirstLocalization && ValidFirstSegcoordsLocalization )
		{
			BU::codeGreen( _CF_ +"All the initial data availability checks passed.for Policy UID ",MyPolicy.uid,1);
			Env::initStates(Policy::ActiveDetailedPolicy,false,0,true);//Init environment

			try{

				if(Env::initVehicleState(Policy::ActiveDetailedPolicy)){
					BU::codeGreen( _CF_ +"Init Vehicle states done. ");
					if(PRINT_PATHPLAN)
						BU::codeGreen( _CF_ +" Policy::ActiveDetailedPolicy.PolicyDQ ",Policy::ActiveDetailedPolicy.PolicyDQ,-1);


					Env::initDone=true;
				}
				else{
					BehaviorUtils::codeRed( _CF_  + "Init Vehicle States not completed. Still trying ... ");
				}			}
			catch(...){
				BehaviorUtils::codeRed( _CF_  + "Caught init VehicleState error. "," ",1);
			}


		}
		else
		{

			BU::codeRed( _CF_  + "Init NOT done. validpathPlanAvailable",validpathPlanAvailable,0.5);
			BU::codeRed( _CF_  + "Init NOT done. allMapletlanesegmentsAvailable",allMapletlanesegmentsAvailable,0.5);
			BU::codeRed( _CF_  + "Init NOT done. validRouteSegListAvailable",validRouteSegListAvailable,0.5);
			BU::codeRed( _CF_  + "Init NOT done. ValidFirstLocalization",ValidFirstLocalization,0.5);
			BU::codeRed( _CF_  + "Init NOT done. ValidFirstSegcoordsLocalization",ValidFirstSegcoordsLocalization,0.5);


			//	firstTime=false;
		}
	}
}



bool Env::operator ==(const Env::PolicyState& A, const Env::PolicyState& B){
	return ((A.DAR_m==B.DAR_m)&&(A.step.current_segment_station_m==B.step.current_segment_station_m)&&(A.step.step_index_n==B.step.step_index_n)&&\
			(A.step.step_rule==B.step.step_rule)&&(A.step.step_task==B.step.step_task)&&(A.step.current_segment_group==B.step.current_segment_group)&&\
			(A.step.target_segment_group==B.step.target_segment_group));
}

bool Env::operator !=(const Env::PolicyState& A, const Env::PolicyState& B){
	return !((A.step.current_segment_station_m==B.step.current_segment_station_m)&&(A.step.step_index_n==B.step.step_index_n)&&\
			(A.step.step_rule==B.step.step_rule)&&(A.step.step_task==B.step.step_task)&&(A.step.current_segment_group==B.step.current_segment_group)&&\
			(A.step.target_segment_group==B.step.target_segment_group));
}

PolicyDequeType & Env::operator+(const PolicyDequeType & PolicyDQ_A, const PolicyDequeType & PolicyDQ_B ){
	PolicyDequeType MyPolicyDQ ;
	PolicyDequeType MyPolicyDQTail = PolicyDQ_B;
	Env::PolicyState LastState_A = PolicyDQ_A.back();
	Env::PolicyState FrontState_B = PolicyDQ_B.front();
	double delta_DAR_m = LastState_A.DAR_m-FrontState_B.DAR_m;
	LaneSegUidType lastlaneUID_A = LastState_A.CurrentGrpLanes.back();
	LaneSegUidType FirstlaneUID_B = FrontState_B.CurrentGrpLanes.front();
	int connId;
	BehaviorUtils::getConn( lastlaneUID_A, FirstlaneUID_B, &connId);

	if(LastState_A==FrontState_B){
		MyPolicyDQ = PolicyDQ_A;
		MyPolicyDQ.insert( MyPolicyDQ.end(), std::next(PolicyDQ_B.begin()), PolicyDQ_B.end() );
	}
	else if(connId!=-1){ // Not done yet
		MyPolicyDQ = PolicyDQ_A;
		for(auto &state:MyPolicyDQTail)
			state.DAR_m += delta_DAR_m;
		MyPolicyDQ.insert( MyPolicyDQ.end(), PolicyDQ_B.begin(), PolicyDQ_B.end() );
	}
	else if(LastState_A.step.current_segment_group==FrontState_B.step.current_segment_group){ //There is overlap

		for(auto &state:MyPolicyDQTail)
			state.DAR_m += delta_DAR_m;
		MyPolicyDQ.insert( MyPolicyDQ.end(), std::next(MyPolicyDQTail.begin()), MyPolicyDQTail.end() );

	}

	return MyPolicyDQ;
}
