#include "../PCH/pch.hpp"


#include "../include/io_handler.hpp"
#include "../include/Vehicle.hpp"
#include "../include/BehaviorCals.hpp"
#include "../include/BehaviorUtils.hpp"
#include "../include/BehaviorObject.hpp"
#include "../include/lcmprint.hpp"
#include "../include/behaviorHorizon.hpp"
#include "../include/BehaviorStatic.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"

namespace BU = BehaviorUtils;
std::set<InterSectionUidType> IO_HANDLER::_AllReceivedInterSections;
std::set<LaneSegUidType> IO_HANDLER::_AllReceivedlanesegments;
extern size_t HashRoutePlan(const exlcm::routesegmentlist_t & Myrouteplan);
extern void HashPathStep(const exlcm::pathstep_t & Mypathstep, size_t& seed);
extern size_t HashpathPlan(const exlcm::pathplan_t & MypathPlan);


bool IO_HANDLER::PolicyDataCopy(){
	bool ImpureData= true;
	if (pthread_mutex_lock(&Mutex::_routeSegmentList_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for copying route segment list data in main thread ", " ",1);
		ImpureData = true; // problem
	}else{
		if(validRouteSegListAvailable && allMapletlanesegmentsAvailable && validpathPlanAvailable){
			behaviorinput::routeSegmentList = behaviorinput::routeSegmentList_IN;
			LaneSegUidType LastLaneSegUID = behaviorinput::routeSegmentList_IN.segment_uid[behaviorinput::routeSegmentList_IN.segment_count-1];
			LaneSegUidType next_laneseg_uid;
			std::vector< int16_t > routeSegGrp;
			int pathPlanEndIdx;
			int lastGrpIdx = -1;
			int DubiousGrp ;
			auto LastLaneSegment = BehaviorUtils::getLaneSeg(LastLaneSegUID);
			if(LastLaneSegment.connection_count==0){
				BU::codeYellow( _CF_ +"Something really wrong. LastSegment", LastLaneSegment.uid,2);
				BU::codeYellow( _CF_ +" connection count is ",LastLaneSegment.connection_count,2);
			}
			else if(behaviorinput::pathPlan_IN.destination_included){
				next_laneseg_uid =  (LastLaneSegment.connection[LastLaneSegment.primary_next_connection_n]).next_laneseg_uid;
				if(BU::MapletRequestMissingLanePub(next_laneseg_uid)){
					BehaviorUtils::codeRed( _CF_  + " MapletRequestMissingLanePub ", next_laneseg_uid,1);
					if ( pthread_mutex_unlock(&Mutex::_routeSegmentList_mutex) != 0) {
						BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying route segment list data in main thread", " ",1);
					}
					return true;
				}
				//Create fake group
				behaviorinput::routeSegmentList.segment_group.insert(behaviorinput::routeSegmentList.segment_group.begin()+\
						behaviorinput::routeSegmentList_IN.segment_count,behaviorinput::routeSegmentList_IN.segment_group[behaviorinput::routeSegmentList_IN.segment_count-1]+1000);
				behaviorinput::routeSegmentList.segment_uid.insert(behaviorinput::routeSegmentList.segment_uid.begin()+behaviorinput::routeSegmentList_IN.segment_count,next_laneseg_uid);
				behaviorinput::routeSegmentList.segment_count=behaviorinput::routeSegmentList.segment_uid.size();
				behaviorinput::pathPlan= behaviorinput::pathPlan_IN;
				lastGrpIdx = behaviorinput::routeSegmentList.segment_group[behaviorinput::routeSegmentList.segment_count-1];
				pathPlanEndIdx = behaviorinput::pathPlan.included_step_count-1;
				DubiousGrp = behaviorinput::pathPlan.path_steps[pathPlanEndIdx].target_segment_group;
				routeSegGrp = behaviorinput::routeSegmentList.segment_group;
				while((DubiousGrp==-1)||(std::find(routeSegGrp.begin(),routeSegGrp.end(),DubiousGrp)==routeSegGrp.end())){
					behaviorinput::pathPlan.path_steps[pathPlanEndIdx].target_segment_group = lastGrpIdx;
					pathPlanEndIdx--;
					if(pathPlanEndIdx<0)
						break;
					DubiousGrp = behaviorinput::pathPlan.path_steps[pathPlanEndIdx].target_segment_group;
				}
				if(behaviorinput::pathPlan.path_steps.size()>1){ // Adding stop and stand by at the last step.
					behaviorinput::pathPlan.path_steps.rbegin()[1].step_rule = STOP_AND_STAND_BY;
					behaviorinput::pathPlan.path_steps.back().step_rule = STOP_AND_STAND_BY;
				}
			}
			else{
				behaviorinput::pathPlan= behaviorinput::pathPlan_IN;
			}


			Policy::ActiveDetailedPolicy.Policy = Policy::InputPolicyType(behaviorinput::routeSegmentList,behaviorinput::pathPlan);
		//	if(Policy::ActiveDetailedPolicy.Policy.PolicyDiagnostics())
		//		exit(1);
			if(Policy::ActiveDetailedPolicy.Policy.routeplan.segment_group.empty())
				ImpureData = true;
			else{
				ImpureData = false;
				Policy::BackEndDetailedPolicy.Policy = Policy::ActiveDetailedPolicy.Policy;
			}
		}
		else{
			std::vector<LaneSegUidType> MissingLanes;
			if(!allMapletlanesegmentsAvailable){
				MissingLanes = BU::MapletRequestMissingLanePub(behaviorinput::routeSegmentList_IN);
				if(MissingLanes.empty()&& (!behaviorinput::routeSegmentList_IN.segment_uid.empty()))
					allMapletlanesegmentsAvailable = true;
			}

			if(!allMapletlanesegmentsAvailable)
				BU::codeRed( _CF_  + "allMapletlanesegmentsAvailable False. Missing Lanes are ",MissingLanes,1);

			if(!validpathPlanAvailable)
				BU::codeRed( _CF_  + "validpathPlanAvailable False."," ",1);

			if(!validRouteSegListAvailable)
				BU::codeRed( _CF_  + "validRouteSegListAvailable False."," ",1);


			ImpureData = true;
		}
		if ( pthread_mutex_unlock(&Mutex::_routeSegmentList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying route segment list data in main thread", " ",1);
			ImpureData = true;
		}
	}


	if(ImpureData){
		if(!validRouteSegListAvailable)
			BU::codeRed( _CF_  + "Inside Static Plan copy function. validRouteSegListAvailable False"," ",1);
		if(!allMapletlanesegmentsAvailable){
			std::vector<LaneSegUidType> MissingLanes = BU::MapletRequestMissingLanePub(behaviorinput::routeSegmentList);
			BU::codeRed( _CF_  + "Inside Static Plan copy function. allMapletlanesegmentsAvailable False"," ",1);
			if(BU::codeYellow( _CF_ +" Missing lane segments from the route are "," ",1))
				cout<<MissingLanes<<endl;
		}
		if(!validpathPlanAvailable)
			BU::codeRed( _CF_  + "Inside Static Plan copy function. validpathPlanAvailable False"," ",1);
	}


	return ImpureData;
}


bool IO_HANDLER::CopyIntersectionData()
{
	if (pthread_mutex_lock(&Mutex::_IntersectionList_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for copying intersection list data in main thread ", " ",1);
		return true;
	}else{

		behaviorinput::IntersectionList = behaviorinput::IntersectionList_IN;
		if ( pthread_mutex_unlock(&Mutex::_IntersectionList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying intersection list  data in main thread", " ",1);
			return true;
		}
	}
	return false;
}

bool IO_HANDLER::AlternatePolicyDataCopy()
{
	if (pthread_mutex_lock(&Mutex::_alternateplan_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for copying alternate policy data in main thread ", " ",1);
		return true;
	}else{

		Policy::AllaltPolicies.clear();
		for(auto & route:behaviorinput::AllaltrouteSegmentList_IN){
			if(behaviorinput::AllaltpathPlan_IN.find(route.first)!=behaviorinput::AllaltpathPlan_IN.end())
				Policy::AllaltPolicies[route.first] = Policy::InputPolicyType(route.second,behaviorinput::AllaltpathPlan_IN[route.first]);
		}
		//BehaviorUtils::CleanUpBuffer_AltPlans();
		if ( pthread_mutex_unlock(&Mutex::_alternateplan_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying alternate policy data in main thread", " ",1);
			return true;
		}
	}
	return false;
}

bool IO_HANDLER::CopyTafficSignData()
{
	if (pthread_mutex_lock(&Mutex::_traffic_sign_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for rcopying traffic sign data in main thread ", " ",1);
		return true;
	}else{

		behaviorinput::TrafficSignStatesList = behaviorinput::TrafficSignStatesList_IN;
		if ( pthread_mutex_unlock(&Mutex::_traffic_sign_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying traffic sign data in main thread", " ",1);
			return true;
		}
	}
	return false;
}


bool IO_HANDLER::CopyTafficSignalData()
{
	if (pthread_mutex_lock(&Mutex::_traffic_signal_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for copying traffic signal data in main thread ", " ",1);
		return true;
	}else{

		behaviorinput::TrafficSignalStatesList = behaviorinput::TrafficSignalStatesList_IN;
		if ( pthread_mutex_unlock(&Mutex::_traffic_signal_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying traffic signal data in main thread", " ",1);
			return true;
		}
	}
	return false;
}

bool IO_HANDLER::CopyObjData()
{
	static auto Last_obj_SA_mutex_LockTime = chrono::high_resolution_clock::now();
		if ((pthread_mutex_lock(&Mutex::_obj_SA_mutex) != 0)||(pthread_mutex_lock(&Mutex::_SA_mutex) != 0)) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for copying obj data in main thread ", " ",1);
			return true;
		}else{

			SA = SA_IN;
			SA.HVData = SA_IN_HV.HVData;
			if(SA.FindAuxlane_idx(HV_LANE_SITUATION)==-1)
				SA.HVData.Aux_lanesituation.push_back(SA_IN_HV.HVData.hv_lanesituation);
			Last_obj_SA_mutex_LockTime = chrono::high_resolution_clock::now();
			if (( pthread_mutex_unlock(&Mutex::_obj_SA_mutex) != 0)||( pthread_mutex_unlock(&Mutex::_SA_mutex) != 0)) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for copying obj data in main thread ", " ",1);
				//	continue;
			}
		}
//	}

	return false;
}





void IO_HANDLER::lcmMsgHandler::KINEMATICS( const exlcm::kinematics_t *msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	bool useKinematicsYaw = true;// Temporary fix to the wrong yaw published onto POSITIONPOSE in the white volt.

	if(useKinematicsYaw)
	{
		if(firstKin)
		{
			firstKin=false;
		}
	}
	vehicle.curSpeed_mps=msg->speed_mps;
	vehicle.curAccel_mpss = msg->lon_accel_mpss;


}


void IO_HANDLER::lcmMsgHandler::LOCALIZATION ( const exlcm::localization_t * msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if(msg->valid_f)
	{
		behaviorinput::hvLoc=*msg;
		if(!behaviorinput::routeSegmentList_IN.segment_uid.empty()){
			auto routeseguids = behaviorinput::routeSegmentList_IN.segment_uid;
			if(std::find(routeseguids.begin(),routeseguids.end(),msg->matched_laneseg_uid)!=routeseguids.end()){
				ValidFirstLocalization = true;
				ValidFirstSegcoordsLocalization = true;
			}
		}


	}

}


void IO_HANDLER::lcmMsgHandler::LANESEGMENT(const exlcm::lanesegment_t* msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if(msg->valid_f){
		behaviorinput::lanesegments[msg->uid]= *msg;
		IO_HANDLER::_AllReceivedlanesegments.insert(msg->uid);
	}
	else{
		cout<<red_on<<" Invalid lanesegment data received for segment "<<msg->uid<<color_off<<endl;
		return;
	}


	if (pthread_mutex_trylock(&Mutex::_routeSegmentList_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for input :behaviorinput::routeSegmentList data", " ",1);
	}else{
		allMapletlanesegmentsAvailable=true;
		if(!validRouteSegListAvailable){
			BehaviorUtils::codeRed( _CF_  + " Route segment list is not valid"," ",2.5);
			allMapletlanesegmentsAvailable=false;
		}

		for(auto it =std::begin(behaviorinput::routeSegmentList_IN.segment_uid);it<std::begin(behaviorinput::routeSegmentList_IN.segment_uid)+ behaviorinput::routeSegmentList_IN.segment_count;it++){
			if ( behaviorinput::lanesegments.find(*it) == behaviorinput::lanesegments.end() ){
				BehaviorUtils::codeRed( _CF_  + " could not find all lane segments in the route"," ",2.5);
				allMapletlanesegmentsAvailable=false;
			}
		}
		if ( pthread_mutex_unlock(&Mutex::_routeSegmentList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for input :behaviorinput::routeSegmentList data", " ",1);
		}
	}


	if(allMapletlanesegmentsAvailable)
		BehaviorUtils::codeGreen( _CF_ +"Got all behaviorinput::lanesegments from maplet","",100);

}
void IO_HANDLER::lcmMsgHandler::PATHPLAN( const exlcm::pathplan_t *msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;

	if(msg->valid_f){
		if (pthread_mutex_trylock(&Mutex::_routeSegmentList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for input :behaviorinput::pathPlan  data", " ",1);
		}else if(msg->total_step_count>0){
			validpathPlanAvailable=true;
			size_t currplan_seed = (int64_t)HashpathPlan(*msg);
			size_t prevplan_seed = (int64_t)HashpathPlan(behaviorinput::pathPlan_IN);
			bool samePlan = (currplan_seed == prevplan_seed);
			if(!samePlan){
				abortCurPlan=true;
				allMapletlanesegmentsAvailable = false;
				behaviorinput::pathPlan_IN=*msg;
				BehaviorUtils::codeGreen( _CF_ +"Got valid new behaviorinput::pathPlan with uid ",currplan_seed,0.1);
				BehaviorUtils::codeGreen( _CF_ +"previous behaviorinput::pathPlan uid was  ",prevplan_seed,0.1);

			}
			else{
				abortCurPlan=false;
			}
			//	}
			if ( pthread_mutex_unlock(&Mutex::_routeSegmentList_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for input :behaviorinput::pathPlan data", " ",1);
			}
		}
	}

}

void IO_HANDLER::lcmMsgHandler::ALTERNATEPATHPLAN(const exlcm::pathplan_t *msg){
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if((msg->valid_f)&&(msg->included_step_count > 0)){

		if (pthread_mutex_trylock(&Mutex::_alternateplan_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for alternate plan ", " ",1);
		}else if(msg->total_step_count>0){
			behaviorinput::AllaltpathPlan_IN[msg->reroute_status] = *msg;
			behaviorinput::AllaltpathPlan_IN_LastUpdateTime[msg->reroute_status] = std::chrono::high_resolution_clock::now();
			if ( pthread_mutex_unlock(&Mutex::_alternateplan_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for alternate plan ", " ",1);
			}
		}
	}
}


void IO_HANDLER::lcmMsgHandler::INTERSECTION(const exlcm::intersection_t*msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if(msg->valid_f && ((msg->connection_count)>2)){
		if (pthread_mutex_trylock(&Mutex::_IntersectionList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for behaviorinput::IntersectionList ", " ",1);
		}else{
			if(msg->uid>0){
				behaviorinput::IntersectionList_IN[msg->uid]=(*msg);
				IO_HANDLER::_AllReceivedInterSections.insert(msg->uid);
			}
			//	}
			if ( pthread_mutex_unlock(&Mutex::_IntersectionList_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for behaviorinput::IntersectionList ", " ",1);
			}
		}
	}
	else{
		//validRouteSegListAvailable=false;
		BehaviorUtils::codeGreen( _CF_ +"Received Intersection list is not valid");
	}
}

void IO_HANDLER::lcmMsgHandler::ROUTESEGMENTLIST( const exlcm::routesegmentlist_t *msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if(msg->valid_f && (msg->segment_count>0)){
		if (pthread_mutex_trylock(&Mutex::_routeSegmentList_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for input :behaviorinput::routeSegmentList data", " ",1);
		}else if(msg->segment_count>0){
		//	validpathPlanAvailable=true;
			size_t currplan_seed = (int64_t)HashRoutePlan(*msg);
			size_t prevplan_seed = (int64_t)HashRoutePlan(behaviorinput::routeSegmentList_IN);
			bool samePlan = (currplan_seed == prevplan_seed);
			if(!samePlan){
				behaviorinput::routeSegmentList_IN=*msg;
				BehaviorUtils::codeGreen( _CF_ +"Got valid new RouteSegment List with uid ",currplan_seed,0.1);
				BehaviorUtils::codeGreen( _CF_ +"previous RouteSegment List uid was  ",prevplan_seed,0.1);
			}

			validRouteSegListAvailable=true;

			//	}
			if ( pthread_mutex_unlock(&Mutex::_routeSegmentList_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for input :behaviorinput::routeSegmentList data", " ",1);
			}
		}
	}
	else{
		validRouteSegListAvailable=false;
		BehaviorUtils::codeRed( _CF_  + "Received RouteSegment list behaviorinput::routeSegmentList_IN is not valid",behaviorinput::routeSegmentList_IN,1);
	}
}

void IO_HANDLER::lcmMsgHandler::ALTERNATEROUTESEGMENTLIST( const exlcm::routesegmentlist_t *msg){
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if(msg->valid_f && (msg->segment_count>0)){
		if (pthread_mutex_trylock(&Mutex::_alternateplan_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for alternate plan ", " ",1);
		}else if(msg->segment_count>0){
			behaviorinput::AllaltrouteSegmentList_IN[msg->reroute_status] = *msg;
			behaviorinput::AllaltrouteSegmentList_IN_LastUpdateTime[msg->reroute_status] = std::chrono::high_resolution_clock::now();
			if ( pthread_mutex_unlock(&Mutex::_alternateplan_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for alternate plan ", " ",1);
			}
		}

	}

}

void IO_HANDLER::lcmMsgHandler::SITUATION(const exlcm::situation_t * msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	// Assign the current situation awareness object to the global SA

	if (pthread_mutex_trylock(&Mutex::_SA_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for input HV data", " ",1);
	}else{

		SA_IN_HV.HVData=*msg;
		if ( pthread_mutex_unlock(&Mutex::_SA_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for input HV data", " ",1);
		}
	}

}


void IO_HANDLER::lcmMsgHandler::OBJSITUATION( const exlcm::objsituation_t * msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;

	//TODO: make SA.objData a std::map with uid as keys and replace the values if data comes in for the same obj instead of accumulating all the objects.
	if (pthread_mutex_trylock(&Mutex::_obj_SA_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for input obj data", " ",1);
	}else{
		if((msg->uid > 0)&&(msg->valid_pred_count>0)&&(msg->valid_f)){
			SA_IN.objData[msg->uid]=(*msg);
			SA_IN.objDataUpdateClock[msg->uid] = std::chrono::high_resolution_clock::now();
			BehaviorUtils::CleanUpBuffer_SA();
		}
		if ( pthread_mutex_unlock(&Mutex::_obj_SA_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for input obj data", " ",1);
		}
	}

}





void IO_HANDLER::lcmMsgHandler::TRAJECTORYPLAN(const exlcm::trajectoryplan_t* msg)
{
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	behaviorinput::_trajectoryplan = *msg;
}

void IO_HANDLER::invokeNonBlockingLCMHandle()
{
	/*int rv=poll(&fds,1,50);
	if(rv==-1)
		perror("Poll");
	else if (rv==0){

	}
	else
		if(fds.events&POLLIN)//|| POLLPRI
			lcm1.handle();
	*/
}

//NOTE:Not External dynamic/static param handling not implemented fully. The base class definitions are in place just needs few more lines to invoke the methods. Use paramHandler to add functionality.



void IO_HANDLER::lcmMsgHandler::TRAFFICSIGNALINTERPRETATION(const exlcm::trafficsignalinterpretation_t* msg){

	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if (pthread_mutex_trylock(&Mutex::_traffic_signal_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for traffic signal data", " ",1);
	}else{

		if(msg->valid_f){
			for(int i =0;i<msg->connection_count;i++){
				exlcm::connectiontrafficsignal_t My_connectiontrafficsignal_t =  (msg->connection_traffic_signal_state[i]);

				behaviorinput::TrafficSignalStatesList_IN[std::make_pair(My_connectiontrafficsignal_t.incoming_laneseg_uid,My_connectiontrafficsignal_t.connection_n)]= \
						std::make_pair(msg->connection_traffic_signal_state[i],BehaviorUtils::TimeNow());

			}
		}

		if ( pthread_mutex_unlock(&Mutex::_traffic_signal_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for traffic signal data", " ",1);
		}
	}
}

void IO_HANDLER::lcmMsgHandler::TRAFFICSIGNINTERPRETATION(const exlcm::trafficsigninterpretation_t* msg){
	if(behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)
		return;
	if (pthread_mutex_trylock(&Mutex::_traffic_sign_mutex) != 0) {
		BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for traffic sign data", " ",1);
	}else{

		if(msg->valid_f){
			for(int i =0;i<msg->connection_count;i++){
				exlcm::connectiontrafficsign_t My_connectiontrafficsign_t =  (msg->connection_traffic_sign[i]);
				behaviorinput::TrafficSignStatesList_IN[std::make_pair(My_connectiontrafficsign_t.incoming_laneseg_uid,My_connectiontrafficsign_t.connection_n)]= \
						std::make_pair(msg->connection_traffic_sign[i],BehaviorUtils::TimeNow());
			}
		}

		if ( pthread_mutex_unlock(&Mutex::_traffic_sign_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for traffic sign data", " ",1);
		}
	}
}



void IO_HANDLER::lcmMsgHandler::VISUALIZERSTATES(const exlcm::visualizerstates_t* msg){
	if (pthread_mutex_trylock(&Mutex::_visualizerstates_mutex) != 0) {
		cout<<" could not acquire visualizerstates mutex lock for VISUALIZERSTATES "<<endl;
	}else{

		if((!behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)&&msg->FreezeFrameOnLogPlayerPause)
			LastPauseClock = std::chrono::high_resolution_clock::now();

		behaviorinput::_visualizerstates = *msg;

		if ( pthread_mutex_unlock(&Mutex::_visualizerstates_mutex) != 0) {
			cout<<" could not unlock visualizerstates mutex lock for VISUALIZERSTATES "<<endl;
		}
	}
}


//lcm::LCM lcm2;

int IO_HANDLER::lcmMsgHandler::lcm_thread()
{

    /*if (!lcm2.good()) {
        printf("cannot start LCM");
        return 1;
    } */

    //lcm2.subscribe("SITUATION",&IO_HANDLER::lcmMsgHandler::SITUATION,this);
    //lcm2.subscribe("OBJSITUATION",&IO_HANDLER::lcmMsgHandler::OBJSITUATION,this);


   while(true){
 //   BU::codeGreen(_CF_ +  "my_func called from thread ",std::this_thread::get_id(),0.01);
    //	cout<<"Inside the lcm handler thread"<<endl;
	   //lcm2.handle();
    }


    printf("LCM stopped");

    return 0;
}

int IO_HANDLER::lcmMsgHandler::start()
{
    std::thread t1(&IO_HANDLER::lcmMsgHandler::lcm_thread, this);

    t1.detach();

    return 0;
}
