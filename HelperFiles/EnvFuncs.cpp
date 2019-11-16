#include "../PCH/pch.hpp"


#include "../include/Env.hpp"
#include "../include/BehaviorUtils.hpp"
#include "../include/io_handler.hpp"
#include "../include/lcmprint.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"


namespace BU=BehaviorUtils;

struct  vehicle;


int Env::SearchState(const Env::PolicyState &TargetState, const PolicyDequeType & MyPolicyDQ){ // WORKS FOR BOTH LANE SEGMENT AND GROUPS
	int StateIdx = TargetState.step.step_index_n-MyPolicyDQ[0].step.step_index_n;
	if(MyPolicyDQ.at(StateIdx)==TargetState)
		return StateIdx;
	else{
		if(BehaviorUtils::codeRed( _CF_  + "Could not match the desired state idx ",StateIdx,0.5)){
			cout<<" TargetState : "<<TargetState<<endl;
		}
		return -1;
	}
	return -1;
}

Env::PolicyState Env::prev( const Env::PolicyState& CurrentState){
	Env::PolicyState PrevState = *CurrentState.Prev_State;
	return PrevState;
}
Env::PolicyState Env::next( const Env::PolicyState& CurrentState){
	Env::PolicyState NextState = *CurrentState.Next_State;
	return NextState;
}

int BinarySearchFrontIdx(PolicyDequeType & MyPolicyDQ, int L, int R ){
	int M = (L+R)/2;
	if(vehicle.DTAR>MyPolicyDQ[R].DAR_m)
		return R;
	else if(vehicle.DTAR<MyPolicyDQ[L].DAR_m)
		return L;
	else if(R==L+1)
		return R;
	if(vehicle.DTAR<MyPolicyDQ[M].DAR_m)
		return BinarySearchFrontIdx(MyPolicyDQ,L,M);
	else
		return BinarySearchFrontIdx(MyPolicyDQ,M,R);


}

void Env::FindFrontState( Policy::DetailedPolicy & MyDetailedPolicy){ // WORKS FOR BOTH LANE SEGMENT AND GROUPS
	int StartIdx = 0,FrontIdx = 0;
	StartIdx = BinarySearchFrontIdx(MyDetailedPolicy.PolicyDQ,StartIdx,MyDetailedPolicy.PolicyDQ.size()-1);
	StartIdx = max(StartIdx-5,0);
	for(FrontIdx=StartIdx;FrontIdx<MyDetailedPolicy.PolicyDQ.size();FrontIdx++){
		if(vehicle.DTAR<MyDetailedPolicy.PolicyDQ[FrontIdx].DAR_m){
			if(FrontIdx+1>=MyDetailedPolicy.PolicyDQ.size()){
				MyDetailedPolicy.Front = FrontIdx;
				return;
			}

			int CurrentGrp = MyDetailedPolicy.PolicyDQ[FrontIdx].step.current_segment_group;
			int NextGrp =  MyDetailedPolicy.PolicyDQ[FrontIdx+1].step.current_segment_group;
			int TgtGrp =  MyDetailedPolicy.PolicyDQ[FrontIdx].step.target_segment_group;
			int NextTgtGrp =  MyDetailedPolicy.PolicyDQ[FrontIdx+1].step.target_segment_group;
			auto CurrentGrpLanes = MyDetailedPolicy.PolicyDQ[FrontIdx].CurrentGrpLanes;
			auto NextGrpLanes = MyDetailedPolicy.PolicyDQ[FrontIdx+1].CurrentGrpLanes;
			auto NextTgtGrpLanes = MyDetailedPolicy.PolicyDQ[FrontIdx+1].TargetGrpLanes;

			if(BU::SuspectedLaneChaneGrpConn(CurrentGrpLanes,NextGrpLanes)){
				BU::codeBlue(_CF_ + " Adjusting Front state for Suspected lane changes between groups. (CurrentGrp,TgtGrp,NextGrp,NextTgtGrp) ", std::make_tuple(CurrentGrp,TgtGrp,NextGrp,NextTgtGrp),1);
				BU::codeBlue(_CF_ + " Also (curSegmentUid,targetSegmentUid) ", std::make_tuple(vehicle.curSegmentUid,vehicle.targetSegmentUid),1);
				BU::codeBlue(_CF_ + " CurrentGrp lanes ", CurrentGrpLanes,1);

				if(fabs(vehicle.c0)>MAX_TARGET_LANE_CENTER_OFFSET_FOR_LCX_CMPLT){
					//	cout<<"vehicle.c0 "<<vehicle.c0<<"vehicle.c1 "<<vehicle.c1<<endl;
					MyDetailedPolicy.Front = FrontIdx;
					return;
				}


				// Do more evaluations for lane change
				if(BU::find(CurrentGrpLanes,vehicle.curSegmentUid)){
					MyDetailedPolicy.Front = FrontIdx;
					return;
				}
				else if(BU::find(NextGrpLanes,vehicle.curSegmentUid)){
					if(BU::SuspectedLaneChaneGrpConn(NextGrpLanes,NextTgtGrpLanes)&&(CurrentGrpLanes==NextTgtGrpLanes)){ // This is for double lane change
						//MyPolicyDQ.erase(MyPolicyDQ.begin(),MyPolicyDQ.begin()+FrontIdx+1); // To remove ambiguity for DLCX
						MyDetailedPolicy.PolicyDQ[FrontIdx].DAR_m = -1;
						MyDetailedPolicy.PolicyDQ[FrontIdx+1].DAR_m = -1;
						MyDetailedPolicy.Front = FrontIdx+2;
						if(MyDetailedPolicy.Front>=MyDetailedPolicy.PolicyDQ.size())
							exit(1);
						return;
					}
					else{
						MyDetailedPolicy.PolicyDQ[FrontIdx].DAR_m = -1;
						MyDetailedPolicy.Front = FrontIdx+1;
						if(MyDetailedPolicy.Front>=MyDetailedPolicy.PolicyDQ.size())
							exit(1);
						return;
					}
				}
			}

			MyDetailedPolicy.Front = FrontIdx;
			return;
		}
	}
	MyDetailedPolicy.Front = MyDetailedPolicy.PolicyDQ.size()-1;
	return;
}


int Env::FindClosestState(Policy::DetailedPolicy & MyCandiatePolicy ){ // WORKS FOR BOTH LANE SEGMENT AND GROUPS

	int FrontIdx;
	Policy::DetailedPolicy MyCandiatePolicyCopy = MyCandiatePolicy;
	if(MyCandiatePolicy.Front<=0)
		Env::FindFrontState(MyCandiatePolicyCopy);

	FrontIdx = MyCandiatePolicyCopy.Front;

	if (FrontIdx==0)
		return FrontIdx;
	else if(fabs(MyCandiatePolicy.PolicyDQ[FrontIdx].DAR_m-vehicle.DTAR)<fabs(MyCandiatePolicy.PolicyDQ[FrontIdx-1].DAR_m-vehicle.DTAR))
		return FrontIdx;
	else
		return (FrontIdx-1);

}

Env::PolicyState Env::FindClosestState(Env::PolicyState &TargetState){ // WORKS FOR BOTH LANE SEGMENT AND GROUPS
	Env::PolicyState *MyTargetStateptr;
	MyTargetStateptr = &TargetState;

	do{
		if(MyTargetStateptr->Prev_State==nullptr)
			return *MyTargetStateptr;

		MyTargetStateptr = MyTargetStateptr->Prev_State;

	}while(MyTargetStateptr->DAR_m > vehicle.DTAR);

	return *MyTargetStateptr->Next_State;
}

double Env::CalculateTargetDistanceAlongSeg(const exlcm::pathstep_t& pathStep,const Policy::InputPolicyType & MyPolicy){ // WORKS FOR BOTH LANE SEGMENT AND GROUPS
	double TargetDistance = 0;
	if(pathStep.target_segment_station_m<=0){
		if(pathStep.current_segment_station_m>0)
			TargetDistance = pathStep.current_segment_station_m;
		else if(pathStep.current_segment_station_m != -1)
			TargetDistance = BU::getGrpLen(pathStep.current_segment_group,MyPolicy)+fabs(pathStep.current_segment_station_m);
	}
	else
		TargetDistance = pathStep.target_segment_station_m;
	return TargetDistance;
}



double Env::SegGrpOriginDAR(int segUID,  const Policy::InputPolicyType & MyPolicy, map<int,double>& MyAllGrpSegOriginDAR, double DAR_m_Origin){

	auto it = std::find( std::begin( MyPolicy.routeplan.segment_group ), std::end( MyPolicy.routeplan.segment_group ), segUID);
	double Candidate_DAR = -1;
	if(it != std::end( MyPolicy.routeplan.segment_group)){
		double DAR = DAR_m_Origin;
		for(int i=0;i<MyPolicy.routeplan.segment_group.size();i++){
			double OriginToNxtGrpEndLen = BU::getGrpLen(segUID,MyPolicy);
			int NextGrp = Env::FindNextGrpinRoute(segUID,MyPolicy);

			auto RouteGrp = MyPolicy.routeplan.segment_group;
			if(std::find(RouteGrp.begin(),RouteGrp.end(),NextGrp)!=RouteGrp.end()){
				OriginToNxtGrpEndLen += BU::getGrpLen(NextGrp,MyPolicy);
				OriginToNxtGrpEndLen += BU::getGrpConnLen(segUID,NextGrp,MyPolicy);

			}


			if((MyPolicy.routeplan.segment_group[i]==segUID))
			{

				MyAllGrpSegOriginDAR[MyPolicy.routeplan.segment_group[i]]=DAR;
				return DAR;// Must have found the group by the last iteration and the last iteration will not perform the following lines;

			}
			else
			{
				MyAllGrpSegOriginDAR[MyPolicy.routeplan.segment_group[i]]=DAR;
				if(i+1<MyPolicy.routeplan.segment_group.size()){
					if(MyPolicy.routeplan.segment_group[i]==MyPolicy.routeplan.segment_group[i+1])
						continue;

					if(BU::SuspectedLaneChaneGrpConn(MyPolicy.routeplan.segment_group[i],MyPolicy.routeplan.segment_group[i+1],MyPolicy)){ // This indicates a lane change. Lane change group origin DAR_m's should be the same. Don't update DAR_m
						std::string GrpPair = "{" + std::to_string(MyPolicy.routeplan.segment_group[i]) + "," + std::to_string(MyPolicy.routeplan.segment_group[i+1]) + " }";
						BU::codeRed( _CF_  + " SegGrpOriginDAR suspects lane change between {start, end } " + GrpPair," ",20);
						//	BU::codeRed( _CF_  + " and group ",MyPolicy.routeplan.segment_group[i+1],20);
						continue;
					}
				}
			}
			DAR += BehaviorUtils::getGrpLen(MyPolicy.routeplan.segment_group[i],MyPolicy);
			DAR += BehaviorUtils::getGrpConnLen(MyPolicy.routeplan.segment_group[i],MyPolicy.routeplan.segment_group[i+1],MyPolicy);
		}
	}
	return Candidate_DAR;
}

double Env::SegGrpEndDAR(int segUID,const Policy::DetailedPolicy & MyDetailedPolicy){
	map<int,double> AllGrpSegOriginDAR;
	return(MyDetailedPolicy.SegGrpOriginDAR[segUID]+BehaviorUtils::getGrpLen(segUID,MyDetailedPolicy.Policy));
}

double Env::SegGrpOriginDAR(int segUID, const Policy::DetailedPolicy & MyDetailedPolicy){
	std:: map<int,double> OriginDARs;
	return Env::SegGrpOriginDAR(segUID,MyDetailedPolicy.Policy,OriginDARs);
}

std::pair<InterSectionUidType,double> Env::FindClosestInterSectionInfo(int CurrentGrp){
	InterSectionUidType ClosestIntUID = Policy::BackEndDetailedPolicy.Policy.intersectionplan[CurrentGrp];
	double ClosestInterSectionDist = fabs(Env::SegGrpEndDAR(CurrentGrp,Policy::BackEndDetailedPolicy)-vehicle.DTAR);
	return std::make_pair(ClosestIntUID,ClosestInterSectionDist);
}

std::pair<InterSectionUidType,double> Env::FindClosestInterSectionInfo(Env::PolicyState& TargetState){
	InterSectionUidType ClosestIntUID = TargetState.InterSectionAhead;
	double ClosestInterSectionDist = fabs(TargetState.GrpEnd_DAR_m-vehicle.DTAR);
	return std::make_pair(ClosestIntUID,ClosestInterSectionDist);
}


double Env::LaneSegEndDAR(LaneSegUidType segUID,const Policy::DetailedPolicy & MyPolicy){
	double SegOrigin_DAR = Env::LaneSegOriginDAR(segUID,MyPolicy);
	if(SegOrigin_DAR==-1)
		return -1;
	return (SegOrigin_DAR+BehaviorUtils::getSegLen(segUID));
}

LaneSegUidType Env::FindNextLaneinRoute(LaneSegUidType lanesegUID, const Policy::InputPolicyType & MyPolicy){
	LaneSegUidType NextLaneSegUID = -1;

	if(MyPolicy.routeplan.segment_uid.back()==lanesegUID){ // This is the last lane in the route. Will try to return the primary next lane segment.
		exlcm::lanesegment_t Lane = BU::getLaneSeg(lanesegUID);
		int next_conn = Lane.primary_next_connection_n;
		NextLaneSegUID = Lane.connection[next_conn].next_laneseg_uid;
		return NextLaneSegUID;
	}


	int CurrentGrp = BU::LaneSegmentToRouteSegmentGrp(lanesegUID,MyPolicy);
	auto Alllanesegments = BU::routeSegmentGrpTolanesegments(CurrentGrp,MyPolicy);
	auto it = std::find(Alllanesegments.begin(),Alllanesegments.end(),lanesegUID);
	int NextGrp = Env::FindNextGrpinRoute(CurrentGrp,MyPolicy);

	if(Alllanesegments.size()==1){ // If only 1 segment in this group
		NextLaneSegUID = BU::routeSegmentGrpTolanesegments(NextGrp,MyPolicy).front();
	}
	else{
		auto Next_it = std::next(it);
		if(Next_it!=Alllanesegments.end()){
			NextLaneSegUID = *Next_it;
		}
		else{
			int ID;
			NextLaneSegUID = BU::routeSegmentGrpTolanesegments(NextGrp,MyPolicy).front();
			BU::getConn(lanesegUID,NextLaneSegUID,&ID);
			if(ID==-1)
				NextLaneSegUID = BU::routeSegmentGrpTolanesegments(NextGrp,MyPolicy).back();
		}
	}

	if(NextLaneSegUID==-1){
		cout<<"CurrentGrp "<<CurrentGrp<<" Alllanesegments "<<Alllanesegments;
	}
	return NextLaneSegUID;
}

LaneSegUidType Env::FindPrevLaneinRoute(LaneSegUidType lanesegUID,const Policy::InputPolicyType & MyPolicy){
	LaneSegUidType PrevLaneSegUID = -1;

	if(MyPolicy.routeplan.segment_uid.front()==lanesegUID){ // This is the First lane in the route. Will try to return the primary prrevious lane segment.
		if(behaviorinput::lanesegments.find(lanesegUID)==behaviorinput::lanesegments.end())
			return PrevLaneSegUID;
		exlcm::lanesegment_t Lane = BU::getLaneSeg(lanesegUID);
		int prev_conn = Lane.primary_prev_connection_n;
		PrevLaneSegUID = Lane.connection[prev_conn].next_laneseg_uid;
		return PrevLaneSegUID;
	}


	int CurrentGrp = BU::LaneSegmentToRouteSegmentGrp(lanesegUID,MyPolicy);
	auto Alllanesegments = BU::routeSegmentGrpTolanesegments(CurrentGrp,MyPolicy);
	auto it = std::find(Alllanesegments.begin(),Alllanesegments.end(),lanesegUID);
	if((Alllanesegments.size()==1)||(it==Alllanesegments.begin())) // If only 1 segment in this group
		PrevLaneSegUID = BU::routeSegmentGrpTolanesegments(Env::FindPrevGrpinRoute(CurrentGrp,MyPolicy),MyPolicy).back();
	else{


		if(it!=Alllanesegments.begin()){
			auto Prev_it = std::prev(it);
			PrevLaneSegUID = *Prev_it;
		}
		else{
			int ID;
			PrevLaneSegUID = BU::routeSegmentGrpTolanesegments(Env::FindPrevGrpinRoute(CurrentGrp,MyPolicy),MyPolicy).back();
			BU::getConn(PrevLaneSegUID,lanesegUID,&ID);
			if(ID==-1)
				PrevLaneSegUID = BU::routeSegmentGrpTolanesegments(Env::FindNextGrpinRoute(CurrentGrp,MyPolicy),MyPolicy).front();
		}
	}

	return PrevLaneSegUID;
}

int Env::FindNextGrpinRoute(int segUID, const Policy::InputPolicyType & MyPolicy){
	int NextSegUID = -1;

	auto it = std::find(MyPolicy.routeplan.segment_group.begin(),MyPolicy.routeplan.segment_group.end(),segUID);
	auto itr = it;
	for(itr=it;itr!=MyPolicy.routeplan.segment_group.end();itr++){
		if(*itr!=*it)
			break;
	}
	if(itr!=MyPolicy.routeplan.segment_group.end()){
		NextSegUID = *itr;
	}

	return NextSegUID;
}

int Env::FindPrevGrpinRoute(int segUID, const Policy::InputPolicyType & MyPolicy){
	int PrevSegUID = -1;

	auto it = std::find(MyPolicy.routeplan.segment_group.begin(),MyPolicy.routeplan.segment_group.end(),segUID);
	auto itr = it;
	for(itr=it;itr!=MyPolicy.routeplan.segment_group.begin();itr--){
		if(*itr!=*it)
			break;
	}


	PrevSegUID = *itr;

	return PrevSegUID;
}


template <typename T>
BehaviorUtils::MyStreamingHelper& BehaviorUtils::operator<<(BehaviorUtils::MyStreamingHelper& h, T const& t)
{
	h.out1_ << t;
	h.out2_ << t;
	return h;
}
BehaviorUtils::MyStreamingHelper& BehaviorUtils::MyStreamingHelper::operator=(const MyStreamingHelper& rhs)
{
	MyStreamingHelper temp(rhs);
	return temp;
}

BehaviorUtils::MyStreamingHelper& BehaviorUtils::operator<<(BehaviorUtils::MyStreamingHelper& h, std::ostream&(*f)(std::ostream&))
{
	h.out1_ << f;
	h.out2_ << f;
	return h;
}

