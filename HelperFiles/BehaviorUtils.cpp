#include "../PCH/pch.hpp"
#include "../include/BehaviorUtils.hpp"
#include "../include/BehaviorObject.hpp"
#include "../include/Env.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"


std::string BehaviorUtils::_ColorCode;

pthread_mutex_t codeRBG_mutex     = PTHREAD_MUTEX_INITIALIZER;
namespace BU = BehaviorUtils;

vector<LaneSegUidType> BehaviorUtils::MapletRequestMissingLanePub(const exlcm::routesegmentlist_t & routeplan){
	vector<LaneSegUidType> MissingLaneSegs;
	for(auto lane:routeplan.segment_uid){
		if(BehaviorUtils::MapletRequestMissingLanePub(lane))
			MissingLaneSegs.push_back(lane);
	}
	return MissingLaneSegs;
}

bool BehaviorUtils::MapletRequestMissingLanePub(LaneSegUidType LastLaneSegmentUID){
	if(LastLaneSegmentUID<=0)
		return true;
	static exlcm::mapletrequest_t AlternatePlanMissingLaneRequest; // Static preserves the reference for lcm pub
	if(behaviorinput::lanesegments.find(LastLaneSegmentUID)==behaviorinput::lanesegments.end()){ // behaviorinput::lanesegments contains all the parsed lane segments.
		AlternatePlanMissingLaneRequest.valid_f = 1;
		AlternatePlanMissingLaneRequest.request_type = 3;
		AlternatePlanMissingLaneRequest.requested_item_uid = LastLaneSegmentUID;
		AlternatePlanMissingLaneRequest.timestamp_sec = BehaviorUtils::SysTimeNow();
		//if (LANESEG_REQUEST_ENABLED)
			//lcm1.publish("MAPLETREQUEST",&AlternatePlanMissingLaneRequest);
		return true;
	}
	return false;
}

std::vector<double> BehaviorUtils::ExtractParamFromObjSA(int uid, std::string param){
	std::vector<double> MyVector = {};
	std::vector<double> ZeroVector ;
	ZeroVector.push_back((double)0.0);
	if(uid<=0)
		return ZeroVector;
	if(SA.objData.find(uid)==SA.objData.end())
		return ZeroVector;

	if(SA.objData[uid].valid_pred_count==0)
		return ZeroVector;


	for(int i=0;i<SA.objData[uid].valid_pred_count;i++){

		if(param=="vel")
			MyVector.push_back(SA.objData[uid].pred_obj_SA[i].in_lane_speed_mps);
		else if(param=="accel")
			MyVector.push_back(SA.objData[uid].pred_obj_SA[i].in_lane_accel_mpss);
		else if(param=="lane")
			MyVector.push_back(SA.objData[uid].pred_obj_SA[i].lane_segment_uid);
		else if(param=="pred")
			MyVector.push_back(SA.objData[uid].pred_obj_probability[i]);
		else if(param=="intersection_uid")
			MyVector.push_back(SA.objData[uid].pred_obj_SA[i].intersection_uid);
		else if(param=="stationary_status")
			MyVector.push_back(SA.objData[uid].pred_obj_SA[i].Stationary_Status);

	}



	if(MyVector.empty()){
		cout<<" MyVector.empty in BU::ExtractParamFromObjSA "<<SA.objData[uid];
		MyVector = ZeroVector;
	}


	return MyVector;
}


bool BehaviorUtils::SegExistsInThisGroup(const Policy::InputPolicyType & MyPolicy, LaneSegUidType laneseg_uid , int ThisGrp ){
	auto AllLaneSegInThisGrp = BehaviorUtils::routeSegmentGrpTolanesegments(ThisGrp,MyPolicy);
	return std::find(std::begin(AllLaneSegInThisGrp),std::end(AllLaneSegInThisGrp), laneseg_uid) != std::end(AllLaneSegInThisGrp);
}

bool BehaviorUtils::SegExists(const Policy::InputPolicyType & MyPolicy, LaneSegUidType laneseg_uid){
	return std::find(std::begin(MyPolicy.routeplan.segment_uid),std::begin(MyPolicy.routeplan.segment_uid)+MyPolicy.routeplan.segment_count, laneseg_uid) != \
			std::begin(MyPolicy.routeplan.segment_uid)+MyPolicy.routeplan.segment_count;
}


bool BehaviorUtils::SegExists(const Policy::InputPolicyType & MyPolicy,std::vector<LaneSegUidType> laneseg_uids){
	if(laneseg_uids.empty())
		return false;
	for(auto uid:laneseg_uids){
		if(!BehaviorUtils::SegExists(MyPolicy,uid))
			return false;
	}
	return true;
}

bool BehaviorUtils::GrpExists(int MyGrp, const Policy::InputPolicyType & MyPolicy){
	auto RouteGrp = MyPolicy.routeplan.segment_group;
	return (std::find(RouteGrp.begin(),RouteGrp.end(),MyGrp)!=RouteGrp.end());

}




int BehaviorUtils::LaneSegmentToRouteSegmentGrp(LaneSegUidType LaneSegUID, const Policy::InputPolicyType& MyPolicy){
	int LaneSegIdx;
	auto LaneSegitr= std::find(std::begin(MyPolicy.routeplan.segment_uid), std::end(MyPolicy.routeplan.segment_uid), LaneSegUID);
	if (LaneSegitr!= std::end(MyPolicy.routeplan.segment_uid)){
		LaneSegIdx = std::distance(MyPolicy.routeplan.segment_uid.begin(), LaneSegitr) ;
		return MyPolicy.routeplan.segment_group[LaneSegIdx];
	}

	int PossibleGrp=LaneSegUID;
	for(int i=0;i<MyPolicy.routeplan.segment_group.size();i++){
		if(LaneSegUID==MyPolicy.routeplan.segment_uid[i]){
			PossibleGrp = MyPolicy.routeplan.segment_group[i];
		}
	}

	return PossibleGrp;
}

std::vector<LaneSegUidType> BehaviorUtils::routeSegmentGrpTolanesegments(int GrpValue, const Policy::InputPolicyType& MyPolicy){
	std::vector<LaneSegUidType> AllLanesInThisGrp = {};
	int GrpIdx;
	auto Grpitr= std::find(std::begin(MyPolicy.routeplan.segment_group), std::end(MyPolicy.routeplan.segment_group), GrpValue);
	if (Grpitr!= std::end(MyPolicy.routeplan.segment_group))
		GrpIdx = std::distance(MyPolicy.routeplan.segment_group.begin(), Grpitr) ;
	else
	return AllLanesInThisGrp;

	int LaneSegIdx = GrpIdx;
	do{
		if(BU::SegExists(MyPolicy,MyPolicy.routeplan.segment_uid[LaneSegIdx])){
			AllLanesInThisGrp.push_back(MyPolicy.routeplan.segment_uid[LaneSegIdx]);
		}
		LaneSegIdx++;
		if(LaneSegIdx>=MyPolicy.routeplan.segment_group.size())
			break;
	}while(MyPolicy.routeplan.segment_group[LaneSegIdx]==MyPolicy.routeplan.segment_group[GrpIdx]);
	return AllLanesInThisGrp;
}


std::vector<exlcm::lanesegment_t> BehaviorUtils::getLaneSeg(std::vector<LaneSegUidType> uids){
	std::vector<exlcm::lanesegment_t> segs;
	for(LaneSegUidType i=0;i<uids.size();i++){
		segs.push_back(BehaviorUtils::getLaneSeg(uids[i]));
	}
	return segs;
}


exlcm::lanesegment_t BehaviorUtils::getLaneSeg(LaneSegUidType uid)
{

	BU::MapletRequestMissingLanePub(uid);
	if(behaviorinput::lanesegments.find(uid)!=behaviorinput::lanesegments.end())
		return behaviorinput::lanesegments[uid];
	bool segExists = BehaviorUtils::SegExists(Policy::BackEndDetailedPolicy.Policy,uid);
	if (segExists){
		BU::codeRed( _CF_  + "!!!Possible lanesegment pub error.\n In function getLaneSeg not finding within the route segment list UID = ",uid,1);

	}
	else{
		BU::codeRed( _CF_  + "Critical error. In  function getLaneSeg not finding within the route segment list UID = ",uid,1);
	}
	exlcm::lanesegment_t garbage_lane;
	garbage_lane.uid = -1;
	garbage_lane.intersection_uid = -1;
	garbage_lane.valid_f = 0;
	return garbage_lane;

}

exlcm::intersection_t BehaviorUtils::getInterSection(InterSectionUidType uid){
	if(behaviorinput::IntersectionList.find(uid)!=behaviorinput::IntersectionList.end())
		return behaviorinput::IntersectionList[uid];
	BU::codeRed( _CF_  + "Critical error. In  function getInterSection not finding the intersection UID = ",uid,1);
	exlcm::intersection_t garbage_intersection;
	garbage_intersection.uid = -1;
	garbage_intersection.valid_f = 0;
	return garbage_intersection;
}

double BehaviorUtils::getSegLen(LaneSegUidType uid)
{
	double segLen=0;
	exlcm::lanesegment_t seg=BehaviorUtils::getLaneSeg(uid);
	return seg.length_m;
	for (int j=1;j<seg.nominal_path_waypoint_count;j++)
	{
		double dn=seg.nominal_path_waypoint[j].north_m-seg.nominal_path_waypoint[j-1].north_m;
		double de=seg.nominal_path_waypoint[j].east_m-seg.nominal_path_waypoint[j-1].east_m;
		segLen+=sqrt(dn*dn+de*de);
	}
	return segLen;
}


double BehaviorUtils::getGrpLen(int uid, const Policy::InputPolicyType & MyPolicy)
{
	double GrpLen=0;
	std::vector<LaneSegUidType> AllLanesInThisGrp= BehaviorUtils::routeSegmentGrpTolanesegments(uid,MyPolicy);
	if(AllLanesInThisGrp.size()==1){
		return BehaviorUtils::getSegLen(AllLanesInThisGrp[0]);
	}
	else if(AllLanesInThisGrp.size()==0){
		return -1;
	}
	else{
		GrpLen = BehaviorUtils::getSegLen(AllLanesInThisGrp[0]);
		for(int i=1;i<AllLanesInThisGrp.size();i++){
			GrpLen+= BehaviorUtils::getSegLen(AllLanesInThisGrp[i]);
			GrpLen+= BehaviorUtils::getConnLen(AllLanesInThisGrp[i-1],AllLanesInThisGrp[i]);
		}
	}
	return GrpLen;
}

double BehaviorUtils::getGrpLen(const std::vector<LaneSegUidType>& AllLanesInThisGrp){

	double GrpLen=0;
	if(AllLanesInThisGrp.size()==1){
		return BehaviorUtils::getSegLen(AllLanesInThisGrp[0]);
	}
	else if(AllLanesInThisGrp.size()==0){
		return -1;
	}
	else{
		GrpLen = BehaviorUtils::getSegLen(AllLanesInThisGrp[0]);
		for(int i=1;i<AllLanesInThisGrp.size();i++){
			GrpLen+= BehaviorUtils::getSegLen(AllLanesInThisGrp[i]);
			GrpLen+= BehaviorUtils::getConnLen(AllLanesInThisGrp[i-1],AllLanesInThisGrp[i]);
		}
	}
	return GrpLen;
}

bool BehaviorUtils::SuspectedOpposingLaneGrp(int GrpID,const Policy::InputPolicyType & MyPolicy){
	int PrevToCurrIdx, CurrToNextIdx;
	int NextGrpID = Env::FindNextGrpinRoute(GrpID,MyPolicy);
	int PrevGrpID =  Env::FindPrevGrpinRoute(GrpID,MyPolicy);
	if(NextGrpID<=0)
		return false;
	if(PrevGrpID<=0)
		return false;
	BehaviorUtils::getGrpConn(GrpID,NextGrpID,MyPolicy,&CurrToNextIdx);
	BehaviorUtils::getGrpConn(PrevGrpID,GrpID,MyPolicy,&PrevToCurrIdx);
	if((CurrToNextIdx>=0)&&(PrevToCurrIdx>=0)){
		exlcm::connection_t PrevToCurr,CurrToNext;
		PrevToCurr = BehaviorUtils::getGrpConn(PrevGrpID,GrpID,MyPolicy,&PrevToCurrIdx);
		CurrToNext = BehaviorUtils::getGrpConn(GrpID,NextGrpID,MyPolicy,&CurrToNextIdx);
		if((PrevToCurr.connection_type==PARALLEL_OPPOSING_CONNECTION)&&(CurrToNext.connection_type==PARALLEL_OPPOSING_CONNECTION)){
			return true;
		}
	}
	return false;
}


bool BehaviorUtils::SuspectedLaneChaneGrpConn( std::vector<LaneSegUidType>& AllLanesInStartGrp, std::vector<LaneSegUidType>& AllLanesInEndGrp){
	int connBackToBackIdx, connBackToFrontIdx;
	exlcm::connection_t connBackToBack,connBackToFront;

	if((AllLanesInStartGrp.empty())||AllLanesInEndGrp.empty())
		return false;
	BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.front(),&connBackToFrontIdx);
	BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.back(),&connBackToBackIdx);
	if(connBackToFrontIdx>=0){
		connBackToFront = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.front());
		int ConnType = connBackToFront.connection_type;
		return ( (ConnType == PARALLEL_LEFT_ADJ_CONNECTION) || (ConnType == PARALLEL_RIGHT_ADJ_CONNECTION) || (ConnType == PARALLEL_OPPOSING_CONNECTION));
	}
	if(connBackToBackIdx>=0){
		connBackToBack = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.back());
		int ConnType = connBackToBack.connection_type;
		return ( (ConnType == PARALLEL_LEFT_ADJ_CONNECTION) || (ConnType == PARALLEL_RIGHT_ADJ_CONNECTION) || (ConnType == PARALLEL_OPPOSING_CONNECTION));
	}

	return false;
}

bool BehaviorUtils::SuspectedLaneChaneGrpConn(int startUid,int endUid,const Policy::InputPolicyType & MyPolicy){
	if(startUid==endUid)
		return false;
	std::vector<LaneSegUidType> AllLanesInStartGrp= BehaviorUtils::routeSegmentGrpTolanesegments(startUid,MyPolicy);
	std::vector<LaneSegUidType> AllLanesInEndGrp= BehaviorUtils::routeSegmentGrpTolanesegments(endUid,MyPolicy);
	return BehaviorUtils::SuspectedLaneChaneGrpConn(AllLanesInStartGrp,AllLanesInEndGrp);

}

exlcm::connection_t BehaviorUtils::getGrpConn(int startUid,int endUid, const Policy::InputPolicyType & MyPolicy,int *ID=0){
	auto AllLanesInStartGrp= BehaviorUtils::routeSegmentGrpTolanesegments(startUid,MyPolicy);
	auto AllLanesInEndGrp= BehaviorUtils::routeSegmentGrpTolanesegments(endUid,MyPolicy);
	auto  BackToFrontConnection = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.front(),ID);
	if(ID==-1){
		auto  BackToBackConnection = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.back(),ID);
		return BackToBackConnection;
	}
	else{
		return BackToFrontConnection;
	}
	return BackToFrontConnection;
}

exlcm::connection_t BehaviorUtils::getGrpConn(const std::vector<LaneSegUidType>& AllLanesInStartGrp,const std::vector<LaneSegUidType>& AllLanesInEndGrp, int *ID=0){

	auto  BackToFrontConnection = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.front(),ID);
	if(ID==-1){
		auto  BackToBackConnection = BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.back(),ID);
		return BackToBackConnection;
	}
	else{
		return BackToFrontConnection;
	}
	return BackToFrontConnection;
}


double BehaviorUtils::getGrpConnLen(int startUid,int endUid,const Policy::InputPolicyType & MyPolicy){
	auto AllLanesInStartGrp= BehaviorUtils::routeSegmentGrpTolanesegments(startUid,MyPolicy);
	auto AllLanesInEndGrp= BehaviorUtils::routeSegmentGrpTolanesegments(endUid,MyPolicy);
	return BehaviorUtils::getConnLen(AllLanesInStartGrp.back(),AllLanesInEndGrp.front());
	int ID;
	BehaviorUtils::getConn(AllLanesInStartGrp.back(),AllLanesInEndGrp.front(),&ID);
	if(ID==-1){
		auto  BackToBackConnectionLen = BehaviorUtils::getConnLen(AllLanesInStartGrp.back(),AllLanesInEndGrp.back());
		return BackToBackConnectionLen;
	}
	else{
		auto  BackToFrontConnectionLen = BehaviorUtils::getConnLen(AllLanesInStartGrp.back(),AllLanesInEndGrp.front());
		return BackToFrontConnectionLen;
	}
	return -1;

}



//Argument ID is optional
exlcm::connection_t BehaviorUtils::getConn(LaneSegUidType startUid,LaneSegUidType endUid, int *ID=0)
{
	const double TimeGap= 25;
	int connId=-1;
	if((startUid<=0)||(endUid<=0)){
		BehaviorUtils::codeRed( _CF_  + "gdb debug as start id is ",startUid,1);
		BehaviorUtils::codeRed( _CF_  + "gdb debug as end id is ",endUid,1);
	}
	exlcm::lanesegment_t start=BehaviorUtils::getLaneSeg(startUid);
	for(int i=0;i<start.connection_count;i++)
	{
		if(start.connection[i].next_laneseg_uid==endUid)
		{
			connId=i;
			if(ID)// If the arguement ID is supplied
				*ID=connId;

			break;
		}
	}
	//TODO: Handle the case when conId cant be matched/for loop just exits
	if(connId==-1)
	{
		if(ID) // If the arguement ID is supplied
			*ID = -1;
		BehaviorUtils::codeRed(_CF_+ "ERROR in function getConn:No connection from segment with uid=",startUid,TimeGap);
		BehaviorUtils::codeRed( _CF_  + " to the target segment with uid = ",endUid,TimeGap);
		BehaviorUtils::codeGreen( _CF_ + "vehicle.curSegmentUid ",vehicle.curSegmentUid,TimeGap);
		BehaviorUtils::codeGreen( _CF_ + "vehicle.targetSegmentUid ",vehicle.targetSegmentUid,TimeGap);
		BehaviorUtils::codeGreen( _CF_ +  "vehicle.curConnOriginSegUid ",vehicle.curConnOriginSegUid,TimeGap);
		BehaviorUtils::codeGreen( _CF_ + "vehicle.curConnDestinationSegUid ",vehicle.curConnDestinationSegUid,TimeGap);
		return start.connection[0];
		//throw std::runtime_error("runtime_error");
		//	exit(0);
	}
	return start.connection[connId];
}


double BehaviorUtils::getConnLen(LaneSegUidType startUid,LaneSegUidType endUid)
{
	double connLen=0;
	try {
		exlcm::connection_t conn=BehaviorUtils::getConn(startUid,endUid);
		if(conn.point_count!=0){
			double OriginLaneStation = BehaviorUtils::getSegLen(startUid);
			connLen = conn.point.back().station_m-OriginLaneStation;
			/*
			for (int j=1; j<conn.point_count;j++)
			{
				double dn=conn.point[j].north_m-conn.point[j-1].north_m;
				double de=conn.point[j].east_m-conn.point[j-1].east_m;
				connLen+=sqrt(dn*dn+de*de);
			}
			*/
		}
		else{
			connLen = 0;
		}
	} catch(...) {
		if (startUid==0){
			BehaviorUtils::codeRed( _CF_  + "Possible getConn Error in initialization as connection startUid is 0. Continuing... connection endUID =  ",endUid,2.5);
			return (0);
		}
		else{
			BehaviorUtils::codeRed( _CF_  + " getConn Error in function getConnLen"," ",10);
			//throw std::runtime_error("runtime_error");
		}
	}

	return fabs(connLen);
}


// Eulcidean distance between two Point_t
double BehaviorUtils::pDist(Point_t p1, Point_t p2)
{
	auto deltaX2 = (p2.getX()-p1.getX())*(p2.getX()-p1.getX());
	auto deltaY2 = (p2.getY()-p1.getY())*(p2.getY()-p1.getY());
	return sqrt(deltaX2+deltaY2);
}



std::string  CodeRBGFileName ="printCodeRBG.txt";


void CodeRBGFileWrite(std::string msg)
{

	std::ofstream CodeRBGFile(CodeRBGFileName,ios_base::app);
	cout<<bold_on<<color_on<<setprecision(9)<<msg<<color_off<<bold_off<<endl;
	CodeRBGFile<<msg<<endl;
}



extern ClockType SysTimer;

double BehaviorUtils::TimeNow(){
	SysTimer = chrono::high_resolution_clock::now();
	return behaviorinput::hvLoc.timestamp_sec;
}

double BehaviorUtils::SysTimeNow(){
	auto SysTimeNow = chrono::high_resolution_clock::now() ;
	auto SysTimeDiff = chrono::duration_cast<chrono::milliseconds>(SysTimeNow-SysTimer);
	auto SysTimeDiff_sec = SysTimeDiff.count()/1e3;
	return (BehaviorUtils::TimeNow()+SysTimeDiff_sec);
}

double BU::ModuleTimeNow(){
	static double initTime  = BU::TimeNow();
	return (BU::TimeNow()-initTime);
}




bool BehaviorUtils::codeRed( std::string msg, std::string ignore, float TimeGap , double *Timer ){
	return BehaviorUtils::codeRBG(msg,	BehaviorUtils::defaultvalue,TimeGap,"R",Timer);
}

bool BehaviorUtils::codeGreen(std::string msg, std::string ignore ,float TimeGap ,  double *Timer ){
	return BehaviorUtils::codeRBG(msg,	BehaviorUtils::defaultvalue,TimeGap,"G",Timer);
}

bool  BehaviorUtils::codeBlue(std::string msg, std::string ignore , float TimeGap ,  double *Timer ){
	return BehaviorUtils::codeRBG(msg,	BehaviorUtils::defaultvalue,TimeGap,"B",Timer);
}

bool  BehaviorUtils::codeYellow(std::string msg, std::string ignore , float TimeGap ,  double *Timer ){
	return BehaviorUtils::codeRBG(msg,	BehaviorUtils::defaultvalue,TimeGap,"Y",Timer);
}

bool  BehaviorUtils::codePurple(std::string msg, std::string ignore , float TimeGap,  double *Timer ){
	return BehaviorUtils::codeRBG(msg,	BehaviorUtils::defaultvalue,TimeGap,"P",Timer);
}


bool BehaviorUtils::codeRed( std::string msg){
	double Timer;
	int64_t value = BehaviorUtils::defaultvalue;
	return BehaviorUtils::codeRed( msg,value,CodeRed_TimeGap,&Timer);
}
bool BehaviorUtils::codeGreen(std::string msg){
	double Timer;
	int64_t value = BehaviorUtils::defaultvalue;
	return BehaviorUtils::codeGreen( msg,value,CodeGreen_TimeGap,&Timer);
}
bool BehaviorUtils::codeBlue(std::string msg){
	double Timer;
	int64_t value = BehaviorUtils::defaultvalue;
	return BehaviorUtils::codeBlue( msg,value,CodeBlue_TimeGap,&Timer);
}
bool BehaviorUtils::codeYellow(std::string msg){
	double Timer;
	int64_t value = BehaviorUtils::defaultvalue;
	return BehaviorUtils::codeYellow( msg,value,CodeBlue_TimeGap,&Timer);
}
bool BehaviorUtils::codePurple(std::string msg){
	double Timer;
	int64_t value = BehaviorUtils::defaultvalue;
	return BehaviorUtils::codePurple( msg,value,CodeBlue_TimeGap,&Timer);
}


template<typename pointT>
bool BehaviorUtils::is_lessThanZero(pointT cp)
{
	return cp.x_m;

}


/////
Point_t BehaviorUtils::getENLocOfSegmentStation(LaneSegUidType segUID,int stationD_m)
{
	auto  LS=BehaviorUtils::getLaneSeg(segUID);

	Point_t start=	Point(LS.nominal_path_waypoint[0].east_m,LS.nominal_path_waypoint[0].north_m);

	Point_t end=	Point(LS.nominal_path_waypoint[LS.nominal_path_waypoint_count-1].east_m,LS.nominal_path_waypoint[LS.nominal_path_waypoint_count-1].north_m);

	Point_t station=((end-start)/BehaviorUtils::getSegLen(segUID))*stationD_m;

	return station;

}





std::pair<double,exlcm::intersection_t> BehaviorUtils::ClosestIntersection(Point CurrenthvLoc) // North , East
{
	exlcm::intersection_t Closest_InterSection;
	double ClosestIntersectionDistance = std::numeric_limits<double>::max();
	for(auto const &intsc:behaviorinput::IntersectionList){
		Point IntersectionLoc(intsc.second.center_north_m,intsc.second.center_east_m);
		if (BehaviorUtils::pDist(CurrenthvLoc,IntersectionLoc)<ClosestIntersectionDistance){
			ClosestIntersectionDistance = BehaviorUtils::pDist(CurrenthvLoc,IntersectionLoc);
			Closest_InterSection = intsc.second;
		}
	}
	return std::make_pair(ClosestIntersectionDistance,Closest_InterSection);
}

std::tuple<double,exlcm::intersection_t> BehaviorUtils::ClosestIntersectionAhead(Point CurrenthvLoc){

	exlcm::intersection_t Closest_InterSection;
	double ClosestIntersectionDistance = std::numeric_limits<double>::max();
	for(auto const &intsc:behaviorinput::IntersectionList){
		Point IntersectionLoc(intsc.second.center_north_m,intsc.second.center_east_m);
		if (BehaviorUtils::pDist(CurrenthvLoc,IntersectionLoc)<ClosestIntersectionDistance){
			for(int i=0;i<intsc.second.connection_count;i++){
				if(intsc.second.connection[i].incoming_laneseg_uid==vehicle.curConnOriginSegUid){
					ClosestIntersectionDistance = BehaviorUtils::pDist(CurrenthvLoc,IntersectionLoc);
					Closest_InterSection = intsc.second;
				}
			}
		}

	}

	return std::make_pair(ClosestIntersectionDistance,Closest_InterSection);
}

std::tuple<double,Point,double> BehaviorUtils::GetExtremeCurvature(const exlcm::lanesegment_t& TargetSegment){
	double MaxCurv = 0;
	int LaneSegPt_count=1;
	Point NE_AtMaxCurvature;
	static LaneSegUidType prev_UID;
	static double prev_MaxCurv;
	static Point prev_NE_AtMaxCurvature;
	double FirstStationAheadOfHighCurvature = 0;


	double distAlongLaneSeg = 0;
	double DistBetweenWayPoints = 0;
	Point prev_LaneSegPt = Point(TargetSegment.nominal_path_waypoint[0].north_m,TargetSegment.nominal_path_waypoint[0].east_m);
	for(auto & LaneSegPt:TargetSegment.nominal_path_waypoint){

		// Get the first point on the target segment which is more that the curvature threshold
		if (fabs(LaneSegPt.curve_im)>KAPPA_LOWER_THRESHOLD_FOR_ROAD_BENDS){
			if (FirstStationAheadOfHighCurvature==0)
				FirstStationAheadOfHighCurvature = distAlongLaneSeg ;
		}

		if(fabs(LaneSegPt.curve_im)>fabs(MaxCurv)){
			if(vehicle.curSegmentUid==TargetSegment.uid){  // Ignore the points already traversed through current segment
				if(vehicle.DTACS<distAlongLaneSeg){
					MaxCurv = LaneSegPt.curve_im;
					NE_AtMaxCurvature = Point(LaneSegPt.north_m,LaneSegPt.east_m);
				}

			}
			else{
				MaxCurv = LaneSegPt.curve_im;
				NE_AtMaxCurvature = Point(LaneSegPt.north_m,LaneSegPt.east_m);
			}
		}


		if (LaneSegPt_count>1){
			DistBetweenWayPoints = pDist(prev_LaneSegPt,Point(LaneSegPt.north_m,LaneSegPt.east_m));
			distAlongLaneSeg += DistBetweenWayPoints;
		}

		prev_LaneSegPt = Point(LaneSegPt.north_m,LaneSegPt.east_m);
		LaneSegPt_count++;
		if (LaneSegPt_count>TargetSegment.nominal_path_waypoint_count)
			break;
	} // end for

	prev_UID = TargetSegment.uid;
	prev_MaxCurv = MaxCurv;
	prev_NE_AtMaxCurvature = NE_AtMaxCurvature;
	return std::make_tuple(MaxCurv,NE_AtMaxCurvature,FirstStationAheadOfHighCurvature);
}

std::tuple<double,Point,double> BehaviorUtils::GetExtremeCurvature(const exlcm::connection_t& TargetConnection){
	double MaxCurv = 0;
	Point NE_AtMaxCurvature;
	static int64_t prev_UID;
	static double prev_MaxCurv;
	static Point prev_NE_AtMaxCurvature;
	/*	if(TargetConnection.next_laneseg_uid == prev_UID){
		return std::make_tuple(prev_MaxCurv,prev_NE_AtMaxCurvature,1e6);
	} */
	for (auto & connPt:TargetConnection.point)
	{
		if(connPt.east_m!=0.0 && connPt.north_m!=0.0)//Default-ly constructed points.Typically the ones that are after the valid conn point count
		{
			if (fabs(connPt.curve_im)> fabs(MaxCurv)){
				MaxCurv = connPt.curve_im;
				NE_AtMaxCurvature = Point(connPt.north_m,connPt.east_m);
			}
		}
	}
	prev_UID = TargetConnection.next_laneseg_uid ;
	prev_MaxCurv = MaxCurv;
	prev_NE_AtMaxCurvature = NE_AtMaxCurvature;
	return std::make_tuple(MaxCurv,NE_AtMaxCurvature,1e6);
}

BehaviorUtils::MyStreamingHelper::MyStreamingHelper(std::ostream& out1,
		std::ostream& out2) : out1_(out1), out2_(out2) {}

BehaviorUtils::MyStreamingHelper::MyStreamingHelper(const MyStreamingHelper & src) : out1_(src.out1_), out2_(src.out2_) {}


BehaviorUtils::MyStreamingHelper::MyStreamingHelper() {}




std::ostream& color_on(std::ostream& os)
{
	if(BehaviorUtils::_ColorCode=="R")
		os << red_on ;
	else if (BehaviorUtils::_ColorCode=="G")
		os << green_on ;
	else if (BehaviorUtils::_ColorCode=="B")
		os << blue_on ;
	else if (BehaviorUtils::_ColorCode=="P")
		os << purple_on ;
	else if (BehaviorUtils::_ColorCode=="Y")
		os << yellow_on ;
	return os ;

}

double BehaviorUtils::linearinterp(double a, double b, double f)
{
	return a + f * (b - a);
}

double BehaviorUtils::LookUp1DMap(map<double,double> ValueMap, double XValue){

	if (XValue == -999)
		return 1;

	if ((XValue<= ValueMap.begin()->first)){
		return ValueMap.begin()->second;
	}

	if (XValue >= ValueMap.rbegin()->first){
		//		BehaviorUtils::codeRed( _CF_  + "XValue more than last index value  ",ValueMap.rbegin()->first);
		//		BehaviorUtils::codeRed( _CF_  + "XValue.. = ",XValue,1.0);
		return ValueMap.rbegin()->second;
	}

	auto it = ValueMap.find(XValue);
	if (it != ValueMap.end()){
		//			BehaviorUtils::codeRed( _CF_  + "XValue exactly equal to  index value  ",ValueMap.rbegin()->first);
		//			BehaviorUtils::codeRed( _CF_  + "XValue... = ",XValue,1.0);
		return it->second;
	}

	map<double,double>::iterator it1,it2;

	std::tie(it1,it2) = ValueMap.equal_range(XValue);
	it1--;
	auto returnvalue = linearinterp(it1->second,it2->second,(XValue-(it1->first))/((it2->first) - (it1->first)));

	return returnvalue;

}

pthread_mutex_t mutex_obj=PTHREAD_MUTEX_INITIALIZER;
void BehaviorUtils::CleanUpBuffer_SA(){

	for (auto &obj:SA.objData){
		ObjUidType obj_UID = obj.second.uid;
		if((SA_IN.objDataUpdateClock.find(obj_UID)!=SA_IN.objDataUpdateClock.end())&&(SA_IN.objData.find(obj_UID)!=SA_IN.objData.end())){
			double TimeDiff = 1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-SA_IN.objDataUpdateClock[obj_UID]).count());
			if (TimeDiff > SA_CLEANUP_TIME_THRSH){
				SA_IN.objDataUpdateClock.erase(obj_UID);
				SA_IN.objData.erase(obj_UID);
			}
		}
	}

}

void BehaviorUtils::CleanUpBuffer_AltPlans(){


	for(auto &policy:Policy::AllaltPolicies){
		double TimeDiff;

		if((behaviorinput::AllaltpathPlan_IN_LastUpdateTime.find(policy.first)!=behaviorinput::AllaltpathPlan_IN_LastUpdateTime.end())&&(behaviorinput::AllaltpathPlan_IN.find(policy.first)!=behaviorinput::AllaltpathPlan_IN.end())){
			TimeDiff = 1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>\
					(std::chrono::high_resolution_clock::now()-behaviorinput::AllaltpathPlan_IN_LastUpdateTime[policy.first]).count());
			if (TimeDiff > ALTPOLICIES_CLEANUP_TIME_THRSH){
				behaviorinput::AllaltpathPlan_IN_LastUpdateTime.erase(policy.first);
				behaviorinput::AllaltpathPlan_IN.erase(policy.first);
			}
		}

		if((behaviorinput::AllaltrouteSegmentList_IN_LastUpdateTime.find(policy.first)!=behaviorinput::AllaltrouteSegmentList_IN_LastUpdateTime.end())&&\
				(behaviorinput::AllaltrouteSegmentList_IN.find(policy.first)!=behaviorinput::AllaltrouteSegmentList_IN.end())){
			TimeDiff = 1e-3*(std::chrono::duration_cast<std::chrono::milliseconds>\
					(std::chrono::high_resolution_clock::now()-behaviorinput::AllaltrouteSegmentList_IN_LastUpdateTime[policy.first]).count());
			if (TimeDiff > ALTPOLICIES_CLEANUP_TIME_THRSH ){
				behaviorinput::AllaltrouteSegmentList_IN_LastUpdateTime.erase(policy.first);
				behaviorinput::AllaltrouteSegmentList_IN.erase(policy.first);
			}
		}
	}

}



