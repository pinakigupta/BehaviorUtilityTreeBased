#pragma once
#include "../PCH/pch.hpp"
#include "StackTrace.hpp"
#include "lcmprint.hpp"
#include "behaviorHorizon.hpp"
#include "BehaviorCals.hpp"
#include "BehaviorStatic.hpp"
#include "Point.hpp"
#define PI 3.14159
#define Point_t Point
#define SEC_TO_MICROSEC 1e6
#define NO_DEBUG_PRINT 1
#define STRINGIFY(x) #x
#define __FILENAME__ (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)
#define _CF_   (std::string(__FILENAME__)+" : "+std::to_string(__LINE__)+" : ") // std::string(BOOST_CURRENT_FUNCTION) // Current File


#include "lcmprint.hpp"

typedef Point Point_t;
extern std::string  CodeRBGFileName;
void CodeRBGFileWrite(std::string msg);
extern pthread_mutex_t codeRBG_mutex ;

inline void breakpoint(){
	return;
}

namespace BehaviorUtils
{

inline void DebugObjExit(ObjUidType ObjUID, ObjUidType uid){
	if(ObjUID==uid)
		exit(1);
}

template <class T,class U>
inline bool find(const std::vector<T>& MyVec, const U& MyElement ){
	if(MyVec.empty())
		return false;
	return (std::find(MyVec.begin(),MyVec.end(),(T)MyElement)!=MyVec.end());
}

template <class T,class U>
inline bool find(const std::vector<T>& MyVec, const std::vector<U>& MySearchVec ){
	for(auto searchElem:MySearchVec){
		if(BehaviorUtils::find(MyVec,searchElem))
			return true;
	}
	return false;
}

template <class T,class U,class V>
inline bool find(const std::map<U,T>& MyMap, const V& MyKey ){
	return (MyMap.find((U)MyKey)!=MyMap.end());
}

template <class T,class U,class V>
inline bool find(const std::unordered_map<U,T>& MyMap, const V& MyKey ){
	return (MyMap.find((U)MyKey)!=MyMap.end());
}

template <class T,class U>
inline bool find(const std::set<T>& MySet, const U& MyKey ){
	return (MySet.find((T)MyKey)!=MySet.end());
}


using lcmprint::operator<<;
const int64_t defaultvalue = -99999;
template<typename pointT>
bool is_lessThanZero(pointT cp);
void loadBehaviorParams();
exlcm::lanesegment_t getLaneSeg(LaneSegUidType uid);
exlcm::intersection_t getInterSection(InterSectionUidType uid);
std::vector<exlcm::lanesegment_t> getLaneSeg(std::vector<LaneSegUidType>);
bool SegExists(const Policy::InputPolicyType & MyPolicy, LaneSegUidType laneseg_uid);
//bool SegExists(LaneSegUidType laneseg_uid);
//bool SegExistsInThisGroup(LaneSegUidType , int );
bool SegExistsInThisGroup(const Policy::InputPolicyType & MyPolicy, LaneSegUidType , int );
bool SegExists(const Policy::InputPolicyType & MyPolicy,std::vector<LaneSegUidType>);
bool GrpExists(int MyGrp, const Policy::InputPolicyType & MyPolicy);
bool MapletRequestMissingLanePub(LaneSegUidType LastLaneSegmentUID);
vector<LaneSegUidType> MapletRequestMissingLanePub(const exlcm::routesegmentlist_t & );
extern std::string _ColorCode;
double getSegLen(LaneSegUidType uid);
//double getGrpLen(int uid);
double getGrpLen(const std::vector<LaneSegUidType>&);
double getGrpLen(int uid, const Policy::InputPolicyType & MyPolicy);
exlcm::connection_t getConn(LaneSegUidType startUid,LaneSegUidType endUid, int *ID=0);
exlcm::connection_t getGrpConn(int startUid,int endUid, const Policy::InputPolicyType & MyPolicy, int *ID=0);
exlcm::connection_t getGrpConn(const std::vector<LaneSegUidType>& ,const std::vector<LaneSegUidType>& , int *ID=0);
bool SuspectedLaneChaneGrpConn( std::vector<LaneSegUidType>&, std::vector<LaneSegUidType>&);
bool SuspectedLaneChaneGrpConn(int startUid,int endUid,const Policy::InputPolicyType & MyPolicy);
bool SuspectedOpposingLaneGrp(int laneID,const Policy::InputPolicyType & MyPolicy);
std::vector<LaneSegUidType> routeSegmentGrpTolanesegments(int, const Policy::InputPolicyType & );
//std::vector<LaneSegUidType> routeSegmentGrpTolanesegments(int );
int LaneSegmentToRouteSegmentGrp(LaneSegUidType LaneSegUID,const Policy::InputPolicyType &);


double getConnLen(LaneSegUidType startUid,LaneSegUidType endUid);
//double getGrpConnLen(int startUid,int endUid);
double getGrpConnLen(int startUid,int endUid,const Policy::InputPolicyType &);
double pDist(Point_t p1,Point_t p2);
double TimeNow();

inline exlcm::policybehaviorstring_t construct_policybehaviorstring(int PolicyBehaviorCnt,int route, const exlcm::behaviorstring_t& PolicyBehavior){
	exlcm::policybehaviorstring_t my_policybehaviorstring;
	my_policybehaviorstring.PolicyBehaviorCnt = PolicyBehaviorCnt;
	my_policybehaviorstring.route = route;
	my_policybehaviorstring.PolicyBehaviorStack.clear();
	my_policybehaviorstring.PolicyBehaviorStack.push_back(PolicyBehavior);
	return my_policybehaviorstring;
}




template <class T>
inline exlcm::behaviorstring_t Class_to_BehaviorString(const T& val){
	exlcm::behaviorstring_t SimpleBehaviorStr;
	std::ostringstream ss;
	ss<<val;
	std::string str = ss.str();
	int j=0;
	std::vector<std::string> strvec;
	for(int i=0;i<str.size();i++){
		if(str[i]=='\n'){
			strvec.push_back(str.substr(j,i-j));
			j=i;
		}
	}
	SimpleBehaviorStr.LineCnt = strvec.size();
	SimpleBehaviorStr.Behavior = strvec;
	return SimpleBehaviorStr;
}


template <typename T>
inline std::string to_string(const T val){
	std::ostringstream ss; lcmprint::PrintEnum(ss,val);
	std::string MyString = ss.str();
	std::string MySubString;
	if(MyString.back()=='\n'){
		MySubString =  MyString.substr(0, MyString.size()-1);
		return MySubString;
	}
	else
		return MyString;

}

template <class T>
inline bool codeRBGRaw(std::string msg, T val , float TimeGap, std::string ColorCode ,double *Timer ){

#ifndef NO_DEBUG_PRINT
	return false;
#endif
	
	const double A_LONG_TIME = 20;
	double timenow=BehaviorUtils::TimeNow();
	static std::map<std::string,decltype(timenow) > prev_msg;
	static std::map<std::string,decltype(timenow) > prev_msg_first_time;
	static std::map<std::string,decltype(timenow) > prev_msg_last_time;

	//static exlcm::visualizernotification_t MyVisualizerNotification;

	//MyVisualizerNotification.notification_emblem = PLANNING_EMBLEM;
	//MyVisualizerNotification.requestor_module_id = BEHAVIOR_CONTROL_MODULE_ID;

	bool MsgFound = false;
	std::string ColorCodemsg;
	std::ostringstream valuestrs,Dfltvaluestrs;

	std::string valuestr,Dfltvaluestr;


	valuestrs<<val;
	valuestr = valuestrs.str();
	Dfltvaluestrs<<BehaviorUtils::defaultvalue;
	Dfltvaluestr = Dfltvaluestrs.str();
	if(valuestr==Dfltvaluestr)
		valuestr="";

	if(ColorCode=="R")
	{
		ColorCodemsg = "CODERED: "; BehaviorUtils::_ColorCode = "R";
		std::string str =  msg + valuestr ;
		//if(VISUALIZER_NOTIFICATION_ENABLED_FOR_CODERED){
			//std::strcpy(MyVisualizerNotification.notification_text, str.c_str());
			//  MyVisualizerNotification.timestamp_sec = behaviorinput::hvLoc.timestamp_sec;
			//behavioroutput::behavioroutput.publish("VISUALIZERNOTIFICATION",&MyVisualizerNotification);
		//}
	}
	else if(ColorCode == "G")
	{ ColorCodemsg = "CODEGREEN: "; BehaviorUtils::_ColorCode = "G"; }
	else if (ColorCode == "B")
	{ ColorCodemsg = "CODEBLUE: "; BehaviorUtils::_ColorCode = "B"; }
	else if (ColorCode == "P")
	{ ColorCodemsg = "CODEPURPLE: "; BehaviorUtils::_ColorCode = "P"; }
	else if (ColorCode == "Y")
	{ ColorCodemsg = "CODEYELLOW: "; BehaviorUtils::_ColorCode = "Y"; }


	//	msg =  + msg  ;


	if (!prev_msg.empty())
	{
		auto prev_msg_copy = prev_msg;
		for(auto it:prev_msg_copy)
		{
			if (msg==it.first)
			{
				if((timenow-prev_msg_last_time[msg])>10) // No updates in some time
					prev_msg_first_time[msg]=timenow;
				if(Timer)
					*Timer = (timenow-prev_msg_first_time[msg]);
				prev_msg_last_time[msg]=timenow;

				if ((timenow-it.second)> TimeGap)
				{
					CodeRBGFileWrite(msg+valuestr);
					prev_msg[msg]=timenow;
					return true;
				}
				MsgFound = true;
			}
			else if((timenow-it.second)> A_LONG_TIME){
				prev_msg.erase(it.first);
				prev_msg_first_time.erase(it.first);
				prev_msg_last_time.erase(it.first);
			}
		}
	}
	if (!MsgFound)
	{
		CodeRBGFileWrite( msg+valuestr);
		prev_msg.insert(std::make_pair(msg,timenow));
		prev_msg_first_time[msg]=timenow;
		prev_msg_last_time[msg]=timenow;
		if(Timer)
			*Timer = 0.0;
		return true;
	}
	return false;
}

template <class T>
inline bool codeRBG(std::string msg, T val , float TimeGap, std::string ColorCode ,double *Timer ){
	if(!PRINT_CODE_RBG)
		return false;

	//if(msg.find(PRINT_CODE_RBG_FILES_CONTAINING)==std::string::npos)
	//	return false;

	if(TimeGap>1e6)
		return false;

	bool return_state = false;
	
	//return_state = BehaviorUtils::codeRBGRaw(msg,val,TimeGap,ColorCode,Timer);

	if (pthread_mutex_trylock(&codeRBG_mutex) != 0) {
		cout<<" could not acquire code RBG mutex lock for printing "<<'\n';
	}else{

		return_state = BehaviorUtils::codeRBGRaw(msg,val,TimeGap,ColorCode,Timer);
		


		if ( pthread_mutex_unlock(&codeRBG_mutex) != 0) {
			cout<<" could not unlock code RBG mutex lock for printing "<<'\n';
		}
	}

	return return_state;

}

template <class T>
inline bool codeRed(std::string msg, T val , float TimeGap = CodeRed_TimeGap , double *Timer = 0){
	if(!PRINT_CODE_R)
		return false;
	return BehaviorUtils::codeRBG(msg,val,TimeGap,"R",Timer);
}

template <class T>
inline bool codeGreen(std::string msg, T val ,float TimeGap = CodeGreen_TimeGap, double *Timer = 0){
	if(!PRINT_CODE_G)
		return false;
	return BehaviorUtils::codeRBG(msg,val,TimeGap,"G",Timer);
}

template <class T>
inline bool  codeBlue(std::string msg, T val , float TimeGap  = CodeBlue_TimeGap, double *Timer = 0){
	if(!PRINT_CODE_B)
		return false;
	return BehaviorUtils::codeRBG(msg,val,TimeGap,"B",Timer);
}

template <class T>
inline bool  codeYellow(std::string msg, T val , float TimeGap = CodeBlue_TimeGap, double *Timer = 0 ){
	if(!PRINT_CODE_Y)
		return false;
	return BehaviorUtils::codeRBG(msg,val,TimeGap,"Y",Timer);
}

template <class T>
inline bool codePurple(std::string msg, T val , float TimeGap = CodeBlue_TimeGap, double *Timer = 0 ){
	if(!PRINT_CODE_P)
		return false;
	return BehaviorUtils::codeRBG(msg,val,TimeGap,"P",Timer);
}


bool codeRed(std::string msg);
bool codeGreen(std::string msg);
bool codeBlue(std::string msg);
bool codeYellow(std::string msg);
bool codePurple(std::string msg);

bool codeRed(std::string msg, std::string ignore, float TimeGap ,  double *Timer = 0);
bool codeGreen(std::string msg, std::string ignore ,float TimeGap,  double *Timer = 0 );
bool codeBlue(std::string msg, std::string ignore , float TimeGap ,  double *Timer = 0);
bool codeYellow(std::string msg, std::string ignore , float TimeGap ,  double *Timer = 0);
bool codePurple(std::string msg, std::string ignore , float TimeGap,  double *Timer = 0 );



std::vector<double> ExtractParamFromObjSA(int uid,std::string param);

inline float  targetSpeed(float s, float u, float a)
{
	return sqrt((u*u) +(2*a*s));
}

inline float  targetSpeed(float s, float a)
{
	return targetSpeed(s,vehicle.curSpeed_mps,a);
}

inline float  targetDistance(float u, float v, float a)
{
	return ( (v*v-u*u)/(2*a) );
}

inline float  targetDistance(float v, float a)
{
	return targetDistance(vehicle.curSpeed_mps,v,a);
}


inline float  targetAcceleration(float s, float u, float v)
{
	return ((v*v-u*u)/(2*s));
}

inline float  targetAcceleration(float s, float v)
{
	return targetAcceleration(s,vehicle.curSpeed_mps,v);
}

inline std::pair<float,float>  targetSpeedAndAccel(float s, float u, float t)
{
	 double a,v;
	 a = (s-u*t)/(0.5*t*t);
	 v = u + a*t;
	 return std::make_pair(v,a);
}

inline std::pair<float,float>  targetSpeedAndAccel(float s, float t)
{
	 return targetSpeedAndAccel(s,vehicle.curSpeed_mps,t);
}


Point_t getENLocOfSegmentStation(LaneSegUidType segUID,int stationD_m);
std::pair<InterSectionUidType,double> FindClosestInterSectionInfo(int CurrentGrp);
std::pair<double,exlcm::intersection_t> ClosestIntersection(Point CurrenthvLoc);
std::tuple<double,exlcm::intersection_t> ClosestIntersectionAhead(Point CurrenthvLoc);
std::tuple<double,Point,double> GetExtremeCurvature(const exlcm::lanesegment_t& TargetSegment);
std::tuple<double,Point,double> GetExtremeCurvature(const exlcm::connection_t& TargetConnection);
double SysTimeNow();
double ModuleTimeNow();
double linearinterp(double a, double b, double f);
double LookUp1DMap(map<double,double> ValueMap, double XValue);
void CleanUpBuffer_SA();
void CleanUpBuffer_AltPlans();



struct MyStreamingHelper
{
	MyStreamingHelper(std::ostream& out1,std::ostream& out2);
	MyStreamingHelper();
	MyStreamingHelper(const MyStreamingHelper & src);
	MyStreamingHelper& operator=(const MyStreamingHelper& rhs);
	std::ostream& out1_;
	std::ostream& out2_;
};
template <typename T>
MyStreamingHelper& operator<<(BehaviorUtils::MyStreamingHelper& h, T const& t);
MyStreamingHelper& operator<<(BehaviorUtils::MyStreamingHelper& h, std::ostream&(*f)(std::ostream&));

};

/*
template<typename Key, typename val>
std::ostream& operator<<(std::ostream& os, const std::map< Key, val> & MyMap)
{
	for (const auto &s : MyMap) {
		os << s.first << ' ' << s.second << '\n';
	}
	return os;
}

template<typename T>
std::ostream &operator <<(std::ostream &os, const std::vector<T> &MyVec)
{
	for (const auto &s : MyVec) {
		os << s << '\n';
	}
	return os;
}
 */
std::ostream& color_on(std::ostream& os);



