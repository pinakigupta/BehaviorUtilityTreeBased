#include "../../PCH/pch.hpp"

#include "../../include/BehaviorGapUtils.hpp"
#include "../../include/BehaviorObject.hpp"

namespace BU=BehaviorUtils;
struct  vehicle;

using namespace BehaviorGapUtils;
GapType BehaviorGapUtils::operator&(GapType MyGap, GapType OtherGap){

	if(MyGap.second<OtherGap.first) // No overlap
		return std::make_pair(0,0);

	if(MyGap.first>OtherGap.second) // No overlap
		return std::make_pair(0,0);

	return std::make_pair(max(MyGap.first,OtherGap.first),min(MyGap.second,OtherGap.second));
}

double BehaviorGapUtils::GapOverLapFraction(GapType MyGap, GapType OtherGap){
	GapType OverLappedGap = MyGap & OtherGap;
	if((OverLappedGap.first==0)&&(OverLappedGap.second==0))
		return 0;
	return ((OverLappedGap.second-OverLappedGap.first)/(MyGap.second-MyGap.first));
}

double BehaviorGapUtils::InstantGapOverLapFraction(GapType MyGap, GapPredFuncType GapOverLapPredfunc, double Tm){
	return GapOverLapFraction(MyGap,GapOverLapPredfunc(Tm));
}

GapPredFuncFracType BehaviorGapUtils::GapOverLapFraction(GapType MyGap, GapPredFuncType GapOverLapPredfunc){
	return std::bind(InstantGapOverLapFraction,MyGap,GapOverLapPredfunc,std::placeholders::_1);
}

GapType BehaviorGapUtils::GapOverLapPred(GapPredFuncType MyGapfunc, GapPredFuncType OtherGapfunc,double LkAhdTm){

	if(MyGapfunc(LkAhdTm).second<=OtherGapfunc(LkAhdTm).first) // No overlap
		return std::make_pair(0,0);

	if(MyGapfunc(LkAhdTm).first>=OtherGapfunc(LkAhdTm).second) // No overlap
		return std::make_pair(0,0);

	return std::make_pair(max(MyGapfunc(LkAhdTm).first,OtherGapfunc(LkAhdTm).first),min(MyGapfunc(LkAhdTm).second,OtherGapfunc(LkAhdTm).second));
}

GapPredFuncType BehaviorGapUtils::operator&(GapPredFuncType MyGapfunc, GapPredFuncType OtherGapfunc){
	return std::bind(GapOverLapPred,MyGapfunc,OtherGapfunc,std::placeholders::_1);
}

GapType BehaviorGapUtils::ConstGap(GapType ConstantGap,double LkAhdTm){
	return ConstantGap;
}

GapPredFuncType BehaviorGapUtils::ConstGapFunc(GapType ConstantGap){
	return std::bind(ConstGap,ConstantGap,std::placeholders::_1);
}

GapPredFuncType BehaviorGapUtils::operator &(GapPredFuncType MyGapfunc,  GapType ConstantGap){
	return std::bind(GapOverLapPred,MyGapfunc,ConstGapFunc(ConstantGap),std::placeholders::_1);
}
GapPredFuncType BehaviorGapUtils::operator &(GapType OtherGap,GapPredFuncType MyGapfunc){
	return (MyGapfunc & OtherGap);
}

GapType BehaviorGapUtils::PredictedGap(GapType CurrentGap,GapType CurrentGapVel,GapType CurrentAccel, GapType GapIDs, double LookAheadTime ,  bool IsthisHV ){
	auto PredGap = CurrentGap;
	double GAP_PRED_SPEED_FACTOR_POSSIBLE_HV = GAP_PRED_SPEED_FACTOR;
	double GAP_PRED_ACCEL_FACTOR_POSSIBLE_HV = GAP_PRED_ACCEL_FACTOR;

	if(IsthisHV){
		GAP_PRED_SPEED_FACTOR_POSSIBLE_HV = 1.0;
		GAP_PRED_ACCEL_FACTOR_POSSIBLE_HV = 1.0;
	}


	if((GapIDs.first!=-1)||IsthisHV)
		PredGap.first += GAP_PRED_SPEED_FACTOR_POSSIBLE_HV*CurrentGapVel.first*LookAheadTime+GAP_PRED_ACCEL_FACTOR_POSSIBLE_HV*(1/2)*CurrentAccel.first*LookAheadTime*LookAheadTime;
	if((GapIDs.second!=-1)||IsthisHV)
		PredGap.second += GAP_PRED_SPEED_FACTOR_POSSIBLE_HV*CurrentGapVel.second*LookAheadTime+GAP_PRED_ACCEL_FACTOR_POSSIBLE_HV*(1/2)*CurrentAccel.second*LookAheadTime*LookAheadTime;
	return PredGap;
}

double BehaviorGapUtils::AssessGapCost(GapPredFuncType GapOverLapPredfunc, GapPredFuncType GapPredfunc){
	double LC_cost = 0.0;
	GapPredFuncFracType idealzoneoverlapfrac = GapOverLapFraction(IDEAL_LC_ZONE,GapOverLapPredfunc);
	GapPredFuncFracType blindzoneoverlapfrac = GapOverLapFraction(BLIND_ZONE,GapPredfunc);
	GapPredFuncType idealzoneoverlap = (IDEAL_LC_ZONE & GapOverLapPredfunc);
	for (double Tm=0;Tm<=LCX_LOOKAHEAD_TIME_HORIZON;Tm+=LCX_LOOKAHEAD_STEP){
		if(idealzoneoverlap(Tm).first > IDEAL_LC_ZONE.first) // Doesn't contain the ideal lane change zone
			LC_cost += (LCX_LOOKAHEAD_STEP/LCX_LOOKAHEAD_TIME_HORIZON);
		else if (blindzoneoverlapfrac(Tm)<1.0)
			LC_cost += (LCX_LOOKAHEAD_STEP/LCX_LOOKAHEAD_TIME_HORIZON);
		else
			LC_cost += (1-2*idealzoneoverlapfrac(Tm))*(LCX_LOOKAHEAD_STEP/LCX_LOOKAHEAD_TIME_HORIZON);
		// +ve cost -> (1-idealzoneoverlapfrac) , -ve vost -> 1*-idealzoneoverlapfrac
	}
	if((idealzoneoverlap(0).first<=IDEAL_LC_ZONE.first)&&(idealzoneoverlap(LCX_LOOKAHEAD_TIME_HORIZON).first>IDEAL_LC_ZONE.first)){
		// Gap crossing HV ideal LC zone. Initially outside of ideal LC zone. So an worsening gap.
	//	cout<<yellow_on<<"idealzoneoverlap "<<idealzoneoverlap<<" IDEAL_LC_ZONE "<<IDEAL_LC_ZONE<<color_off<<endl;
		return 1;
	}

	return LC_cost;
}

std::vector<double> BehaviorGapUtils::AssessGapCost(GapPredFuncTypeVec GapOverLapPredfunc, GapPredFuncTypeVec GapPredfunc){
	std::vector<double> MyGapCosts;
	for(int i=0;i<GapOverLapPredfunc.size();i++){
		MyGapCosts.push_back(AssessGapCost(GapOverLapPredfunc[i],GapPredfunc[i]));
	}
	return MyGapCosts;
}



void BehaviorGapUtils::ExtractGapParam(GapObjUidType GapObjIDs, GapType & GapObjVel, GapType & GapObjAccel){
	auto FirstObjSpds = BU::ExtractParamFromObjSA(GapObjIDs.first,"vel");
	auto FirstObjAccels = BU::ExtractParamFromObjSA(GapObjIDs.first,"accel");
	auto SecondObjSpds = BU::ExtractParamFromObjSA(GapObjIDs.second,"vel");
	auto SecondObjAccels = BU::ExtractParamFromObjSA(GapObjIDs.second,"accel");
	GapObjVel = std::make_pair(FirstObjSpds[0]-vehicle.curSpeed_mps,SecondObjSpds[0]-vehicle.curSpeed_mps);  // rel velocity and acceleration
	GapObjAccel = std::make_pair(FirstObjAccels[0]-vehicle.PredAccel_mpss, SecondObjAccels[0]-vehicle.PredAccel_mpss);  // rel velocity and acceleration
	if(GapObjIDs.first==-1){
		GapObjVel.first=0.0;
		GapObjAccel.first=0.0;
	}
	if(GapObjIDs.second==-1){
		GapObjVel.second=0.0;
		GapObjAccel.second=0.0;
	}

}

void BehaviorGapUtils::AdjustGapOffset(const GapObjUidType ObjIDs ,GapType& offset){
	if(ObjIDs.first==-1)
		offset.first=0;
	else{
		double half_length = SA.objData[ObjIDs.first].length_m/2;
		if(offset.first>0)
			offset.first=std::fmax(half_length,offset.first);
		else
			offset.first=std::fmin(-half_length,offset.first);
	}

	if(ObjIDs.second==-1)
		offset.second=0;
	else{
		double half_length = SA.objData[ObjIDs.second].length_m/2;
		if(offset.second>0)
			offset.second=std::fmax(half_length,offset.second);
		else
			offset.second=std::fmin(-half_length,offset.second);
	}
}

GapType BehaviorGapUtils::operator+(GapType A,GapType B){
	GapType C;
	C.first = A.first + B.first;
	C.second = A.second + B.second;
	return C;
}

std::tuple<GapType,GapType,GapType,GapObjUidType,GapPredFuncType,GapPredFuncType,GapIntType> BehaviorGapUtils::CreateGapFuncs\
(GapPredFuncType center_ahead_gap_pred, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc,int gap_idx ){

	GapType Gap;
	GapObjUidType ObjIDs;
	GapIntType HypIdx,dummy;
	GapType offset = {VEHICLE_LENGTH/2 , -VEHICLE_LENGTH/2};
	if(gap_idx>0){
		ObjIDs = std::make_pair(SA.ahead_obj_id(LANE_SITUATION,gap_idx-1,&dummy.first,&HypIdx.first),SA.ahead_obj_id(LANE_SITUATION,gap_idx,&dummy.second,&HypIdx.second));
		Gap = std::make_pair(SA.clear_dist_ahead(LANE_SITUATION,gap_idx-1),SA.clear_dist_ahead(LANE_SITUATION,gap_idx));
	}
	else if(gap_idx<0){
		gap_idx = fabs(gap_idx);
		ObjIDs = std::make_pair(SA.behind_obj_id(LANE_SITUATION,gap_idx,&dummy.first,&HypIdx.first),SA.behind_obj_id(LANE_SITUATION,gap_idx-1,&dummy.second,&HypIdx.second));
		Gap = std::make_pair(SA.clear_dist_behind(LANE_SITUATION,gap_idx),SA.clear_dist_behind(LANE_SITUATION,gap_idx-1));
	}
	AdjustGapOffset(ObjIDs,offset);
	Gap = Gap + offset;
	GapType GapVel,GapAccel;
	ExtractGapParam(ObjIDs,GapVel,GapAccel);
	GapPredFuncType GapPred = std::bind(PredictedGap,Gap,GapVel,GapAccel,ObjIDs,std::placeholders::_1,false);
	GapPred = GapPred & LaneStaticGapPredFunc;
	GapPredFuncType OverLapPred = GapPred & center_ahead_gap_pred;
	Gap = GapPred(0);
	return std::make_tuple(Gap,GapVel,GapAccel,ObjIDs,GapPred,OverLapPred,HypIdx);
}



std::ostream& BehaviorGapUtils::operator<<(std::ostream& os,GapPredFuncType MyGapFunc){
	for(double Tm=0;Tm<=LCX_LOOKAHEAD_TIME_HORIZON;Tm+=LCX_LOOKAHEAD_STEP){
		os<<" { t(s)= "<<Tm<<" , "<<std::setprecision(3)<<MyGapFunc(Tm)<<" } ";
	}
	os<<endl;
	return os;
}

std::ostream& BehaviorGapUtils::operator<<(std::ostream& os,GapPredFuncFracType MyGapFuncFrac){
	for(double Tm=0;Tm<=LCX_LOOKAHEAD_TIME_HORIZON;Tm+=LCX_LOOKAHEAD_STEP){
		os<<" { t(s)= "<<Tm<<" , "<<std::setprecision(3)<<MyGapFuncFrac(Tm)<<" } ";
	}
	os<<endl;
	return os;
}

std::ostream& BehaviorGapUtils::operator<<(std::ostream& os,GapPredFuncTypeVec MyGapFuncVec){
	for(int i=0;i<MyGapFuncVec.size();i++)
		os<<MyGapFuncVec[i];
	return os;
}

bool BehaviorGapUtils::operator==(const GapType lhs,const GapType rhs){
	return ((lhs.first==rhs.first)&&(lhs.second==rhs.second));
}

bool BehaviorGapUtils::operator==(const GapObjUidType lhs,const int Val){
	return ((lhs.first==Val)&&(lhs.second==Val));
}

bool BehaviorGapUtils::operator==(const GapObjUidTypeVec lhs,const int Val){
	bool equal = true;
	for(auto GapObjUid:lhs )
		equal = equal && (GapObjUid==Val);
	return equal;
}


GapObjectVec BehaviorGapUtils::LaneGapObjVec(const BehaviorGapUtils::GapObject& Center_Ahead, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc, int sign){
	int gap_idx = sign;

	GapObjectVec MyGaps;
	GapObject MyGapObj(Center_Ahead,LANE_SITUATION,LaneStaticGapPredFunc,gap_idx);
	MyGapObj.lane = LANE_SITUATION;
	MyGaps.push_back(MyGapObj);

	do{
		gap_idx += sign;
		MyGapObj = GapObject(Center_Ahead,LANE_SITUATION,LaneStaticGapPredFunc,gap_idx);
		if(fabs(MyGapObj.Gap.second)>BEHAVIOR_LOOK_AHEAD_HORIZON)
			break;
		MyGaps.push_back(MyGapObj);
	}while(!(MyGaps.back().GapObjIDs==-1));

	return MyGaps;
}

double BehaviorGapUtils::AssessGapCost(GapObject& MyGapObject){
	return AssessGapCost(MyGapObject.GapOverlapPredFunc,MyGapObject.GapPredFunc);
}

std::vector<double> BehaviorGapUtils::AssessGapCost(GapObjectVec& MyGapObjectVec){
	std::vector<double> MyGapCosts;
	for(int i=0;i<MyGapObjectVec.size();i++){
		MyGapCosts.push_back(AssessGapCost(MyGapObjectVec[i]));
	}
	return MyGapCosts;
}

BehaviorGapUtils::GapObject::GapObject(GapType x,GapType v,GapType a,GapObjUidType objUID, int LANE_SITUATION,GapIntType ObjPredIdx, \
		GapPredFuncType LaneStaticGapPredFunc):Gap(x),GapVel(v),GapAccel(a),GapObjIDs(objUID),lane(LANE_SITUATION),objpred_idx(ObjPredIdx){
	ObsrvdTime = BU::TimeNow();
	GapPredFunc = std::bind(PredictedGap,Gap,GapVel,GapAccel,GapObjIDs,std::placeholders::_1,false);
	GapPredFunc = GapPredFunc & LaneStaticGapPredFunc;
	Gap = GapPredFunc(0);
	GapCost = 0;
	Hash(this->uid);
};
BehaviorGapUtils::GapObject::GapObject(GapType x,GapType v,GapType a,GapObjUidType UID, int LANE_SITUATION, GapIntType ObjPredIdx,\
		const GapObject& Center_Ahead, GapPredFuncType LaneStaticGapPredFunc):GapObject(x,v,a,UID,LANE_SITUATION, ObjPredIdx,LaneStaticGapPredFunc){
	ObsrvdTime = BU::TimeNow();
	GapOverlapPredFunc = (Center_Ahead.GapPredFunc & GapPredFunc);
	GapCost = AssessGapCost(*this);
	Hash(this->uid);
};
BehaviorGapUtils::GapObject::GapObject(const GapObject& Other,const GapObject& Center_Ahead, \
		GapPredFuncType LaneStaticGapPredFunc):GapObject(Other.Gap,Other.GapVel,Other.GapAccel,Other.GapObjIDs,Other.lane,Other.objpred_idx,LaneStaticGapPredFunc){
	ObsrvdTime = BU::TimeNow();
	GapOverlapPredFunc = (Center_Ahead.GapPredFunc & GapPredFunc);
	GapCost = AssessGapCost(*this);
	Hash(this->uid);
};
BehaviorGapUtils::GapObject::GapObject(const GapObject& Center_Ahead, int LANE_SITUATION, GapPredFuncType LaneStaticGapPredFunc, int gap_idx ){
	ObsrvdTime = BU::TimeNow();
	std::tie(Gap,GapVel,GapAccel,GapObjIDs,GapPredFunc,GapOverlapPredFunc,objpred_idx) = CreateGapFuncs(Center_Ahead.GapPredFunc,LANE_SITUATION,LaneStaticGapPredFunc,gap_idx);
	GapCost = AssessGapCost(*this);
	Hash(this->uid);
}

void BehaviorGapUtils::GapObject::UpdateOverLapPredFunc(GapPredFuncType center_ahead_gap_pred){
	GapOverlapPredFunc = (center_ahead_gap_pred & GapPredFunc);
	GapCost = AssessGapCost(*this);
};
void BehaviorGapUtils::GapObject::UpdatePredFunc(){
	GapPredFunc = std::bind(PredictedGap,Gap,GapVel,GapAccel,GapObjIDs,std::placeholders::_1,false);
	Gap = GapPredFunc(0);
};
void BehaviorGapUtils::GapObject::Hash(size_t& seed){
	boost::hash_combine(seed,lane);
	boost::hash_combine(seed,GapObjIDs);
}

GapObject BehaviorGapUtils::operator&(const BehaviorGapUtils::GapObject& MyGapobj, const BehaviorGapUtils::GapObject& OtherGapObj){
	GapObject CombinedGapObj;
	GapType EmptyGap(0,0);
	CombinedGapObj.Gap = (MyGapobj.Gap&OtherGapObj.Gap);
	if(CombinedGapObj.Gap==EmptyGap)
		return CombinedGapObj;

	if(CombinedGapObj.Gap.first==MyGapobj.Gap.first){
		CombinedGapObj.GapAccel.first = MyGapobj.GapAccel.first;
		CombinedGapObj.GapObjIDs.first = MyGapobj.GapObjIDs.first;
		CombinedGapObj.GapVel.first = MyGapobj.GapVel.first;
		CombinedGapObj.objpred_idx.first = MyGapobj.objpred_idx.first;
	}
	else if (CombinedGapObj.Gap.first==OtherGapObj.Gap.first){
		CombinedGapObj.GapAccel.first = OtherGapObj.GapAccel.first;
		CombinedGapObj.GapObjIDs.first = OtherGapObj.GapObjIDs.first;
		CombinedGapObj.GapVel.first = OtherGapObj.GapVel.first;
		CombinedGapObj.objpred_idx.first = OtherGapObj.objpred_idx.first;
	}
	else{
		BU::codeRed(_CF_+" Something is wrong in GapObj & operator, CombinedGapObj ",CombinedGapObj,0.01);
	}

	if(CombinedGapObj.Gap.second==MyGapobj.Gap.second){
		CombinedGapObj.GapAccel.second = MyGapobj.GapAccel.second;
		CombinedGapObj.GapObjIDs.second = MyGapobj.GapObjIDs.second;
		CombinedGapObj.GapVel.second = MyGapobj.GapVel.second;
		CombinedGapObj.objpred_idx.second = MyGapobj.objpred_idx.second;
	}
	else if (CombinedGapObj.Gap.second==OtherGapObj.Gap.second){
		CombinedGapObj.GapAccel.second = OtherGapObj.GapAccel.second;
		CombinedGapObj.GapObjIDs.second = OtherGapObj.GapObjIDs.second;
		CombinedGapObj.GapVel.second = OtherGapObj.GapVel.second;
		CombinedGapObj.objpred_idx.second = OtherGapObj.objpred_idx.second;
	}
	else{
		BU::codeRed(_CF_+" Something is wrong in GapObj & operator, CombinedGapObj ",CombinedGapObj,0.01);
	}

	CombinedGapObj.GapOverlapPredFunc = MyGapobj.GapOverlapPredFunc & OtherGapObj.GapOverlapPredFunc;
	CombinedGapObj.GapPredFunc =  MyGapobj.GapPredFunc & OtherGapObj.GapPredFunc;
	CombinedGapObj.GapCost = AssessGapCost(CombinedGapObj);
	size_t seed;
	CombinedGapObj.Hash(seed);
	CombinedGapObj.uid = seed;
	return CombinedGapObj;
}


std::ostream& BehaviorGapUtils::operator<<(std::ostream& os,const BehaviorGapUtils::GapObject& MyGapObject){
	os<<" Gap UID "<<MyGapObject.uid;
	os<<" Gap "<<MyGapObject.Gap<<" Gap Vel "<<MyGapObject.GapVel<<" Gap Accel "<<MyGapObject.GapAccel<<" Gap Objs "<<MyGapObject.GapObjIDs<<" GapCost " <<\
			MyGapObject.GapCost<<" Gap UID "<<MyGapObject.uid<<" Obj pred idx " <<MyGapObject.objpred_idx<<endl;
//	os<<" Pred Func "<<MyGapObject.GapPredFunc<<endl;
	return os;
}

std::ostream& BehaviorGapUtils::operator<<(std::ostream& os,const BehaviorGapUtils::GapObjectVec& MyGapObjectVec){
	for(int i=0;i<MyGapObjectVec.size();i++)
		os<<" Gap["<<i<<"] = "<<MyGapObjectVec[i]<<endl;
	return os;
}

BehaviorTertiary::Behavior& operator<<(BehaviorTertiary::Behavior& MyBehavior,const BehaviorGapUtils::GapObject& MyGap){ // Assign this gap to both current and Target LCZ
	MyBehavior.laneChangeZoneStart_m = min(max(MyGap.Gap.first,MyBehavior.laneChangeZoneStart_m),MyBehavior.laneChangeZoneEnd_m);
	MyBehavior.laneChangeZoneEnd_m = max(min(MyBehavior.laneChangeZoneEnd_m,MyGap.Gap.second),MyBehavior.laneChangeZoneStart_m);
	MyBehavior.TgtlaneChangeZoneStart_m = MyBehavior.laneChangeZoneStart_m;
	MyBehavior.TgtlaneChangeZoneEnd_m = MyBehavior.laneChangeZoneEnd_m;
	MyBehavior.Spatial_cost = MyGap.GapCost+1; // Shift to +ve only cost from (-1 to +1)
	MyBehavior.Maneuver_Spatial::CalculateCost();
	MyBehavior.InsertObj(CreateObjDetails(MyGap.GapObjIDs,MyGap.Gap,MyGap.lane,MyGap.objpred_idx));
	return MyBehavior;
}

BehaviorTertiary::Behavior& operator<(BehaviorTertiary::Behavior& MyBehavior,const BehaviorGapUtils::GapObject& MyGap){ // Assign this gap to only current LCZ
	MyBehavior.laneChangeZoneStart_m = min(max(MyGap.Gap.first,MyBehavior.laneChangeZoneStart_m),MyBehavior.laneChangeZoneEnd_m);
	MyBehavior.laneChangeZoneEnd_m = max(min(MyBehavior.laneChangeZoneEnd_m,MyGap.Gap.second),MyBehavior.laneChangeZoneStart_m);
	MyBehavior.InsertObj(CreateObjDetails(MyGap.GapObjIDs,MyGap.Gap,MyGap.lane,MyGap.objpred_idx));
	return MyBehavior;
}


void* BehaviorGapUtils::PlotPredictedGapHandler(void *){
	while(true){

		if (pthread_mutex_trylock(&Mutex::_lanechangeplot_mutex) != 0) {
			BehaviorUtils::codeRed( _CF_  + " could not acquire mutex lock for _lanechangeplot_mutex in plotting thread ", " ",1);
		}else{

	/*		if((!TargetLane_Ahead.empty())&&(!TargetLane_Behind.empty())){
				BehaviorGapUtils::PlotGaps(TargetLane_Behind,TargetLane_Adjacent,TargetLane_Ahead,Center_Ahead);
			}
    */
			if ( pthread_mutex_unlock(&Mutex::_lanechangeplot_mutex) != 0) {
				BehaviorUtils::codeRed( _CF_  + " could not unlock mutex lock for _lanechangeplot_mutex in plotting thread ", " ",1);
			}
			else{
				usleep(1e6*BEHAVIORPLOT_WAIT_TIME_S);
			}
		}
	}
	return NULL;
}

exlcm::behaviorgapviz_t BehaviorGapUtils::construct_behaviorgapviz(float gap_pred_time,float gap_start,float gap_end){
	exlcm::behaviorgapviz_t My_behaviorgapviz;
	My_behaviorgapviz.gap_pred_time = gap_pred_time;
	My_behaviorgapviz.gap_start = gap_start;
	My_behaviorgapviz.gap_end = gap_end;
	return My_behaviorgapviz;
}

exlcm::behaviorgapviz_t BehaviorGapUtils::construct_behaviorgapviz(float gap_pred_time,const BehaviorGapUtils::GapObject& MyGap){
	exlcm::behaviorgapviz_t My_behaviorgapviz;
	My_behaviorgapviz.gap_pred_time = gap_pred_time;
	My_behaviorgapviz.gap_start = MyGap.GapPredFunc(gap_pred_time).first;
	My_behaviorgapviz.gap_end = MyGap.GapPredFunc(gap_pred_time).second;
	My_behaviorgapviz.gap_cost = MyGap.GapCost;
	My_behaviorgapviz.gap_start_vel = MyGap.GapVel.first;
	My_behaviorgapviz.gap_end_vel = MyGap.GapVel.second;
	My_behaviorgapviz.gap_start_accel = MyGap.GapAccel.first;
	My_behaviorgapviz.gap_end_accel = MyGap.GapAccel.second;
	return My_behaviorgapviz;
}


void BehaviorGapUtils::PlotPredictedGap(GapPredFuncTypeVec & MyGapfuncs,GapPredFuncTypeVec & MyGapOverLapfuncs, GapPredFuncType & CurrentLaneGap, \
		std::vector<std::string> Mylabels){
	//plt::clf();
	std::string title = "Predicted Gaps";
	//plt::title(title);
	int N = MyGapfuncs.size();
	std::vector<double> x,y1,y2;
	std::vector<double> CurrentLane_t,CurrentLane_y1,CurrentLane_y2;
	std::vector<double> overlap_t,overlap_y1,overlap_y2;

	//const double TIME_LMT = 3;
	for(double t=0;t<LCX_LOOKAHEAD_TIME_HORIZON;t+=0.1){
		CurrentLane_t.push_back(t);
		CurrentLane_y1.push_back(CurrentLaneGap(t).first);
		CurrentLane_y2.push_back(CurrentLaneGap(t).second);
	}


	for (int i=1;i<=N;i++){
		for(double t=0;t<LCX_LOOKAHEAD_TIME_HORIZON;t+=0.1){
			x.push_back(t);
			y1.push_back(MyGapfuncs[i-1](t).first);
			y2.push_back(MyGapfuncs[i-1](t).second);
		}

		for(double t=0;t<LCX_LOOKAHEAD_TIME_HORIZON;t+=0.1){
			overlap_t.push_back(t);
			overlap_y1.push_back(MyGapOverLapfuncs[i-1](t).first);
			overlap_y2.push_back(MyGapOverLapfuncs[i-1](t).second);
		}
		/*
		plt::subplot(N,1,i);
		std::map<std::string, std::string> keywords = {{"facecolor","yellow"}};
		plt::fill_between(CurrentLane_t,CurrentLane_y1,CurrentLane_y2,keywords);
		std::map<std::string, std::string> keywords1 = {{"facecolor","blue"}};
		plt::fill_between(x,y1,y2,keywords1);
		std::map<std::string, std::string> keywords2 = {{"facecolor","green"}};
		plt::fill_between(overlap_t,overlap_y1,overlap_y2,keywords2);
		if(!Mylabels.empty())
			plt::ylabel(Mylabels[i-1]);
		plt::ylim(-35, 35);
		x.clear();y1.clear();y2.clear();
		overlap_t.clear();overlap_y1.clear();overlap_y2.clear();
		*/
	}

	//plt::show(false);
	//plt::pause(0.005);
}


