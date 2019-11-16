#pragma once
#include "../PCH/pch.hpp"

#include "Vehicle.hpp"
#include "behaviorHorizon.hpp"


namespace IO_HANDLER
{

extern std::set<InterSectionUidType> _AllReceivedInterSections;
extern std::set<LaneSegUidType> _AllReceivedlanesegments;


void exitWithError(const std::string &error);




class lcmMsgHandler

{
public:
	~lcmMsgHandler(){};

	bool firstKin=true;

	void KINEMATICS( const exlcm::kinematics_t *msg);

	void LOCALIZATION ( const exlcm::localization_t * msg);


	void LANESEGMENT(const exlcm::lanesegment_t* msg);

	void PATHPLAN(const exlcm::pathplan_t *msg);

	void ALTERNATEPATHPLAN( const exlcm::pathplan_t *msg);

	void INTERSECTION(const exlcm::intersection_t*msg);

	void ROUTESEGMENTLIST( const exlcm::routesegmentlist_t *msg);

	void ALTERNATEROUTESEGMENTLIST( const exlcm::routesegmentlist_t *msg);

	void SITUATION(const exlcm::situation_t * msg);

	void OBJSITUATION(const exlcm::objsituation_t * msg);

	void OBJPREDICTION( const exlcm::objprediction_t * msg);

	void TRAFFICSIGNALINTERPRETATION(const exlcm::trafficsignalinterpretation_t* msg);

	void TRAFFICSIGNINTERPRETATION(const exlcm::trafficsigninterpretation_t* msg);

	void TRAJECTORYPLAN(const exlcm::trajectoryplan_t* msg);

	void VISUALIZERSTATES(const exlcm::visualizerstates_t* msg);


    int start();

    int lcm_thread();
};


void invokeNonBlockingLCMHandle();



bool PolicyDataCopy();
bool AlternatePolicyDataCopy();
bool CopyIntersectionData();
bool CopyTafficSignData();
bool CopyTafficSignalData();
bool CopyObjData();

};
