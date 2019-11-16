#pragma once
#include "../PCH/pch.hpp"
#include "Point.hpp"
#include "shared_enums.h"
#include "lcm_def_includes.hpp"

#include "BehaviorTypeDef.hpp"
namespace Vehicle
{

void ConvertNEToXYVehicleCoords(double E, double N, double eRef, double nRef, double yawRad, double *x, double *y);
void initVehicleState();

class vehicleState
{
public:

	double curSpeed_mps;
	double curAccel_mpss;
	double PredAccel_mpss;

	double speedTargetDistance;
	double speedTarget;

	//Env related
	LaneSegUidType curSegmentUid;
	LaneSegUidType targetSegmentUid;
	double curSegCompletionRatio;//Fraction/% of the curSeg that the vehicle has completed/passed
	double curConnCompletionRatio;
	LaneSegUidType curConnOriginSegUid = 0;//Seg UID of the origin of the conn that the vehicle is currently in
	LaneSegUidType curConnDestinationSegUid = 0;
	double prevDTACS;
	double DTACS;// Distance (meters) Travelled Along Current Segment
	double DRACS;// Distance (meters) Revmaining Along Current Segment
	double DTAR;//Distance (meters) Travelled Along Route
	double c0; // Current lane offset
	double c1; // Current lane local heading


	//bool evasiveOrEmergencyMode;//True when the veh is performing some evasive/escape or emergency maneuvers
	int curIntersectionUid;//UID of the intersection the HV is currently in.

	//OEM/MABX vehicle state info
	float steering_angle_deg;
	float brake_fr;
	float throttle_fr;
	float outside_temp_c_deg;
	float daylight_fr;
	short gear;
	short park_brake_set_f;
	short turn_indicator_state; // 0 - unknown 1- left 2 - right 3 - Emergency Flash 4- No turn
	float oem_speed_mps;
	float oem_yawrate_rps;
	short wiper_state;
	float wiper_period_sec;

	vehicleState();
	vehicleState(int64_t ConnOriginSegUid, int64_t ConnDestinationSegUid);


} ;

std::ostream& operator <<(std::ostream &os, const Vehicle::vehicleState &MyvehicleState);

};

