
#include "../include/Vehicle.hpp"
#define PI 3.14159
#include "../include/behaviorHorizon.hpp"
#include "../include/BehaviorUtils.hpp"

	Vehicle::vehicleState::vehicleState(){
		this->curConnOriginSegUid= 0;
		this->curConnDestinationSegUid = 0;
		this->curSegCompletionRatio = 0.0;
	};

	Vehicle::vehicleState::vehicleState(int64_t ConnOriginSegUid, int64_t ConnDestinationSegUid):vehicleState(){
		this->curConnOriginSegUid= ConnOriginSegUid;
		this->curConnDestinationSegUid = ConnDestinationSegUid;
	}

// NOTE: Vehicle coordinate frame is assumed to be: X along the direction of travel and Y is clockwise and Zpointing inward. The heading according to the positionPose data dict. is zero at N and increases positively in the clock-wise direction.
// Heading is in degrees
void Vehicle::ConvertNEToXYVehicleCoords(double E, double N,double  eRef,double nRef, double yawRad, double *x, double * y)
{

	double th =std::atan2((N-nRef),(E-eRef));
	double r= sqrt((N-nRef)*(N-nRef)+(E-eRef)*(E-eRef));
	if(yawRad<=0)
		yawRad=2*PI-std::fabs(yawRad);

	*x=(E-eRef)*cos(yawRad)+(N-nRef)*sin(yawRad);
	*y=-(E-eRef)*sin(yawRad)+(N-nRef)*cos(yawRad);
}

std::ostream& Vehicle::operator <<(std::ostream &os, const Vehicle::vehicleState &MyvehicleState)
{
	os<<" prev DTACS = "<<MyvehicleState.prevDTACS<<" DTACS = "<<MyvehicleState.DTACS<<" DTAR = "<<MyvehicleState.DTAR<<endl;
	os<<" curSegmentUid = "<<MyvehicleState.curSegmentUid<<" targetSegmentUid = "<<MyvehicleState.targetSegmentUid<<endl;
	os<<" curSegCompletionRatio = "<<MyvehicleState.curSegCompletionRatio<<" curConnCompletionRatio = " <<MyvehicleState.curConnCompletionRatio<<endl;
	os<<" curConnOriginSegUid = "<<MyvehicleState.curConnOriginSegUid<<" curConnDestinationSegUid = "<<MyvehicleState.curConnDestinationSegUid<<endl;
	os<<" Current Accel "<<MyvehicleState.curAccel_mpss<<" Predicted Accel "<<MyvehicleState.PredAccel_mpss<<endl;
	return os;
			os<<" CurrConnLength = "<<\
			BehaviorUtils::getConnLen(MyvehicleState.curConnOriginSegUid,MyvehicleState.curConnDestinationSegUid)<<" segment Length = "  <<\
			BehaviorUtils::getSegLen(MyvehicleState.curSegmentUid)<<endl;

}
