#include "../../include/BehaviorTertiary.hpp"
#include "../../include/behaviorHorizon.hpp"
#include "../../include/shared_enums.h"

namespace BU=BehaviorUtils;





BehaviorTertiary::Behavior BehaviorElementary::turn(Env::PolicyState &TargetState,double TurnSpeedTarget)
{

	BehaviorTertiary::Behavior Behavior_Turn = BehaviorElementary::GetMaxSpeedBehavior(TargetState);
	const double TimeGap = 10.0;
	//If there is no threat, let the vehicle continue within the boundaries defined by the Scenario planner.Just slow down the vehicle to execute a turn maneuver
	double env_s = TargetState.GrpEnd_DAR_m-vehicle.DTAR;
	double SpdTgtDistance = env_s;
	int rule = TargetState.step.step_rule;
	//if(rule==FOLLOW_INTERSECTION_RULE)
	//	SpdTgtDistance= SpdTgtDistance - TargetState.CurrentToTgtGrpConnLen ;
	SpdTgtDistance -=TURN_TGT_DIST_OFFSET;
	double SpeedTarget=TargetState.step.posted_speed_lim_mps*DEC_VEL_TO_TURN_FACTOR;
	if(TurnSpeedTarget>0)
		SpeedTarget = TurnSpeedTarget;


		if(TargetState.step.direction==TURN_LEFT){
			//BehaviorTertiary::Maneuver_Temporal Maneuver_Turn = BehaviorElementary::GetMaxSpeedBehavior(TargetState,SpdFactor);
			//Maneuver_Turn.TemporalRule = TempRules::LEFT_TURN;
			if(SpdTgtDistance>0){
				Behavior_Turn.speedTargetDistance = SpdTgtDistance;
				Behavior_Turn.speedTargetDistanceActual = SpdTgtDistance;
				Behavior_Turn.AccelTarget=BehaviorUtils::targetAcceleration(SpdTgtDistance,SpeedTarget);
				Behavior_Turn.ProjectedAcceleration = Behavior_Turn.AccelTarget;
				Behavior_Turn.speedTarget = SpeedTarget;
				Behavior_Turn.SpatialTask = TURN_TASK;
				Behavior_Turn.direction = TURN_LEFT;
				Behavior_Turn.TemporalRule = TempRules::LEFT_TURN;
				Behavior_Turn.CalculateBehavior();
				Behavior_Turn.CalculateCost();
				BehaviorUtils::codeBlue( _CF_ +"Evaluating LEFT_TURN ","  ",TimeGap);
			}
			Behavior_Turn.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_LEFT;
			Behavior_Turn.turn_indicator_target_distance = env_s;


		}
		else if(TargetState.step.direction==TURN_RIGHT){
			//BehaviorTertiary::Maneuver_Temporal Maneuver_Turn = BehaviorElementary::GetMaxSpeedBehavior(TargetState,SpdFactor);
			//Maneuver_Turn.TemporalRule = TempRules::RIGHT_TURN;
			if(SpdTgtDistance>0){
				Behavior_Turn.speedTargetDistance = SpdTgtDistance;
				Behavior_Turn.speedTargetDistanceActual = SpdTgtDistance;
				Behavior_Turn.speedTarget = SpeedTarget;
				Behavior_Turn.AccelTarget=BehaviorUtils::targetAcceleration(SpdTgtDistance,SpeedTarget);
				Behavior_Turn.ProjectedAcceleration = Behavior_Turn.AccelTarget;
				Behavior_Turn.SpatialTask = TURN_TASK;
				Behavior_Turn.direction = TURN_RIGHT;
				Behavior_Turn.TemporalRule = TempRules::RIGHT_TURN;
				Behavior_Turn.CalculateBehavior();
				Behavior_Turn.CalculateCost();
				BehaviorUtils::codeBlue( _CF_ +"Evaluating RIGHT_TURN "," ",TimeGap);
			}

			Behavior_Turn.turn_indicator_command = BehaviorTertiary::TURN_INDICATOR_RIGHT;
			Behavior_Turn.turn_indicator_target_distance = env_s;

		}






	return Behavior_Turn;
}

