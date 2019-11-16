#include "../include/BehaviorUtils.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"
#include "../include/BehaviorGapUtils.hpp"
#include "../include/lcmprint.hpp"


BehaviorTertiary::Behavior& Policy::CreateBehaviorFromPolicyDeque(BehaviorTertiary::Behavior& BehaviorDesired, PolicyDequeType &MyPolicyDQ, int alternaterouteNeeded,  int FronIdxInMyPolicyDQ ,\
		int StartStep, bool ActivePolicyFlag,bool AlternatePolicyFlag ){
	int step = StartStep;
	double EndOfStepDistanceFromHV = 0;
	std::string StepStr = "<"+std::to_string(step)+">";
	Env::PolicyState FrontState = MyPolicyDQ[FronIdxInMyPolicyDQ];

	exlcm::behaviorstring_t tempstring;

	if(ActivePolicyFlag){

		behavioroutput::AllPolicyBehaviorStacks.clear();
		behavioroutput::AllPolicyBehaviorStacks.push_back(BehaviorUtils::construct_policybehaviorstring(1, alternaterouteNeeded, BehaviorUtils::Class_to_BehaviorString(BehaviorDesired))) ;
		tempstring = BehaviorUtils::Class_to_BehaviorString(BehaviorDesired);

		behavioroutput::CurrPolicy.behavior_steps_count = 1;
		behavioroutput::CurrPolicy.behavior_steps.clear();behavioroutput::CurrPolicy.behavior_step_distance_along_route.clear();
		behavioroutput::CurrPolicy.behavior_steps.push_back(BehaviorDesired.TargetState.step);
		behavioroutput::CurrPolicy.behavior_step_distance_along_route.push_back(BehaviorDesired.TargetState.DAR_m -vehicle.DTAR); // Changing the meaning
		behavioroutput::CurrPolicy.timestamp_sec = BehaviorUtils::TimeNow();
		behavioroutput::CurrPolicy.valid_f = true;
	}
	else if(AlternatePolicyFlag){
		bool PolicyFound = false;
		if(behavioroutput::_altbehaviorviz.PolicyCnt>0){
			for(int i=0;i<behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks.size();i++){
				if(behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[i].route == alternaterouteNeeded){
					behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[i] = BehaviorUtils::construct_policybehaviorstring(1, alternaterouteNeeded, BehaviorUtils::Class_to_BehaviorString(BehaviorDesired));
					PolicyFound = true;
					break;
				}
			}
		}
		if(!PolicyFound){
			behavioroutput::_altbehaviorviz.PolicyCnt++;
			behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks.push_back(BehaviorUtils::construct_policybehaviorstring(1, alternaterouteNeeded, BehaviorUtils::Class_to_BehaviorString(BehaviorDesired))) ;
		}
	}

	//BehaviorUtils::codePurple( _CF_ + " FrontState.DAR_m - vehicle.DTAR " + StepStr ,FrontState.DAR_m - vehicle.DTAR,0.1);


	if((FrontState.DAR_m - vehicle.DTAR)<BEHAVIOR_LOOK_AHEAD_HORIZON){
		BehaviorTertiary::Behavior TempBehavior = BehaviorDesired;
		//BU::codeBlue(_CF_ + "BehaviorDesired "+StepStr,BehaviorDesired,1);
		do{
			if(Policy::CreateNextStepBehavior(TempBehavior,MyPolicyDQ,FronIdxInMyPolicyDQ,alternaterouteNeeded,step,EndOfStepDistanceFromHV,ActivePolicyFlag,AlternatePolicyFlag)){
				BehaviorDesired = BehaviorDesired + TempBehavior;
				StepStr = "<"+std::to_string(step)+">";
				//	BehaviorUtils::codePurple( _CF_ + " step DAR_m - vehicle.DTAR " + StepStr ,BehaviorDesired.TargetState.DAR_m - vehicle.DTAR,0.1);
				//	BehaviorUtils::codeBlue( _CF_ + " EndOfStepDistanceFromHV , FronIdxInMyPolicyDQ " + StepStr ,std::make_pair(EndOfStepDistanceFromHV,FronIdxInMyPolicyDQ),0.1);
				break;
			}
			else{
				BehaviorDesired = BehaviorDesired + TempBehavior;
				StepStr = "<"+std::to_string(step)+">";
				//	BehaviorUtils::codeGreen( _CF_ + " step DAR_m - vehicle.DTAR " + StepStr ,BehaviorDesired.TargetState.DAR_m - vehicle.DTAR,0.1);
				//	BehaviorUtils::codeBlue( _CF_ + " EndOfStepDistanceFromHV , FronIdxInMyPolicyDQ " + StepStr ,std::make_pair(EndOfStepDistanceFromHV,FronIdxInMyPolicyDQ),0.1);
				/*	if(BehaviorUtils::codeGreen( _CF_ + " TempBehavior at step " + StepStr ,TempBehavior,1.0)){
					cout<<"vehicle.DTAR "<<vehicle.DTAR<<" FronIdxInMyPolicyDQ " <<FronIdxInMyPolicyDQ<<endl;
				} */
				//BehaviorUtils::codeGreen( _CF_ + " BehaviorDesired after adding TempBehavior in step " + StepStr ,BehaviorDesired,1.0);
			}
			//BU::codeBlue("BehaviorDesired "+StepStr,BehaviorDesired,1);


		}while(EndOfStepDistanceFromHV< BEHAVIOR_LOOK_AHEAD_HORIZON);
	}else{
		//	BehaviorUtils::codeYellow( _CF_ + " FrontState DAR_m - vehicle.DTAR " + StepStr ,FrontState.DAR_m - vehicle.DTAR,0.1);
	}
	//BU::codeBlue("BehaviorDesired At the End  "+StepStr,BehaviorDesired,1);

	return BehaviorDesired;
}

bool Policy::CreateNextStepBehavior(BehaviorTertiary::Behavior& BehaviorDesired, PolicyDequeType &MyPolicyDQ,int FronIdxInMyPolicyDQ, int alternaterouteNeeded, \
		int& step,double& EndOfStepDistanceFromHV,  bool ActivePolicyFlag,bool AlternatePolicyFlag){
	const double TimeGap = 1;
	std::string StepStr = "<"+std::to_string(step)+">";
	if(step+FronIdxInMyPolicyDQ>=MyPolicyDQ.size()){
		BehaviorUtils::codeRed( _CF_  + "End of route coming within the lookahead horizon at step "+StepStr,step+FronIdxInMyPolicyDQ,TimeGap);
		return true;
	}
	BehaviorTertiary::Behavior NextBehaviorDesired(MyPolicyDQ[step+FronIdxInMyPolicyDQ]);
	std::string NextBehaviorStr = " NextBehaviorDesired for Step " + StepStr;
	//BehaviorUtils::codeGreen( _CF_ + NextBehaviorStr ,NextBehaviorDesired,1.0);

	//BehaviorUtils::codeGreen( _CF_ + " BehaviorDesired before adding " + NextBehaviorStr ,BehaviorDesired,1.0);
	BehaviorDesired = BehaviorDesired + NextBehaviorDesired;
	//BehaviorUtils::codeGreen( _CF_ + " BehaviorDesired after adding " + NextBehaviorStr ,BehaviorDesired,1.0);
	if(ActivePolicyFlag){
		behavioroutput::CurrPolicy.behavior_steps_count++;
		behavioroutput::CurrPolicy.behavior_steps.push_back(NextBehaviorDesired.TargetState.step);
		behavioroutput::CurrPolicy.behavior_step_distance_along_route.push_back(NextBehaviorDesired.TargetState.DAR_m-vehicle.DTAR); // Changing the meaning

		behavioroutput::AllPolicyBehaviorStacks[0].PolicyBehaviorCnt++;
		behavioroutput::AllPolicyBehaviorStacks[0].PolicyBehaviorStack.push_back(BehaviorUtils::Class_to_BehaviorString(NextBehaviorDesired));

	}
	else if(AlternatePolicyFlag){
		for(int i=0;i<behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks.size();i++){
			if(behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[i].route == alternaterouteNeeded){
				behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[i].PolicyBehaviorStack.push_back(BehaviorUtils::Class_to_BehaviorString(NextBehaviorDesired));
				behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[i].PolicyBehaviorCnt++;
				break;
			}
		}

		//	behavioroutput::_altbehaviorviz.AllPolicyBehaviorStacks[0].PolicyBehaviorStack.push_back(BehaviorUtils::Class_to_BehaviorString(NextBehaviorDesired));

	}

	step++;
	if(step+FronIdxInMyPolicyDQ>=MyPolicyDQ.size()){
		BehaviorUtils::codeRed( _CF_  + "End of route coming within the lookahead horizon at step "+StepStr,step+FronIdxInMyPolicyDQ,TimeGap);
		return true;
	}
	EndOfStepDistanceFromHV = (MyPolicyDQ[step+FronIdxInMyPolicyDQ].DAR_m - vehicle.DTAR);
	return false;
}

