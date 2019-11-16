#include "../PCH/pch.hpp"


#include "../include/lcmprint.hpp"
#include "../include/lcm_def_includes.hpp"
#include "../include/Vehicle.hpp"
#include "../include/Env.hpp"
#include "../include/io_handler.hpp"
#include "../include/BehaviorElementary.hpp"
#include "../include/BehaviorUtils.hpp"


void my_exit_handler(sig_t s){
	BehaviorUtils::codeGreen( _CF_ +"  ---------   Exiting the Behavior main function ---------");

	BehaviorUtils::codeRed( _CF_  + "validpathPlanAvailable = ",validpathPlanAvailable);
	BehaviorUtils::codeRed( _CF_  + "validRouteSegListAvailable = ",validRouteSegListAvailable);
	BehaviorUtils::codeRed( _CF_  + "allMapletlanesegmentsAvailable = ",allMapletlanesegmentsAvailable);

	BehaviorUtils::codeBlue( _CF_ +" Env::initDone ",(double)Env::initDone);

	if(BehaviorUtils::codeBlue( _CF_ +" Vehicle Details: ")," ",-1)
		cout<<vehicle<<endl;

	BehaviorUtils::codeRed( _CF_  + "DTAR = ",vehicle.DTAR);
	cout<<vehicle<<endl;



		BehaviorUtils::codeRed( _CF_  + "StateDQ is empty"," ",-1);
	exit(1);
}

void *exit_handler(void *){

	try{
		signal (SIGINT,my_exit_handler);
		signal (SIGTERM,my_exit_handler);
		signal (SIGABRT,my_exit_handler);
		signal (SIGSEGV,my_exit_handler);
		signal (SIGFPE,my_exit_handler);
		std::atexit(my_exit_handler);
		//std::at_quick_exit(my_exit_handler);
	}catch(...){
		cout<<" Error in function handler exit_handler. Maybe not running in Linux ??"<<endl;
	}

}



std::sig_atomic_t gSignalStatus;
void signal_handler(int signal)
{
	gSignalStatus = signal;
}
void *VisualizerFreeze_handler(void *){
	static bool FreezeFrameOnLogPlayerPause_prev;
	std::signal(SIGINT, signal_handler);
	while(true){
		SIMFREEZEONSET = false;
		if (pthread_mutex_trylock(&Mutex::_visualizerstates_mutex) != 0) {
			cout<<" could not acquire visualizer states mutex lock for VisualizerFreeze_handler "<<endl;
		}else{
			if((behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause)&& !FreezeFrameOnLogPlayerPause_prev)
				SIMFREEZEONSET = true;

			FreezeFrameOnLogPlayerPause_prev = behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause;

			if ( pthread_mutex_unlock(&Mutex::_visualizerstates_mutex) != 0) {
				cout<<" could not unlock visualizer states mutex lock for VisualizerFreeze_handler "<<endl;
			}
		}

	//	if(RAISE_INTERRUPT)
	//		std::raise(SIGKILL);
		if(SIMFREEZEONSET)
			BehaviorUtils::codeYellow( _CF_ +" SIMFREEZEONSET is set "," ",0.25);
		usleep(2*1e4);
	}
	return NULL;
}


void *LcmPublish_handler(void *){
	while(true){
		if (pthread_mutex_trylock(&Mutex::_behavioroutputfast_mutex) != 0) {
			cout<<" could not acquire  mutex lock for LCM Pub_handler "<<endl;
		}else{

			if(!behaviorinput::_visualizerstates.FreezeFrameOnLogPlayerPause){
				//if(USEROUTPUTS_PUB_ENABLED)
					//behavioroutput::behavioroutputfast.publish("USEROUTPUTS",&behavioroutput::userout);

				//behavioroutput::behavioroutputfast.publish("BEHAVIORPLAN",&behavioroutput::behaviorPlan);

			}

			if ( pthread_mutex_unlock(&Mutex::_behavioroutputfast_mutex) != 0) {
				cout<<" could not unlock mutex unlock for LCM Pub_handler "<<endl;
			}

		}
		usleep(BEHAVIOR_PUB_WAIT_TIME_S*1e6);
	} // while(true)
	return NULL;
}
