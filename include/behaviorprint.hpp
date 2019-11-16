#pragma once
#include "../PCH/pch.hpp"
#include "shared_enums.h"
#include "lcm_def_includes.hpp"

namespace BehaviorTertiary{
class Maneuver_Temporal;
class Maneuver_Spatial;
class Behavior;
}

namespace Env{
class PolicyState;
}


std::ostream& operator<<(std::ostream &os, const set<behavior_type>& );
std::ostream& operator<<(std::ostream &os, const BehaviorTertiary::Behavior &);
std::ostream& operator<<(std::ostream &os, const BehaviorTertiary::Maneuver_Temporal& );
std::ostream& operator<<(std::ostream &os, const BehaviorTertiary::Maneuver_Spatial& );
//std::ostream& operator <<(std::ostream &os, const exlcm::pathstep_t &MyStep);

std::ostream &operator <<(std::ostream &os, const Env::PolicyState &MyState);
std::ostream &operator <<(std::ostream &os, const PolicyDequeType &MyDeque);


//void PrintSpatialTaskGroup(std::ostream &os, int SpatialTask );
