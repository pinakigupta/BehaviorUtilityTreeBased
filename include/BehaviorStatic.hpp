#pragma once
#include "../PCH/pch.hpp"

#include "../include/BehaviorEnums.h"
#include "../include/lcmprint.hpp"
#include "../include/BehaviorTypeDef.hpp"


size_t HashRoutePlan(const exlcm::routesegmentlist_t & Myrouteplan);
void HashPathStep(const exlcm::pathstep_t & Mypathstep, size_t& seed);
size_t HashPathStep(const exlcm::pathstep_t & Mypathstep);
size_t HashpathPlan(const exlcm::pathplan_t & MypathPlan);

template <class T> inline void Hash_combine(std::size_t& seed, const T& v)
{
    boost::hash<T> hasher;
    const std::size_t kMul = 0x9ddfea08eb382d69ULL;
    std::size_t a = (hasher(v) ^ seed) * kMul;
    a ^= (a >> 47);
    std::size_t b = (seed ^ a) * kMul;
    b ^= (b >> 47);
    seed = b * kMul;
}

namespace Policy{
class InputPolicyType
{
public:
	int64_t uid;
	exlcm::routesegmentlist_t routeplan;
	exlcm::pathplan_t pathPlan;
	std::map<int,InterSectionUidType> intersectionplan; // key = group no , value = intersection uid

	int getUID();
	void CalculateIntersectionPlan();
	InputPolicyType(exlcm::routesegmentlist_t Myrouteplan,exlcm::pathplan_t MypathPlan);
	InputPolicyType();
	bool PolicyDiagnostics();


};


}


inline bool operator==(const Policy::InputPolicyType& lhs,const Policy::InputPolicyType& rhs){
	return (lhs.uid==rhs.uid);
}

namespace Policy{
extern std::map <int,Policy::InputPolicyType > AllaltPolicies;
}



