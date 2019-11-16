#include <cassert>
#include "../include/BehaviorGapUtils.hpp"
#include "../include/BehaviorOptimalPolicy.hpp"

std::map<int, PolicyDequeType> Prev_ALL_AlternatePolicyDQ;
std::map<int, Policy::InputPolicyType> Prev_ALL_AlternatePolicies;

namespace Policy {
void SearchAlternatePolicy(Policy::DetailedPolicy& MyDetailedPolicy,
                           const Env::PolicyState& TargetState,
                           int alternaterouteNeeded) {
  PolicyDequeType AlternatePolicyDQ, EmptyPolicyDQ;
  Policy::InputPolicyType AlternatePolicy, AlternatePolicy_IN, EmptyPolicy;
  Policy::DetailedPolicy LocalDetailedPolicy;
  auto EmptyPair = std::make_pair(EmptyPolicyDQ, EmptyPolicy);
  exlcm::routesegmentlist_t altrouteSegmentList;
  exlcm::pathplan_t altpathPlan;
  const double TimeGap = 0.1;
  std::string altstr =
      "<altroute:" +
      BehaviorUtils::to_string((reroute_type_enum)alternaterouteNeeded) + ">";

  if (Policy::AllaltPolicies.find(alternaterouteNeeded) ==
      Policy::AllaltPolicies.end()) {
    if (BehaviorUtils::codeRed(
            _CF_ +
                " Returning as All alt policies doesn't have an alternate path "
                "plan for alternaterouteNeeded =   " +
                altstr,
            " ", TimeGap)) {
      lcmprint::PrintEnum(cout, (reroute_type_enum)alternaterouteNeeded);
      cout << " Following alternate policies are available " << endl;
      for (auto& plan : Policy::AllaltPolicies)
        lcmprint::PrintEnum(cout, (reroute_type_enum)plan.first);
      cout << endl;
    }
    return;
  } else if (Policy::AllaltPolicies[alternaterouteNeeded]
                 .routeplan.segment_count <= 0) {
    BehaviorUtils::codeRed(
        _CF_ + "Returning EmptyPair as Alt route Plan segment_count is  " +
            altstr,
        Policy::AllaltPolicies[alternaterouteNeeded].routeplan.segment_count,
        TimeGap);
    return;
  } else if (Policy::AllaltPolicies[alternaterouteNeeded]
                 .pathPlan.included_step_count <= 0) {
    BehaviorUtils::codeRed(
        _CF_ + "Returning EmptyPair as Alt path plan included step count  is " +
            altstr,
        Policy::AllaltPolicies[alternaterouteNeeded]
            .pathPlan.included_step_count,
        TimeGap);
    return;
  } else if (Policy::AllaltPolicies[alternaterouteNeeded]
                 .routeplan.segment_group.size() <= 0) {
    BehaviorUtils::codeRed(
        _CF_ + "Returning EmptyPair as Alt route Plan segment group size  " +
            altstr,
        Policy::AllaltPolicies[alternaterouteNeeded]
            .routeplan.segment_group.size(),
        TimeGap);
    return;
  } else if (Policy::AllaltPolicies[alternaterouteNeeded]
                 .pathPlan.path_steps.size() <= 0) {
    BehaviorUtils::codeRed(
        _CF_ + "Returning EmptyPair as Alt path plan path steps size is " +
            altstr,
        Policy::AllaltPolicies[alternaterouteNeeded].pathPlan.path_steps.size(),
        TimeGap);
    return;
  }

  int MissingLaneID;
  AlternatePolicy_IN = Policy::AllaltPolicies[alternaterouteNeeded];
  if (!Prev_ALL_AlternatePolicyDQ[alternaterouteNeeded].empty()) {
    int reRouteNeeded = alternaterouteNeeded;
    //	alternaterouteNeeded = NO_REROUTE;
    if (BehaviorUtils::codeGreen(_CF_ +
                                     " Requested ALTERNATE_LANE_SITUATION has "
                                     "been previously created " +
                                     altstr,
                                 " ", 1)) {
      lcmprint::PrintEnum(cout, (reroute_type_enum)reRouteNeeded);
      // cout<<" vehicle.DTAR "<<vehicle.DTAR<<endl;
      // cout<<"ThisLaneBehavior "<<BehaviorDesired<<endl;
    }

    MyDetailedPolicy.Policy = Prev_ALL_AlternatePolicies[reRouteNeeded];
    MyDetailedPolicy.PolicyDQ = Prev_ALL_AlternatePolicyDQ[reRouteNeeded];
    return;  // std::make_pair(Prev_ALL_AlternatePolicyDQ[reRouteNeeded],Prev_ALL_AlternatePolicies[reRouteNeeded]);
  }

  altrouteSegmentList = AlternatePolicy_IN.routeplan;
  altpathPlan = AlternatePolicy_IN.pathPlan;
  auto altrouteSegmentList_OUT = altrouteSegmentList;
  auto altpathPlan_OUT = altpathPlan;
  int LastLaneSegmentUID =
      altrouteSegmentList_OUT
          .segment_uid[altrouteSegmentList_OUT.segment_count - 1];
  auto Missinglanesegments =
      BehaviorUtils::MapletRequestMissingLanePub(altrouteSegmentList);
  if (!Missinglanesegments.empty()) {
    BehaviorUtils::codeYellow(
        _CF_ +
            "Aborting Alternate plan generation as missing the lane segments " +
            altstr,
        Missinglanesegments, TimeGap);
    return;
  }

  if (altpathPlan.destination_included) {
    auto LastLaneSegment = BehaviorUtils::getLaneSeg(LastLaneSegmentUID);
    int next_laneseg_uid =
        (LastLaneSegment.connection[LastLaneSegment.primary_next_connection_n])
            .next_laneseg_uid;
    if (BehaviorUtils::MapletRequestMissingLanePub(next_laneseg_uid)) {
      BehaviorUtils::codeYellow(
          _CF_ +
              " Alternate plan generation compromised as missing the primary "
              "after last lane segment " +
              altstr,
          next_laneseg_uid, TimeGap);
      return;
    }
    BehaviorUtils::codeGreen(
        _CF_ + " Received the complete alternate plan lane segments ");

    // Create fake group
    altrouteSegmentList_OUT.segment_group.insert(
        altrouteSegmentList_OUT.segment_group.begin() +
            altrouteSegmentList.segment_count,
        altrouteSegmentList
                .segment_group[altrouteSegmentList.segment_count - 1] +
            100);
    altrouteSegmentList_OUT.segment_uid.insert(
        altrouteSegmentList_OUT.segment_uid.begin() +
            altrouteSegmentList.segment_count,
        next_laneseg_uid);
    altrouteSegmentList_OUT.segment_count =
        altrouteSegmentList_OUT.segment_uid.size();

    int lastGrpIdx =
        altrouteSegmentList_OUT
            .segment_group[altrouteSegmentList_OUT.segment_count - 1];
    int pathPlanEndIdx = altpathPlan_OUT.included_step_count - 1;
    int DubiousGrp =
        altpathPlan_OUT.path_steps[pathPlanEndIdx].target_segment_group;
    std::vector<int> altrouteSegGrp;
    for (int i = 0; i < altrouteSegmentList_OUT.segment_group.size(); i++)
      altrouteSegGrp.push_back(altrouteSegmentList_OUT.segment_group[i]);
    //	altrouteSegGrp = altrouteSegmentList_OUT.segment_group;
    while ((DubiousGrp == -1) ||
           !BehaviorUtils::find(altrouteSegGrp, DubiousGrp)) {
      altpathPlan_OUT.path_steps[pathPlanEndIdx].target_segment_group =
          lastGrpIdx;
      pathPlanEndIdx--;
      if (pathPlanEndIdx < 0) break;
      DubiousGrp =
          altpathPlan_OUT.path_steps[pathPlanEndIdx].target_segment_group;
    }
  }

  AlternatePolicy =
      Policy::InputPolicyType(altrouteSegmentList_OUT, altpathPlan_OUT);

  if (AlternatePolicy.routeplan.segment_group.empty()) {
    BehaviorUtils::codeRed(
        _CF_ + "Returning Empty plan as routeplan.segment_group empty " +
            altstr,
        " ", TimeGap);
    return;
  }

  std::vector<LaneSegUidType> TargetStateLanes = TargetState.CurrentGrpLanes;
  std::vector<LaneSegUidType> FrontStateLanes =
      Policy::ActiveDetailedPolicy.PolicyDQ[Policy::ActiveDetailedPolicy.Front]
          .CurrentGrpLanes;

  std::vector<LaneSegUidType> AlternatePolicyFirstStateLanes =
      BehaviorUtils::routeSegmentGrpTolanesegments(
          AlternatePolicy.routeplan.segment_group[0], AlternatePolicy);

  // Env::PolicyState PrevToTargetState =
  // Env::prev(TargetState,Policy::ActiveDetailedPolicy.PolicyDQ);

  double Origin_DAR_m =
      TargetState.GrpOrigin_DAR_m;  // TargetState.DAR_m - DAR_step;//Kind of
                                    // hard coding the fact the LCZ end if target
                                    // seg end station.

  Policy::BackEndDetailedPolicy.Policy = AlternatePolicy;
  LocalDetailedPolicy.Policy = AlternatePolicy;

  // cout<<"Origin_DAR_m "<<Origin_DAR_m<<endl;
  Env::initStates(LocalDetailedPolicy, true, Origin_DAR_m, false);
  AlternatePolicyDQ = LocalDetailedPolicy.PolicyDQ;
  // cout<<" AlternatePolicy "<<AlternatePolicyDQ<<endl;

  for (int i = 0; i < AlternatePolicyDQ.size(); i++) {
    exlcm::pathstep_t pathstep = AlternatePolicyDQ[i].step;
    auto AlternatePolicyGrp = AlternatePolicy.routeplan.segment_group;
    if (!BU::find(AlternatePolicyGrp, pathstep.current_segment_group)) {
      BehaviorUtils::codeRed(_CF_ +
                                 "Returning as could not find a suitable match "
                                 "in Alternate plan for group " +
                                 altstr,
                             pathstep.current_segment_group, TimeGap);
      return;
    }
  }

  std::string alternatepolicystr =
      "<uid:" + std::to_string(AlternatePolicy.uid) + ">";
  if (PRINT_ALTPATHPLAN)
    BehaviorUtils::codeBlue(_CF_ +
                                " Successful compilation of AlternatePolicyDQ. "
                                "AlternatePolicyDQ = " +
                                alternatepolicystr,
                            AlternatePolicyDQ, 1);

  bool IsTgtStLnSameAsAltFrntStLns =
      (AlternatePolicyFirstStateLanes == TargetStateLanes);
  bool IsFrntStLnSameAsAltFrntStLns =
      (AlternatePolicyFirstStateLanes == FrontStateLanes);
  bool IsTgtStLnsNOTInsideAltFrntStLns =
      std::search(AlternatePolicyFirstStateLanes.begin(),
                  AlternatePolicyFirstStateLanes.end(),
                  TargetStateLanes.begin(), TargetStateLanes.end()) ==
      AlternatePolicyFirstStateLanes.end();
  bool IsFrntStLnsNOTInsideAltFrntStLns =
      std::search(AlternatePolicyFirstStateLanes.begin(),
                  AlternatePolicyFirstStateLanes.end(), FrontStateLanes.begin(),
                  FrontStateLanes.end()) ==
      AlternatePolicyFirstStateLanes.end();

  if (!IsTgtStLnSameAsAltFrntStLns && !IsFrntStLnSameAsAltFrntStLns &&
      IsTgtStLnsNOTInsideAltFrntStLns && IsFrntStLnsNOTInsideAltFrntStLns) {
    if (BehaviorUtils::codeRed(
            _CF_ + " AlternatePolicy doesn't begin from the current Target "
                   "state. AlternatePolicy size ",
            AlternatePolicyDQ.size(), 1)) {
      //	cout<<yellow_on<<" AlternatePolicyDQ[0] :
      //"<<color_off<<AlternatePolicyDQ.front()<<endl; 	cout<<yellow_on<<"
      //TargetState : "<<color_off<<TargetState<<endl; 	cout<<yellow_on<<"
      //FrontState :
      //"<<color_off<<Policy::ActiveDetailedPolicy.PolicyDQ[Policy::ActiveDetailedPolicy.Front]<<endl;
    }
  } else if (AlternatePolicy.pathPlan.reroute_status == alternaterouteNeeded) {
    Prev_ALL_AlternatePolicyDQ[alternaterouteNeeded] = AlternatePolicyDQ;
    Prev_ALL_AlternatePolicies[alternaterouteNeeded] = AlternatePolicy;
    //	alternaterouteNeeded = NO_REROUTE;
  }

  Policy::BackEndDetailedPolicy.Policy = Policy::ActiveDetailedPolicy.Policy;

  if (BehaviorUtils::codeRed(
          _CF_ +
              " Returning empty alternate plan as reached the end of "
              "SearchAlternatePolicy() function. " +
              altstr,
          " ", TimeGap)) {
    /*	cout<<" Following alternate input policies are available "<<endl;
    for(auto & plan:Policy::AllaltPolicies)
            lcmprint::PrintEnum(cout,(reroute_type_enum)plan.first);
    cout<<endl; */

    cout << " Following alternate policy DQs are available " << endl;
    for (auto& plan : Prev_ALL_AlternatePolicyDQ)
      lcmprint::PrintEnum(cout, (reroute_type_enum)plan.first);
    cout << endl;
  }

  return;
}

bool AlternatePolicyBehavior(
    const Env::PolicyState& TargetState, int alternaterouteNeeded, int rootLane,
    std::map<int, Policy::DetailedPolicy>& AllCandidatePolicies) {
  BehaviorTertiary::Behavior AlternateBehavior;
  // PolicyDequeType AlternatePolicyDQ;
  // Policy::InputPolicyType AltPolicy;
  Policy::DetailedPolicy AltDetailedPolicy;

  Policy::SearchAlternatePolicy(AltDetailedPolicy, TargetState,
                                alternaterouteNeeded);
  // AltPolicy = AltDetailedPolicy.Policy; AlternatePolicyDQ =
  // AltDetailedPolicy.PolicyDQ;

  if (AltDetailedPolicy.PolicyDQ.empty()) {
    // cout<<"AlternatePolicyDQ.empty. alternaterouteNeeded
    // "<<alternaterouteNeeded<<endl;
    return false;
  }

  Policy::BackEndDetailedPolicy.Policy = AltDetailedPolicy.Policy;
  Policy::BackEndDetailedPolicy.PolicyDQ = AltDetailedPolicy.PolicyDQ;
  Policy::BackEndDetailedPolicy.Front = 0;

  for (int i = 1; i < AltDetailedPolicy.PolicyDQ.size(); i++) {
    AltDetailedPolicy.PolicyDQ[i].Prev_State =
        &AltDetailedPolicy.PolicyDQ[i - 1];

    AltDetailedPolicy.PolicyDQ[i - 1].Next_State =
        &AltDetailedPolicy.PolicyDQ[i];
  }

  AlternateBehavior =
      BehaviorTertiary::Behavior(AltDetailedPolicy.PolicyDQ.front());
  AlternateBehavior = Policy::CreateBehaviorFromPolicyDeque(
      AlternateBehavior, AltDetailedPolicy.PolicyDQ, alternaterouteNeeded, 1, 1,
      false, true);
  Policy::BackEndDetailedPolicy = Policy::ActiveDetailedPolicy;

  AllCandidatePolicies[rootLane] = Policy::DetailedPolicy(
      alternaterouteNeeded, rootLane, 0, AlternateBehavior,
      AltDetailedPolicy.Policy, AltDetailedPolicy.PolicyDQ);

  for (int i = 1; i < AllCandidatePolicies[rootLane].PolicyDQ.size(); i++) {
    AllCandidatePolicies[rootLane].PolicyDQ[i].Prev_State =
        &AllCandidatePolicies[rootLane].PolicyDQ[i - 1];

    AllCandidatePolicies[rootLane].PolicyDQ[i - 1].Next_State =
        &AllCandidatePolicies[rootLane].PolicyDQ[i];
  }

  return true;
}

bool SoughtAndFoundAlternatePolicy(
    BehaviorTertiary::Behavior& BehaviorDesired,
    std::map<int, Policy::DetailedPolicy>& AllCandidatePolicies,
    int alternaterouteNeeded, int LanePolicyKey, bool LaneSAExists) {
  if (LaneSAExists) {
    if (AllCandidatePolicies.find(LanePolicyKey) !=
        AllCandidatePolicies.end()) {
      //	cout<<" Found candidate policy. LanePolicyKey
      //"<<LanePolicyKey<<endl;
      return true;
    } else if (Policy::AlternatePolicyBehavior(
                   BehaviorDesired.TargetState, alternaterouteNeeded,
                   LanePolicyKey, AllCandidatePolicies)) {
      //	cout<<" Found input policy but not candidate policy. Trying to
      //calculate Candidate policy. LanePolicyKey "<<LanePolicyKey<<endl;
      return false;
    } else {
      BehaviorDesired.alternaterouteNeeded.insert(alternaterouteNeeded);

      return false;
    }
  }
  return false;
}
}  // namespace Policy
