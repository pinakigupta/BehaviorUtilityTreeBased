/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

//#include <lcm/lcm_coretypes.h>

#ifndef __exlcm_objprediction_t_hpp__
#define __exlcm_objprediction_t_hpp__

#include <vector>
#include "trajectoryprediction_t.hpp"

namespace exlcm
{

class objprediction_t
{
    public:
        int8_t     version_n;

        int64_t    lane_segment_uid;

        int64_t    intersection_uid;

        int64_t    intersecting_intersection_id;

        int8_t     Stationary_Status;

        float      time_at_intersection_sec;

        float      time_to_intersection_sec;

        int8_t     relative_location;

        int8_t     predicted_relative_location;

        int8_t     expected_relative_maneuver;

        int8_t     expected_to_yield_f;

        int8_t     collision_possible;

        float      host_dist_to_conflict_m;

        float      dist_to_conflict_m;

        float      time_to_collision_sec;

        float      in_lane_speed_mps;

        float      in_lane_accel_mpss;

        float      lane_station_m;

        int8_t     behavior_type;

        int64_t    primary_target_object_id;

        float      x_m;

        float      y_m;

        float      velocity_mps;

        float      phi_rad;

        int8_t     trajectoryprediction_count;

        std::vector< exlcm::trajectoryprediction_t > objtrajectoryprediction;

};

}

#endif
