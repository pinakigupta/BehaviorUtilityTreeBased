/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

//#include <lcm/lcm_coretypes.h>

#ifndef __exlcm_lanegap_t_hpp__
#define __exlcm_lanegap_t_hpp__

#include <vector>
#include "lanegappred_t.hpp"
#include "lanegappred_t.hpp"
#include "objdetails_t.hpp"

namespace exlcm
{

class lanegap_t
{
    public:
        int8_t     gap_UID;

        int8_t     gap_pred_count;

        std::vector< exlcm::lanegappred_t > gap_predictions;

        exlcm::lanegappred_t optimal_predicted_gap;

        float      gap_occupancy_cost;

        int8_t     relevant_obj_count;

        exlcm::objdetails_t relevant_objs[3];

};
}

#endif
