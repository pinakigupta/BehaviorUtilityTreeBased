/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

//#include <lcm/lcm_coretypes.h>

#ifndef __exlcm_lanegapvisualization_t_hpp__
#define __exlcm_lanegapvisualization_t_hpp__

#include "behaviorlanegapsviz_t.hpp"
#include "behaviorlanegapsviz_t.hpp"
#include "behaviorlanegapsviz_t.hpp"
#include "behaviorlanegapsviz_t.hpp"

namespace exlcm
{

/// MAL 2017-04-26
class lanegapvisualization_t
{
    public:
        int16_t    entity_type;

        int8_t     version_n;

        int8_t     valid_f;

        int64_t    ref_n;

        double     timestamp_sec;

        exlcm::behaviorlanegapsviz_t Current_Lane_Pred_Gap;

        exlcm::behaviorlanegapsviz_t Target_Lane_Pred_Gap;

        exlcm::behaviorlanegapsviz_t All_Current_Lane_Gaps;

        exlcm::behaviorlanegapsviz_t All_Target_Lane_Gaps;

};
}

#endif
