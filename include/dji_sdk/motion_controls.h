#ifndef __DJI_SDK_MOTION_CONTROLS_H__
#define __DJI_SDK_MOTION_CONTROLS_H__

#include "dji_variable.h"
#include "DJI_Pro_App.h"

namespace motion_controls
{
    void fly_to_localpos(api_common_data_t los,
                         bool use_height
    );

    void fly_to_globalpos(api_pos_custom_data_t los,
                         bool use_height
    );

    void set_velocity(api_common_data_t msg);
};

#endif