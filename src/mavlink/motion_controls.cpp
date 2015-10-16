//
// Created by Hao Xu on 15/5/5.
//

#include "DJI_Pro_App.h"
#include "dji_variable.h"
#include "dji_gimbal.h"


namespace motion_controls
{
    void fly_to_localpos(api_common_data_t los,
                         bool use_height
    )
    {
        //TODO: ALT and HEIGHT
        attitude_data_t send_data = {0};


        send_data.ctrl_flag = 0x90; 
        send_data.roll_or_x = los.x - dji_variable::local_position_ref.x;
        send_data.pitch_or_y = los.y - dji_variable::local_position_ref.y;
        send_data.thr_z = los.z; //m/s
        send_data.yaw = gimbal::gimbal_yaw_control_sp;

        printf("%f %f %f \n",
               los.x - dji_variable::local_position_ref.x,
               los.y - dji_variable::local_position_ref.y,
               los.z
        );

        DJI_Pro_Attitude_Control(&send_data);
    }

    void fly_to_globalpos(api_pos_custom_data_t los,
                          bool use_height
    )
    {
        //TODO: ALT and HEIGHT
        api_common_data_t move ;
//        printf("los %f %f %f\n",los.lon,los.lat,los.height);
//        printf("reflos %f %f %f\n",dji_variable::global_position_ref.lon,dji_variable::global_position_ref.lat,los.height);
        move = dji_variable::gps_convert_ned(los);
        move.z = los.alti - dji_variable::global_position_ref.alti;
//        printf("move %f %f %f\n",move.x,move.y,move.height);
        fly_to_localpos(move, false);
    }
    void set_velocity(api_common_data_t msg)
    {

        attitude_data_t send_data = {0};

        send_data.ctrl_flag = 68; 
        send_data.roll_or_x = msg.x;
        send_data.pitch_or_y = msg.y;
        send_data.thr_z = msg.z; //m/s
        send_data.yaw = gimbal::gimbal_yaw_control_sp;

        DJI_Pro_Attitude_Control(&send_data);
    }

};
