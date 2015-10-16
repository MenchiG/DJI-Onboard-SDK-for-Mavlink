//============================================================================
// Name        : main.cpp
// Author      : wuyuwei
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard API test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <unistd.h>
#include <thread>
#include "DJI_Pro_Sample.h"
#include "dji_mavlink_adapter.h"
#include "dji_variable.h"
#include "dji_gimbal.h"
using namespace std;
using namespace dji_variable;
void update_vars()
{
	sdk_std_msg_t dji_flight_data;
    DJI_Pro_Get_Broadcast_Data(&dji_flight_data);
    uint32_t times_stamp = dji_flight_data.time_stamp;

   // auto current_time = ros::Time::now();

    attitude_quad.q0 = dji_flight_data.q.q0;
    attitude_quad.q1 = dji_flight_data.q.q1;
    attitude_quad.q2 = dji_flight_data.q.q2;
    attitude_quad.q3 = dji_flight_data.q.q3;

    w.x = dji_flight_data.w.x;
    w.y = dji_flight_data.w.y;
    w.z = dji_flight_data.w.z;


    global_position.lati = dji_flight_data.pos.lati;
    global_position.longti = dji_flight_data.pos.longti;
    global_position.height = dji_flight_data.pos.height;
    global_position.alti = dji_flight_data.pos.alti;

    global_position_degree = global_position;

    global_position_degree.lati = global_position.lati * 180.0f /M_PI;
    global_position_degree.longti = global_position.longti * 180.0f /M_PI;

    static int seted = 0;
    //TODO:
    // FIX BUG about flying at lat = 0
    // if (global_position.ts != 0 && seted == 0 && global_position.lat != 0) {
    //     dji_variable::global_position_ref = global_position;
    //     seted = 1;
    // }

    velocity.x = dji_flight_data.v.x;
    velocity.y = dji_flight_data.v.y;
    velocity.z = dji_flight_data.v.z;

    acc.x = dji_flight_data.a.x;
    acc.y = dji_flight_data.a.y;
    acc.z = dji_flight_data.a.z;

    dji_variable::gps_convert_ned(
            local_position.x,
            local_position.y,
            global_position.longti,
            global_position.lati,
            dji_variable::global_position_ref.longti,
            dji_variable::global_position_ref.lati
    );


    local_position.z = global_position.height;

    dji_variable::local_position_ref = local_position;


    if (gimbal::gimbal_lookat_enable) {
        gimbal::control(
                local_position.x,
                local_position.y,
                local_position.z
        );
    }

    rc_channels.pitch = dji_flight_data.rc.pitch;
    rc_channels.roll = dji_flight_data.rc.roll;
    rc_channels.mode = dji_flight_data.rc.mode;
    rc_channels.gear = dji_flight_data.rc.gear;
    rc_channels.throttle = dji_flight_data.rc.throttle;
    rc_channels.yaw = dji_flight_data.rc.yaw;

    ctrl_device = dji_flight_data.ctrl_info.cur_ctrl_dev_in_navi_mode;
    flight_status = dji_flight_data.status;

    battery = dji_flight_data.battery_remaining_capacity;
//    recv_sdk_std_msgs.status

}
int main(int argc,char **argv)
{
	int main_operate_code = 0;
	int temp32;
	activate_data_t user_act_data;
	char temp_buf[65];
	char app_bundle_id[32] = "1234567890";

	if(argc == 2 && strcmp(argv[1],"-v") == 0)
	{
		printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");
		return 0;
	}
	printf("\nDJI Onboard API Cmdline Test,Ver 1.1.0\n\n");

	if(DJI_Sample_Setup() < 0)
	{
		printf("Serial Port Open ERROR\n");
		return 0;
	}

	user_act_data.app_key = temp_buf;
	user_act_data.app_ver = SDK_VERSION;
	strcpy((char*)user_act_data.app_bundle_id, app_bundle_id);
    if(DJI_Pro_Get_Cfg(NULL,NULL,&user_act_data.app_id,&user_act_data.app_api_level,user_act_data.app_key) == 0)
	{
		/* user setting */
		printf("--------------------------\n");
		printf("app id=%d\n",user_act_data.app_id);
		printf("app api level=%d\n",user_act_data.app_api_level);
		printf("app key=%s\n",user_act_data.app_key);
		printf("--------------------------\n");

		DJI_Pro_Activate_API(&user_act_data,NULL);
		DJI_Pro_Control_Management(1,NULL);
	}
	else
	{
		printf("ERROR:There is no user account\n");	
		return 0;
	}

 	mavlink_adapter::set_mavlink("10.60.23.178",14550);
   	std::thread th_rec([&]{
       mavlink_adapter::recv_function();
    }
    );
	while(1)
	{

    	update_vars();
    	mavlink_adapter::loop_callback(NULL);
    	dji_variable::wp_m.loop();
    	usleep(20000);

	}

	return 0;
}
