#ifndef __DJI_SDK_LOCALS_H__
#define __DJI_SDK_LOCALS_H__


typedef struct
{
    double lati;
    double longti;
    float alti;
    float height;
    unsigned char health_flag;
    float uncertain;
    unsigned int cmd;
}api_pos_custom_data_t;


#include "dji_waypoints.h"
#include "DJI_Pro_App.h"


namespace dji_variable
{   
    extern bool flag_visit;
    extern bool flag_success;
    extern bool flag_open_or_close;
    extern api_common_data_t local_position_ref;
    extern api_pos_custom_data_t global_position_ref;
    extern bool localposbase_use_height;
    extern api_quaternion_data_t attitude_quad;
    extern api_vel_data_t velocity;
    extern api_common_data_t acc;
    extern api_common_data_t w;
    extern api_rc_data_t rc_channels;
    extern api_pos_custom_data_t global_position;
    extern api_pos_custom_data_t global_position_degree;
    extern api_common_data_t local_position;
    extern float battery;
    extern uint8_t  flight_status;
    extern uint8_t ctrl_device;
  //  extern nav_msgs::Odometry odem;
    extern bool opened ;
    extern bool activated;
    void gps_convert_ned(float &ned_x, float &ned_y,
                     double gps_t_lon,
                     double gps_t_lat,
                     double gps_r_lon,
                     double gps_r_lat
    );
    api_common_data_t gps_convert_ned(api_pos_custom_data_t loc);
    extern dji_waypoints wp_m;
};

#endif
    
