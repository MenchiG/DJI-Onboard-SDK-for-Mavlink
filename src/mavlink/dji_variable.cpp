#include <cmath>
#include "DJI_Pro_App.h"

#include "dji_waypoints.h"
#include "dji_variable.h"

#define C_EARTH (double) 6378137.0
#define C_PI    (double) 3.141592653589793

namespace dji_variable
{

    api_common_data_t local_position_ref;
    api_pos_custom_data_t global_position_ref;
    bool localposbase_use_height = true;
    api_quaternion_data_t attitude_quad;
    api_vel_data_t velocity;
    api_common_data_t acc;
    api_common_data_t w;
    api_rc_data_t rc_channels;
    api_pos_custom_data_t global_position;
    api_pos_custom_data_t global_position_degree;
    api_common_data_t local_position;
   // nav_msgs::Odometry odem;
    uint8_t flight_status;
    uint8_t ctrl_device;
    float battery = 0;
    bool opened = false;
    bool activated = false;
    bool flag_visit = false;
    bool flag_success = false;
    bool flag_open_or_close = true;

    void gps_convert_ned(float &ned_x, float &ned_y,
                         double gps_t_lon,
                         double gps_t_lat,
                         double gps_r_lon,
                         double gps_r_lat
    )
    {

        //TODO :
        //fix bug with ellipsoid

        double d_lon = gps_t_lon - gps_r_lon;
        double d_lat = gps_t_lat - gps_r_lat;

        ned_x = d_lat * C_EARTH;
        ned_y = d_lon * C_EARTH * cos((gps_r_lat + gps_t_lat) / 2 * M_PI / 180.0f);
        return;
    }
    
    api_common_data_t gps_convert_ned(api_pos_custom_data_t loc)
    {
        api_common_data_t local;
        gps_convert_ned(
          local.x,
          local.y,
          loc.longti,
          loc.lati,
          global_position_ref.longti,
          global_position_ref.lati
        );
        local.z = global_position.height;
        return local;
    }

    dji_waypoints wp_m;
};
