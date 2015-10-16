
#include "math.h"

#include "stdio.h"

#include "DJI_Pro_App.h"
namespace gimbal
{

    float gimbal_yaw_control_sp = 0;
    float gimbal_lookat_x,
            gimbal_lookat_y,
            gimbal_lookat_z;
    bool gimbal_lookat_enable = false;
    //TODO: Gimabl
    void send_gimbal_angle(float yaw, float roll, float pitch)
    {
//        gimbal_custom_control_angle_t send_data = {0};
//
//        send_data.yaw_angle = floor(yaw * 10.0f);  // unit 0.1 degree
//        send_data.roll_angle = floor(roll * 10.0f);
//        send_data.pitch_angle = floor(pitch * 10.0f);
//        send_data.duration = 10;
//        send_data.ctrl_byte.base = 1;
//        App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_GIMBAL_ANGLE, (uint8_t *) &send_data, sizeof(send_data), NULL, 0,
//                      0);
    }

    void control(float x, float y, float z)
    {

        float dx = gimbal_lookat_x - x;
        float dy = gimbal_lookat_y - y;
        float dz = gimbal_lookat_z - z;


        float theta = atan2(dy,dx) * 180.0f / M_PI;

        if (theta > 180)
            theta -= 360;
        if (theta < -180)
            theta += 360;

        float fai = atan(dz / sqrt(dx * dx + dy * dy)) * 180.0f / M_PI;

        if (dx * dx + dy * dy < 1e-2) {
            fai = 0;
        }

        gimbal::gimbal_yaw_control_sp = theta;
        send_gimbal_angle(0, 0, fai);

    }

    void look_at(api_common_data_t lop)
    {
        gimbal::gimbal_lookat_enable = true;
        gimbal::gimbal_lookat_x = lop.x;
        gimbal::gimbal_lookat_y = lop.y;
        gimbal::gimbal_lookat_z = lop.z;
    }

};
