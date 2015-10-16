#include "dji_sdk_node.h"
#include <dji_sdk/dji_ros_modules.h>
#include <sdk_lib/DJI_Pro_App.h>
#include <dji_sdk/mavlink_connector.h>
#include <thread>
#include <functional>
#include <dji_sdk/dji_waypoints.h>

using namespace dji_variable;
void update_ros_vars()
{

    static int frame_id = 0;
    frame_id ++;
    auto current_time = ros::Time::now();

    attitude_quad.q0 = recv_sdk_std_msgs.q.q0;
    attitude_quad.q1 = recv_sdk_std_msgs.q.q1;
    attitude_quad.q2 = recv_sdk_std_msgs.q.q2;
    attitude_quad.q3 = recv_sdk_std_msgs.q.q3;
    attitude_quad.wx = recv_sdk_std_msgs.w.x;
    attitude_quad.wy = recv_sdk_std_msgs.w.y;
    attitude_quad.wz = recv_sdk_std_msgs.w.z;
    attitude_quad.ts = recv_sdk_std_msgs.time_stamp;

    global_position.lat = recv_sdk_std_msgs.pos.lati;
    global_position.lon = recv_sdk_std_msgs.pos.longti;
    global_position.height = recv_sdk_std_msgs.pos.height;
    global_position.alti = recv_sdk_std_msgs.pos.alti;
    global_position.ts = recv_sdk_std_msgs.time_stamp;

    global_position_degree = global_position;

    global_position_degree.lat = global_position.lat * 180.0f /M_PI;
    global_position_degree.lon = global_position.lon * 180.0f /M_PI;

    static int seted = 0;
    //TODO:
    // FIX BUG about flying at lat = 0
    if (global_position.ts != 0 && seted == 0 && global_position.lat != 0) {
        dji_variable::global_position_ref = global_position;
        seted = 1;
    }

    velocity.ts = recv_sdk_std_msgs.time_stamp;
    velocity.velx = recv_sdk_std_msgs.v.x;
    velocity.vely = recv_sdk_std_msgs.v.y;
    velocity.velz = recv_sdk_std_msgs.v.z;

    acc.ax = recv_sdk_std_msgs.a.x;
    acc.ay = recv_sdk_std_msgs.a.y;
    acc.az = recv_sdk_std_msgs.a.z;

    dji_variable::gps_convert_ned(
            local_position.x,
            local_position.y,
            global_position.lon,
            global_position.lat,
            dji_variable::global_position_ref.lon,
            dji_variable::global_position_ref.lat
    );


    local_position.height = global_position.height;
    local_position.ts = global_position.ts;
    dji_variable::local_position_ref = local_position;

    odem.header.frame_id = "dji_sys_0";
    odem.header.stamp = current_time;

    odem.pose.pose.position.x = local_position.x;
    odem.pose.pose.position.y = local_position.y;
    odem.pose.pose.position.z = local_position.height;

    odem.pose.pose.orientation.w = attitude_quad.q0;
    odem.pose.pose.orientation.x = attitude_quad.q1;
    odem.pose.pose.orientation.y = attitude_quad.q2;
    odem.pose.pose.orientation.z = attitude_quad.q3;

    odem.twist.twist.angular.x = attitude_quad.wx;
    odem.twist.twist.angular.y = attitude_quad.wy;
    odem.twist.twist.angular.z = attitude_quad.wz;

    odem.twist.twist.linear.x = velocity.velx;
    odem.twist.twist.linear.y = velocity.vely;
    odem.twist.twist.linear.z = velocity.velz;

    publishers::odem_publisher.publish(odem);

    if (gimbal::gimbal_lookat_enable) {
        gimbal::control(
                local_position.x,
                local_position.y,
                local_position.height
        );
    }

    rc_channels.pitch = recv_sdk_std_msgs.rc.pitch;
    rc_channels.roll = recv_sdk_std_msgs.rc.roll;
    rc_channels.mode = recv_sdk_std_msgs.rc.mode;
    rc_channels.gear_up = recv_sdk_std_msgs.rc.gear;
    rc_channels.throttle = recv_sdk_std_msgs.rc.throttle;
    rc_channels.yaw = recv_sdk_std_msgs.rc.yaw;

    ctrl_device = recv_sdk_std_msgs.ctrl_device;
    flight_status = recv_sdk_std_msgs.status;

    battery = recv_sdk_std_msgs.battery_remaining_capacity;
//    recv_sdk_std_msgs.status

    publishers::local_pos_pub.publish(local_position);

    publishers::att_quad_pub.publish(attitude_quad);

    publishers::gps_pub.publish(global_position);

    publishers::vel_pub.publish(velocity);

    publishers::acc_pub.publish(acc);

    publishers::rc_channels_pub.publish(rc_channels);
}


//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------
void spin_callback(const ros::TimerEvent &e)
{

    update_ros_vars();
    mavlink_adapter::loop_callback(recv_sdk_std_msgs.time_stamp);
    dji_variable::wp_m.loop();

  }
//----------------------------------------------------------
// main_function
//----------------------------------------------------------
int main(int argc, char **argv)
{
    printf("SDK Protocol\n");
    // initialize ros
    ros::init(argc, argv, "SDK_serial");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("serial_name", serial_name, std::string("/dev/ttyACM0"));    // /dev/ttySAC0 or /dev/ttyUSB0
    nh_private.param("baud_rate", baud_rate, 230400);

    nh_private.param("app_id", app_id,1009412);
    nh_private.param("app_api_level", app_api_level, 2);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));

    nh_private.param("enc_key", enc_key,
                     std::string("5e758c5f4e76da52a202550e7923884f760decea78587f21d1ea91c1f15d1d7b"));

    std::string mavlink_ip;
    int port;

    nh_private.param("mavlink_ip",mavlink_ip,std::string("127.0.0.1"));
    nh_private.param("mavlink_port",port,14550);

    mavlink_adapter::set_mavlink(mavlink_ip,port);
    std::thread th_rec([&]{
       mavlink_adapter::recv_function();
    }
    );

    activation_msg.app_id = (uint32_t) app_id;
    activation_msg.app_api_level = (uint32_t) app_api_level;
    activation_msg.app_ver = (uint32_t) app_version;
    memcpy(activation_msg.app_bundle_id, app_bundle_id.c_str(), 32);

    key = (char *) enc_key.c_str();


    printf("[INIT] SET serial_port	: %s \n", serial_name.c_str());
    printf("[INIT] SET baud_rate	: %d \n", baud_rate);
    printf("[INIT] ACTIVATION INFO	: \n");
    printf("[INIT] 	  app_id     	  %d \n", activation_msg.app_id);
    printf("[INIT]    app_api_level	  %d \n", activation_msg.app_api_level);
    printf("[INIT]    app_version     %d \n", activation_msg.app_ver);
    printf("[INIT]    app_bundle_id	  %s \n", activation_msg.app_bundle_id);
    printf("[INIT]    enc_key	  %s \n", key);


    publishers::init_publishers(nh);
    service_handles::init_services(nh);
    init_subscibers(nh);


    //wp_m.load("/Users/xuhao/data/wp.txt");

    // ros timer 50Hz
    simple_task_timer = nh.createTimer(ros::Duration(1.0 / 50.0), (const TimerCallback &) spin_callback);
    // open serial port
    Pro_Hw_Setup((char *) serial_name.c_str(), baud_rate);
    Pro_Link_Setup();


    App_Recv_Set_Hook(App_Recv_Req_Data);
    App_Set_Table(set_handler_tab, cmd_handler_tab);

    activate();
    
    CmdStartThread();



    ros::spin();

    return 0;
}
