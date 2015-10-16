//
//  mavlink_unreal.cpp
//  dSim
//
//  Created by Hao Xu on 15/4/26.
//  Copyright (c) 2015年 xuhao. All rights reserved.
//

#include "mavlink_connector.h"
#include "../../dji_mavlink/dji_sdk_onboard/mavlink.h"
#include "dji_variable.h"
#include "dji_commands.h"

#define FLIGHT_STATUS_UNKNOWN 0
#define FLIGHT_STATUS_STANDBY 1
#define FLIGHT_STATUS_TAKINGOFF 2
#define FLIGHT_STATUS_INAIR 3
#define FLIGHT_STATUS_LANDING 4
#define FLIGHT_STATUS_POSTLANDING 5

#define MISSION_REC_STANDBY 0
#define MISSION_REC_WAIT_FOR 1

#include <vector>

namespace mavlink_adapter
{

    void mavlink_connector::init_network(std::string ip, int port)
    {
        //create a UDP socket
        if ((socket_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            printf("Socket Create failed");
        }

        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(port);
        if (inet_aton(ip.c_str(), &si_other.sin_addr) == 0) {
            fprintf(stderr, "inet_aton() failed\n");
        }

        printf("build udp 2 %s via port %d\n", ip.c_str(), port);

        addr_len = sizeof(struct sockaddr_in);
    }


    int mavlink_connector::write(const char *s, int len)
    {
        const char *buf = s;
        int slen = sizeof(si_other);
        if (sendto(socket_s, buf, len, 0, (sockaddr *) &si_other, slen) == -1) {
            printf("failed:len %d\n", len);
            return -1;
        }

        return 0;
    }
    mavlink_heartbeat_t * mavlink_connector::make_heartbeat()
    {
        heartbeat_t->system_status = MAV_STATE_ACTIVE;

        switch (dji_variable::flight_status) {
            case FLIGHT_STATUS_LANDING:
            case FLIGHT_STATUS_POSTLANDING:
            case FLIGHT_STATUS_TAKINGOFF:
                heartbeat_t->base_mode = MAV_MODE_AUTO_ARMED;
                break;
            case FLIGHT_STATUS_INAIR:
                heartbeat_t->base_mode = MAV_MODE_MANUAL_ARMED;
                break;
            default:
                heartbeat_t->base_mode = MAV_MODE_MANUAL_DISARMED;
        }

        heartbeat_t->autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat_t->type = MAV_TYPE_QUADROTOR;
        heartbeat_t->mavlink_version = 3;
        return heartbeat_t;
    }

    void mavlink_connector::slow_send()
    {

        mavlink_gps_raw_int_t gps_raw_int_t;
        mavlink_battery_status_t battery_status_t;
        mavlink_rc_channels_t rc_channels_t;

        mavlink_message_t msg;


        gps_raw_int_t.lat = (int32_t)(dji_variable::global_position_degree.lati * 1e7) ;
        gps_raw_int_t.lon = (int32_t)(dji_variable::global_position_degree.longti * 1e7);
        gps_raw_int_t.alt = (int32_t)(dji_variable::global_position_degree.alti * 1000);

        mavlink_msg_gps_raw_int_encode(0,200,&msg,&gps_raw_int_t);
        send_msg(&msg);


        mavlink_msg_heartbeat_encode(0,200,&msg,make_heartbeat());
        send_msg(&msg);
//        printf("sended heartbeating......\n");

        battery_status_t.battery_remaining = dji_variable::battery;
        for (int i = 0 ;i< 6;i++)
            battery_status_t.voltages[i] = (uint16_t)((float)dji_variable::battery  / 100.0f * 22.2 * 1000);

        mavlink_msg_battery_status_encode(0,200,&msg,&battery_status_t);
        send_msg(&msg);

        mavlink_sys_status_t sys_status_t;
        memset(&sys_status_t,0, sizeof(sys_status_t));
        sys_status_t.battery_remaining = dji_variable::battery;
        sys_status_t.voltage_battery = 222 * dji_variable::battery ;

        mavlink_msg_sys_status_encode(0,200,&msg,&sys_status_t);
        send_msg(&msg);

        mavlink_rc_channels_scaled_t rc_channels_scaled_t;
        rc_channels_scaled_t.chan1_scaled = dji_variable::rc_channels.roll;
        rc_channels_scaled_t.chan2_scaled = dji_variable::rc_channels.pitch;
        rc_channels_scaled_t.chan3_scaled = dji_variable::rc_channels.throttle;
        rc_channels_scaled_t.chan4_scaled = dji_variable::rc_channels.yaw;
        rc_channels_scaled_t.chan5_scaled = dji_variable::rc_channels.mode;
        rc_channels_scaled_t.chan6_scaled = dji_variable::rc_channels.gear;

        mavlink_msg_rc_channels_scaled_encode(0,200,&msg,&rc_channels_scaled_t);
        send_msg(&msg);

    }

    void mavlink_connector::fast_send()
    {

        mavlink_attitude_quaternion_t att;
        mavlink_local_position_ned_t pos;
        mavlink_global_position_int_t global_position_int_t;
        mavlink_message_t msg;

        att.q1 = dji_variable::attitude_quad.q0;
        att.q2 = dji_variable::attitude_quad.q1;
        att.q3 = dji_variable::attitude_quad.q2;
        att.q4 = dji_variable::attitude_quad.q3;

        att.pitchspeed = dji_variable::w.y;
        att.rollspeed = dji_variable::w.x;
        att.yawspeed = dji_variable::w.z;

        pos.x = dji_variable::local_position.x;
        pos.y = dji_variable::local_position.y;
        pos.z = dji_variable::local_position.z;

        pos.vx = dji_variable::velocity.x;
        pos.vy = dji_variable::velocity.y;
        pos.vz = dji_variable::velocity.z;


        global_position_int_t.lat = (int32_t)(dji_variable::global_position_degree.lati * 1e7) ;
        global_position_int_t.lon = (int32_t)(dji_variable::global_position_degree.longti * 1e7);
        global_position_int_t.alt = (int32_t)(dji_variable::global_position_degree.alti * 1000);
        global_position_int_t.relative_alt =(int32_t) (dji_variable::global_position_degree.height * 1000);
        global_position_int_t.vx = (int16_t)(pos.vx * 100);
        global_position_int_t.vy = (int16_t)(pos.vy * 100);
        global_position_int_t.vz = (int16_t)(pos.vz * 100);


        mavlink_msg_attitude_quaternion_encode(0, 200, &msg, &att);
        send_msg(&msg);
        mavlink_msg_local_position_ned_encode(0, 200, &msg, &pos);
        send_msg(&msg);
        mavlink_msg_global_position_int_encode(0,200,&msg,&global_position_int_t);
        send_msg(&msg);

    }

    int mavlink_connector::send_msg(mavlink_message_t * msg)
    {

        int len = mavlink_msg_to_send_buffer((uint8_t *) buffer, msg);
        write(buffer, len);
        return 0;
    }

    void mavlink_connector::recv()
    {
        bzero(rec_buffer,sizeof(rec_buffer));
        int len =(int) recvfrom(socket_s,(void*)rec_buffer,sizeof(rec_buffer), 0 , (struct sockaddr *)&si_other ,(socklen_t *)&addr_len);
        handle_mavlink(rec_buffer,len);
    }

    mavlink_connector::mavlink_connector(std::string ip, int port)
    {
        init_network(ip, port);
        heartbeat_t = new mavlink_heartbeat_t;
    }

   void mavlink_connector::handle_mavlink(char *buffer, int len)
   {
       mavlink_message_t msg;
       mavlink_status_t status;
       for (ssize_t i = 0; i < len; i++) {
           if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status)) {
               handle_message(&msg);
           }
       }
   }

    void mavlink_connector::handle_message(mavlink_message_t *msg)
    {
        switch (msg->msgid)
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
//                printf("I can hear you heart\n");
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handle_command_long(msg);
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            case MAVLINK_MSG_ID_MISSION_REQUEST:
            case MAVLINK_MSG_ID_MISSION_COUNT:
            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            case MAVLINK_MSG_ID_MISSION_ITEM:
                handle_missions(msg);
            default:
                printf("msg !!!!!!%d\n",msg->msgid);
        }
    }

    void mavlink_connector::handle_command_long(mavlink_message_t *msg)
    {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg,&cmd);
        switch(cmd.command)
        {
            case MAV_CMD_NAV_TAKEOFF:
                dji_commands::set_takeoff();
                printf("recv takeof..\n");
                break;

            case MAV_CMD_NAV_LAND:
                dji_commands::set_land();
                printf("land mode..\n");
                break;

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                dji_commands::set_return2home();
                printf("return to home\n");
                break;
            default:
                printf("cmd is %d\n",cmd.command);
        }
    }

    void mavlink_connector::handle_missions(mavlink_message_t *msg)
    {
        mavlink_message_t resp;
        switch (msg->msgid)
        {
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                mavlink_mission_count_t count;
                count.count = dji_variable::wp_m.missions[0].size();
                count.target_component = msg->compid;
                count.target_system = msg->sysid;
                mavlink_msg_mission_count_encode(0, 200, &resp, &count);
                this->send_msg(&resp);
                break;
            }
            case MAVLINK_MSG_ID_MISSION_REQUEST: {
                mavlink_mission_request_t req;
                mavlink_msg_mission_request_decode(msg, &req);
//                printf("ground is trying to request mission %d \n",req.seq);
                std::vector<mavlink_mission_item_t> mission_msgs
                        = dji_variable::wp_m.missions[0].to_mission_items();
                mavlink_mission_item_t a = mission_msgs[req.seq];
                a.target_system = msg->sysid;
                a.target_component = msg->compid;
                a.current =  (a.seq == dji_variable::wp_m.waypoint_ptr && dji_variable::wp_m.mission_id == 0)? 1:0;
                mavlink_msg_mission_item_encode(0,200, &resp, &a);
                this->send_msg(&resp);

                break;
            }
            case MAVLINK_MSG_ID_MISSION_COUNT: {
                mavlink_mission_count_t mission_count_t;
                mavlink_msg_mission_count_decode(msg, &mission_count_t);
                if (MISSION_REC_STATE == MISSION_REC_STANDBY) {
                    printf("Ground is trying to set a waypoint long as %d\n", mission_count_t.count);
                    waypoint_length_size = mission_count_t.count;
                    waypoint_waitfor = 0;
                    dji_variable::wp_m.missions[0].clear();
                    MISSION_REC_STATE = MISSION_REC_WAIT_FOR;
                    mavlink_mission_request_t request_t;
                    request_t.target_component = msg->compid;
                    request_t.target_system = msg->sysid;
                    request_t.seq = 0;
                    mavlink_msg_mission_request_encode(0,200,&resp,&request_t);
                    this->send_msg(&resp);
                    break;
                }
            };
            case MAVLINK_MSG_ID_MISSION_ITEM:
            {
                if (MISSION_REC_STATE != MISSION_REC_WAIT_FOR)
                    break;
                mavlink_mission_item_t mission_item_t;
                mavlink_msg_mission_item_decode(msg,&mission_item_t);
                printf("seq:%d %f %f\n",mission_item_t.seq,mission_item_t.x,mission_item_t.y);
                api_pos_custom_data_t wp;
                wp.lati = mission_item_t.x /180 * M_PI;
                wp.longti = mission_item_t.y /180 * M_PI;
                wp.alti = mission_item_t.z;
//                wp.uncertain = mission_item_t.
                wp.uncertain = 10 ;

                mavlink_mission_request_t request_t;
                request_t.target_component = msg->compid;
                request_t.target_system = msg->sysid;

                if (mission_item_t.seq == waypoint_waitfor)
                {
                    waypoint_waitfor ++;
                    request_t.seq = waypoint_waitfor;
                    dji_variable::wp_m.missions[0].waypoints.push_back(wp);
                    if (request_t.seq < waypoint_length_size) {
                        mavlink_msg_mission_request_encode(0, 200, &resp, &request_t);
                        this->send_msg(&resp);
                    }
                    else{
                        std::cout << dji_variable::wp_m.missions[0];
                        waypoint_waitfor = 0;
                        waypoint_length_size = 0;
                        MISSION_REC_STATE = MISSION_REC_STANDBY;
                        mavlink_mission_ack_t ack_t;
                        ack_t.target_component = msg->compid;
                        ack_t.target_system = msg->sysid;
                        ack_t.type = MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED;
                        mavlink_msg_mission_ack_encode(0,200,&resp,&ack_t);
                        this->send_msg(&resp);
                    }
                }
                else{
                    request_t.seq = waypoint_waitfor;
                    mavlink_msg_mission_request_encode(0, 200, &resp, &request_t);
                    this->send_msg(&resp);
                }
                break;

            };
            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            {
                mavlink_mission_set_current_t  set_current_t;
                mavlink_msg_mission_set_current_decode(msg,&set_current_t);
                int ptr = set_current_t.seq;
                printf("say system is trying to set current %d \n",ptr);
                dji_variable::wp_m.mission_id = 0;
                dji_variable::wp_m.waypoint_ptr = ptr;
                dji_variable::wp_m.begin_fly_waypoints(0,ptr);
                mavlink_mission_ack_t ack_t;
                ack_t.target_component = msg->compid;
                ack_t.target_system = msg->sysid;
                ack_t.type = MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED;
                mavlink_msg_mission_ack_encode(0,200,&resp,&ack_t);
                this->send_msg(&resp);
            };

            default:
                break;
        }
    }

    void mavlink_connector::handle_local_position_sp(mavlink_message_t *msg)
    {
        printf("setted \n");
    }

};