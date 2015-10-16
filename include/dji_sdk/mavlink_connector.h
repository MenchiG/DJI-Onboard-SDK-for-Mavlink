//
//  mavlink_unreal.h
//  dSim
//
//  Created by Hao Xu on 15/4/26.
//  Copyright (c) 2015年 xuhao. All rights reserved.
//

#ifndef __dSim__mavlink_unreal__
#define __dSim__mavlink_unreal__

#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <ostream>
#include <vector>
#include "../dji_mavlink/dji_sdk_onboard/mavlink.h"

namespace mavlink_adapter
{
    class mavlink_connector
    {

//        struct sockaddr_in addr;
        int addr_len;

        int socket_s;
        struct sockaddr_in si_other;

        void init_network(std::string ip, int port);

        int write(const char *s, int len);

        int send_msg(mavlink_message_t *msg);

        char buffer[1024];
        char rec_buffer[1024];

        void handle_mavlink(char * buffer,int len);

        void handle_message(mavlink_message_t * msg);

        void handle_local_position_sp(mavlink_message_t* msg);

        void handle_command_long(mavlink_message_t * msg);

        mavlink_heartbeat_t * heartbeat_t;

        mavlink_heartbeat_t * make_heartbeat();

        void handle_missions(mavlink_message_t * msg);

        int MISSION_REC_STATE = 0;

        int waypoint_waitfor = 0;
        int waypoint_length_size = 0;

    public:
        mavlink_connector(std::string ip,int port);

        void slow_send();
        void fast_send();
        void recv();
    };
};

#endif /* defined(__dSim__mavlink_unreal__) */