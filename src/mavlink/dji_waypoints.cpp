//
// Created by Hao Xu on 15/6/8.
//

#include "dji_waypoints.h"
#include "dji_variable.h"
#include "motion_controls.h"
#include "DJI_Pro_App.h"
#include <fstream>

using namespace std;


dji_waypoints::dji_waypoints(std::string path)
{
    init_state_machine();
    this->load(path);
}

dji_waypoints::dji_waypoints()
{
    init_state_machine();
    mission mission1;
    this->missions.push_back(mission1);
}
int dji_waypoints::load(std::string path)
{
    std::ifstream wp_file(path);
    mission mission1;
    std::cout << "loading.....wp at " << path<<"\n";
    wp_file >> mission1;

    if (this->missions.size() == 1 && this->missions[0].size() == 0)
    {
        this->missions[0] = mission1;
    }
    else {
        this->missions.push_back(mission1);
    }
    std::cout << mission1;
    return mission1.size();
}

int dji_waypoints::add_waypoints(mission mission1)
{
    missions.push_back(mission1);
    return (int)(missions.size()-1);

}

int dji_waypoints::begin_fly_waypoints(int id)
{
    int status = switch_status(this->status,ACTION_START_MISSION);
    if (status == 0 || id >= missions.size() || id < 0)
    {
        return 0;
    }

    mission_id = id;
    waypoint_ptr = 0;
    this->status = status;
    return 1;
}


int dji_waypoints::begin_fly_waypoints(int id,int ptr)
{
     int status = switch_status(this->status,ACTION_START_MISSION);
    if (status == 0 || id >= missions.size() || id < 0)
    {
        return 0;
    }

    mission_id = id;
    waypoint_ptr = ptr;
    this->status = status;
    printf("will begin at %d %d\n",id,ptr);
    return 1;
}

int dji_waypoints::pause_flying()
{
    int status = switch_status(this->status,ACTION_PAUSE_MISSION);
    if (status == 0)
    {
        return 0;
    }
    this -> status = status;
    return 1;
}

int dji_waypoints::cont_flying()
{

    int status = switch_status(this->status,ACTION_CONT_MISSION);
    if (status == 0)
    {
        return 0;
    }
    this -> status = status;
    return 1;
}

int dji_waypoints::loop()
{
    switch (status)
    {
        case STATUS_STANDBY:
        case STATUS_PAUSE :
            break;
        case STATUS_FLYWAYPOINT:
            continue_mission();
            break;
        default:
            break;
    }
    return 1;
}

int dji_waypoints::continue_mission()
{
//    printf("continue mission...\d");
    auto wp = missions[mission_id][waypoint_ptr];
    if (approach(wp , dji_variable::global_position ,wp.uncertain))
    {
        if (wp.cmd == 21) //land
        {
            DJI_Pro_Status_Ctrl(6,NULL);
        }
        else if(wp.cmd == 20) //RTH
        {
            DJI_Pro_Status_Ctrl(1,NULL);
        }
        waypoint_ptr ++;
        if (waypoint_ptr >= missions[mission_id].size())
        {
            status = switch_status(status,ACTION_STOP_MISSION);
            return 1;
        }
    }
    else
    {
        if(wp.cmd == 22 && dji_variable::flight_status != 3)
            DJI_Pro_Status_Ctrl(4,NULL);
        motion_controls::fly_to_globalpos(wp,false);
        
    }
    return 0;
}

bool dji_waypoints::approach(api_pos_custom_data_t wp1, api_pos_custom_data_t wp2,float uncertain)
{
    api_common_data_t local_position1,local_position2; 
    local_position1 = dji_variable::gps_convert_ned(wp1);
    local_position2 = dji_variable::gps_convert_ned(wp2);
    local_position1.z = wp1.alti;
    local_position2.z = wp2.alti;
    float raduis =
            (local_position1.x - local_position2.x) *  (local_position1.x - local_position2.x) +
            (local_position1.y - local_position2.y) *  (local_position1.y - local_position2.y) +
            (local_position1.z - local_position2.z) *  (local_position1.z - local_position2.z);

    if ( raduis > uncertain * uncertain )
    {
        return false;
    }
    return true;
}

int dji_waypoints::switch_status(int status, int action)
{
    return state_machine[status][action];
}

void dji_waypoints::init_state_machine()
{
    state_machine[STATUS_STANDBY][ACTION_START_MISSION] = STATUS_FLYWAYPOINT;
    state_machine[STATUS_STANDBY][ACTION_STOP_MISSION] = STATUS_STANDBY;
    state_machine[STATUS_STANDBY][ACTION_PAUSE_MISSION] = STATUS_STANDBY;

    state_machine[STATUS_PAUSE][ACTION_START_MISSION] = STATUS_FLYWAYPOINT;
    state_machine[STATUS_PAUSE][ACTION_CONT_MISSION] = STATUS_FLYWAYPOINT;
    state_machine[STATUS_PAUSE][ACTION_STOP_MISSION] = STATUS_STANDBY;
    state_machine[STATUS_PAUSE][ACTION_PAUSE_MISSION] = STATUS_PAUSE;

    state_machine[STATUS_FLYWAYPOINT][ACTION_START_MISSION] = STATUS_FLYWAYPOINT;
    state_machine[STATUS_FLYWAYPOINT][ACTION_PAUSE_MISSION] = STATUS_PAUSE;
    state_machine[STATUS_FLYWAYPOINT][ACTION_CONT_MISSION] = STATUS_FLYWAYPOINT;
    state_machine[STATUS_FLYWAYPOINT][ACTION_STOP_MISSION] = STATUS_STANDBY;
}