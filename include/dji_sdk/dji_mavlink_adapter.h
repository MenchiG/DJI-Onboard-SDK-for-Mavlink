#include "mavlink.h"
#include <string>

namespace mavlink_adapter
{
    extern int fd;
    void loop_callback(long timestamp);
    void set_mavlink(std::string _tty,int _port);
    void recv_function();
}
