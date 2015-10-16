#ifndef __DJI_SDK_ACT_OPEN_H__
#define __DJI_SDK_ACT_OPEN_H__
#include <sdk_lib/DJI_Pro_Link.h>
#include <sdk_lib/DJI_Pro_App.h>

namespace pre_act
{
	void activation();
	void activation_ack_cmd_callback(ProHeader *header);
	void ros_nav_open_close();
	void nav_open_close_callback(ProHeader *header);
}

#endif //DJI_SDK_ACT_OPEN_H