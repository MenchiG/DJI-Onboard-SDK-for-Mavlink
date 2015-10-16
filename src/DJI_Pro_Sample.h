/*
 * DJI_Pro_Test.h
 *
 *  Created on: 7 Apr, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_SAMPLE_H_
#define DJI_PRO_SAMPLE_H_

#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Config.h"
#include "DJI_Pro_Rmu.h"

/* external functions */

int DJI_Pro_Get_Cfg(int *baud, char *dev,unsigned int *app_id,
                    unsigned int *app_api_level,char *app_key);

int DJI_Sample_Setup(void);


#endif /* DJI_PRO_TEST_H_ */
