/*
 * DJI_Pro_Sample.cpp
 *
 *  Created on: Aug 24, 2015
 *  Author: wuyuwei
 */

#include "DJI_Pro_Sample.h"

#include <stdio.h>
#include "tinyxml2.h"
using namespace tinyxml2;
using namespace std;


int DJI_Pro_Get_Cfg(int *baud, char *dev,unsigned int *app_id, unsigned int *app_api_level,char *app_key)
{
	XMLDocument xml_file;
	const XMLAttribute *xml_attr;
	xml_file.LoadFile("config.xml");
	if(xml_file.ErrorID())
	{
		printf("%s:%d:Load user config.xml file ERROR,using default user setting\n",__func__,__LINE__);
		return -1;
	}
	xml_attr = xml_file.RootElement()->FirstChildElement("UART")->FirstAttribute();
    if(baud)
        *baud = atoi(xml_attr->Value());
    if(dev)
        strcpy(dev,xml_attr->Next()->Value());

	xml_attr = xml_file.RootElement()->FirstChildElement("Account")->FirstAttribute();
    if(app_id)
        *app_id = atoi(xml_attr->Value());
    if(app_api_level)
        *app_api_level = atoi(xml_attr->Next()->Value());
    if(app_key)
        strcpy(app_key,xml_attr->Next()->Next()->Value());
	return 0;
}


int DJI_Sample_Setup(void)
{
	int ret;
    int baudrate = 115200;
    char uart_name[32] = {"/dev/ttyUSB0"};

    if(DJI_Pro_Get_Cfg(&baudrate,uart_name,NULL,NULL,NULL) == 0)
	{
		/* user setting */
		printf("\n--------------------------\n");
		printf("uart_baud=%d\n",baudrate);
		printf("uart_name=%s\n",uart_name);
		printf("--------------------------\n");
	}
    ret = Pro_Hw_Setup(uart_name,baudrate);
	if(ret < 0)
		return ret;
    DJI_Pro_Setup(NULL);
	return 0;
}
