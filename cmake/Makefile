TARGET = ../output/DJI_Onboard_API_Cmdline_Test

OBJECTS = main.o DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Sample.o DJI_Pro_Codec.o DJI_Pro_Rmu.o tinyxml2.o dji_commands.o dji_gimbal.o dji_mavlink_adapter.o dji_variable.o dji_waypoints.o mavlink_connector.o motion_controls.o

CFLAGS = -Wall -O3 -DPLATFORM_LINUX -std=c++11 -I..src/ -I../src/DJI_LIB -I../include/dji_sdk -I../include/dji_mavlink/dji_sdk_onboard

LDFLAGS = -lpthread

CXX = g++-4.8

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
main.o : ../src/main.cpp
	$(CXX) $(CFLAGS) -c ../src/main.cpp

tinyxml2.o : ../src/tinyxml2.cpp
	$(CXX) $(CFLAGS) -c ../src/tinyxml2.cpp

DJI_Pro_App.o : ../src/DJI_LIB/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../src/DJI_LIB/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../src/DJI_LIB/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Link.cpp

DJI_Pro_Sample.o : ../src/DJI_Pro_Sample.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Sample.cpp
	
DJI_Pro_Codec.o : ../src/DJI_LIB/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Codec.cpp

DJI_Pro_Rmu.o: ../src/DJI_LIB/DJI_Pro_Rmu.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Rmu.cpp

dji_commands.o : ../src/mavlink/dji_commands.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/dji_commands.cpp

dji_gimbal.o : ../src/mavlink/dji_gimbal.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/dji_gimbal.cpp

dji_mavlink_adapter.o : ../src/mavlink/dji_mavlink_adapter.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/dji_mavlink_adapter.cpp

dji_variable.o : ../src/mavlink/dji_variable.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/dji_variable.cpp

dji_waypoints.o : ../src/mavlink/dji_waypoints.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/dji_waypoints.cpp

mavlink_connector.o : ../src/mavlink/mavlink_connector.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/mavlink_connector.cpp

motion_controls.o : ../src/mavlink/motion_controls.cpp
	$(CXX) $(CFLAGS) -c ../src/mavlink/motion_controls.cpp



	
clean:
	-rm *.o
	-rm $(TARGET)
