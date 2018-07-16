LIBS=-L ../DynamixelSDK/c++/build/linux_sbc -ldxl_sbc_cpp
LIBS=-Wl,-rpath,/opt/ros/melodic/lib -L /opt/ros/melodic/lib -l roscpp -l rosconsole -l roscpp_serialization -l rostime

INCLUDES=-I ../DynamixelSDK/c++/include/dynamixel_sdk
INCLUDES=-I /opt/ros/melodic/include/

go: main.o BioloidEX.o ax12Serial.o
	g++ -std=c++14 $^ -o go -O0 -g $(LIBS) -lrt 2>&1

%.o: %.cpp
	g++ -std=c++14 $^ -I . $(INCLUDES) -c -O0 -g 2>&1

clean:
	rm *.o
