#include "ros_cfg.h"

#include "ax12Serial.hh"
#include <iostream>

#include "unistd.h"


using namespace std;

string getAx12RegName(int reg)
{
static vector<string> ax12RegIdToName = {
"AX_MODEL_NUMBER_L", //0
"AX_MODEL_NUMBER_H", //1
"AX_VERSION", //2
"AX_ID", //3
"AX_BAUD_RATE", //4
"AX_RETURN_DELAY_TIME", //5
"AX_CW_ANGLE_LIMIT_L", //6
"AX_CW_ANGLE_LIMIT_H", //7
"AX_CCW_ANGLE_LIMIT_L", //8
"AX_CCW_ANGLE_LIMIT_H", //9
"AX_SYSTEM_DATA2", //10
"AX_LIMIT_TEMPERATURE", //11
"AX_DOWN_LIMIT_VOLTAGE", //12
"AX_UP_LIMIT_VOLTAGE", //13
"AX_MAX_TORQUE_L", //14
"AX_MAX_TORQUE_H", //15
"AX_RETURN_LEVEL", //16
"AX_ALARM_LED", //17
"AX_ALARM_SHUTDOWN", //18
"AX_OPERATING_MODE", //19
"AX_DOWN_CALIBRATION_L", //20
"AX_DOWN_CALIBRATION_H", //21
"AX_UP_CALIBRATION_L", //22
"AX_UP_CALIBRATION_H", //23
/**", //RAM AREA **/
"AX_TORQUE_ENABLE", //24
"AX_LED", //25
"AX_CW_COMPLIANCE_MARGIN", //26
"AX_CCW_COMPLIANCE_MARGIN", //27
"AX_CW_COMPLIANCE_SLOPE", //28
"AX_CCW_COMPLIANCE_SLOPE", //29
"AX_GOAL_POSITION_L", //30
"AX_GOAL_POSITION_H", //31
"AX_GOAL_SPEED_L", //32
"AX_GOAL_SPEED_H", //33
"AX_TORQUE_LIMIT_L", //34
"AX_TORQUE_LIMIT_H", //35
"AX_PRESENT_POSITION_L", //36
"AX_PRESENT_POSITION_H", //37
"AX_PRESENT_SPEED_L", //38
"AX_PRESENT_SPEED_H", //39
"AX_PRESENT_LOAD_L", //40
"AX_PRESENT_LOAD_H", //41
"AX_PRESENT_VOLTAGE", //42
"AX_PRESENT_TEMPERATURE", //43
"AX_REGISTERED_INSTRUCTION", //44
"AX_PAUSE_TIME", //45
"AX_MOVING", //46
"AX_LOCK", //47
"AX_PUNCH_L", //48
"AX_PUNCH_H" //49
};
  if (reg < ax12RegIdToName.size())
    return ax12RegIdToName[reg];
  return "unknown";
}
string getAx12RegWithName(int reg)
{
   return std::to_string(reg) + "(" + getAx12RegName(reg) + ")";
}


void setTXall() {
  cout << "setTXall" << endl;
}     // for sync write
void setTX(int id) {
  cout << "setTX " << id << endl;
}
void setRX(int id) {
  cout << "setRX " << id << endl;
}

void ax12write(unsigned char data) {
  cout << "ax12write " << getAx12RegWithName((int)data) << endl;
}
void ax12write(unsigned char *pdata, int length) {
  cout << "ax12write.2 " << endl;
}
void ax12writeB(unsigned char data) {
  cout << "ax12writeB " << endl;
}

int ax12ReadPacket(int length) { 
  cout << "ax12ReadPacket " << length << endl;
  return 0;
}

int ax12GetLastError() {
  cout << "ax12GetLastError" << endl;
  return 0;
}

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];
#if defined(AX_RX_SWITCHED)
// Need to stow type of servo (which bus it's on)
unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif




#ifndef USE_GAZEBO_SERVOS

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library


dynamixel::PortHandler *portHandler = 0;
dynamixel::PacketHandler *packetHandler = 0;

void ax12Init(long baud)
{
  if (portHandler && packetHandler) return; // don't initialise twice

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  usleep(500000);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
 // int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Terminating...\n");
    exit(1);
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Terminating...\n");
    exit(1);
  }
  usleep(500000);


    // Read all servo positions
    for (int servoId=1; servoId<=18; servoId++)
    {
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servoId, AX_PRESENT_POSITION_L, &dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Pos:%03d\n", servoId, dxl_present_position);
    }


  // Debugging mode: make servos slow and weak
  for (int servoId=1; servoId<=18; servoId++)
  {
    #define ADDR_SET_MOVING_SPEED 32
    #define ADDR_LED 25
    #define ADDR_TORQUE_LIMIT 34
    ax12SetRegister( servoId, ADDR_LED, 1 );
    ax12SetRegister( servoId, ADDR_SET_MOVING_SPEED, 200, 2 );
    ax12SetRegister( servoId, ADDR_TORQUE_LIMIT, 900, 2 );
  }
  void setAllPunch(int val);
  setAllPunch(4);
}

void setAllPunch(int val)
{
   for (int servoId=1; servoId<=18; servoId++)
  { 
    #define ADDR_PUNCH 48
    ax12SetRegister( servoId, ADDR_PUNCH, val, 2 );
  }
}


void ax12Finish()
{
  std::cout << "ax12Finish - closing ax12 port" << std::endl;

  // Close port
  portHandler->closePort();

  portHandler = 0;
  packetHandler = 0;
}


int ax12GetRegister(int servoId, int regstart, int length) { 
 cout << "ax12GetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " length=" << length << endl;

 int retries = 0;
 int val;

 for (;;) {
  if (retries == 5) { cout << "Aborting" << endl; exit(1); }
  if (retries++ > 0) { cout << "Retrying" << endl; }

  int dxl_comm_result;
  uint8_t dxl_error;

  switch (length) {
   case 1:
    {
      uint8_t val1;
      dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, servoId, regstart, &val1, &dxl_error);
      val = val1;
    }
    break;
   case 2:
    {
      uint16_t val2;
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servoId, regstart, &val2, &dxl_error);
      val = val2;
    }
    break;
   default:
    cout << "TODO: length>2" << endl;
    exit(1);
  }
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //val = (1<<(length*8))-1; // 255 or 65535
    continue; // retry
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    //val = (1<<(length*8))-1; // 255 or 65535
    continue; // retry
  }
  else
    break;
 }
 printf(" => [ID:%03d] %03d:%03d\n", servoId, regstart, val);
 return val;
}


void ax12GroupSyncWriteDetailed(uint8_t startAddr, uint8_t length, uint8_t bVals[], const uint8_t servoIds[], unsigned int NUM_SERVOS)
{
 // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, startAddr, length);
  
  // Add each servo id and value
  for (unsigned int i=0; i<NUM_SERVOS; ++i) {
    uint8_t servoId = servoIds[i];
    bool dxl_addparam_result = groupSyncWrite.addParam(servoId, &bVals[i*length]);
    if (dxl_addparam_result != true)
    { 
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", servoId);
      throw 1;
    }
  }
  
  // Syncwrite goal position
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  
  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void ax12GroupSyncWrite(uint8_t bReg, uint8_t bVal, const uint8_t cPinTable[], unsigned int NUM_SERVOS)
{
  // Initialize GroupSyncWrite instance
  int len = 1;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, bReg, len);

  // Add each servo id and value
  for (unsigned int i=0; i<NUM_SERVOS; ++i) {
    uint8_t servoId = cPinTable[i];
    bool dxl_addparam_result = groupSyncWrite.addParam(servoId, &bVal);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", servoId);
      throw 1;
    }
  }

  // Syncwrite goal position
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}


void ax12SetRegister(int servoId, int regstart, int data, int length) {
  cout << "ax12SetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " data=" << data << " length=" << length << endl;

 int retries = 0;
 int val;

 for (;;) {
  if (retries == 5) { cout << "Aborting" << endl; exit(1); }
  if (retries++ > 0) { cout << "Retrying" << endl; }

  int dxl_comm_result;
  uint8_t dxl_error;

  switch (length) {
   case 1:
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servoId, regstart, data, &dxl_error);
    }
    break;
   case 2:
    {
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, servoId, regstart, data, &dxl_error);
    }
    break;
   default:
    cout << "TODO: length>2" << endl;
    exit(1);
  }
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    continue; // retry
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    continue; // retry
  }
  else
    break;
 }
}


#else // ifndef USE_GAZEBO_SERVOS


#include "mytypes.h"
#include "Hex_Cfg.h"
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/JointState.h"

//sensor_msgs::JointState lastJointState;
vector<string> lastJointState_names;
vector<double> lastJointState_positions;
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::cout << "jointStateCallback" << std::endl;
  lastJointState_names = msg->name;
  lastJointState_positions = msg->position;

  //  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

class MyRosClass {
public:
  ros::NodeHandle n;
  //ros::Publisher chatter_pub;
  vector<ros::Publisher> joint_channels;
  ros::Publisher allJointsPosAndSpeed;
  vector<string> servoId2jointName;
  ros::Subscriber jointStateSub;
  //ros::Rate loop_rate(10);

  MyRosClass()
    : n()
    , allJointsPosAndSpeed(n.advertise<std_msgs::UInt8MultiArray>("/phantomx/allJointsPosAndSpeed", 1))
    , jointStateSub( n.subscribe("/phantomx/joint_states", 1, jointStateCallback) )
  {
    servoId2jointName.resize(18+1);
    servoId2jointName[cRRCoxaPin] = "c1_rr";
    servoId2jointName[cRRFemurPin] = "thigh_rr";
    servoId2jointName[cRRTibiaPin] = "tibia_rr";
    servoId2jointName[cRMCoxaPin] = "c1_rm";
    servoId2jointName[cRMFemurPin] = "thigh_rm";
    servoId2jointName[cRMTibiaPin] = "tibia_rm";
    servoId2jointName[cRFCoxaPin] = "c1_rf";
    servoId2jointName[cRFFemurPin] = "thigh_rf";
    servoId2jointName[cRFTibiaPin] = "tibia_rf";
    servoId2jointName[cLRCoxaPin] = "c1_lr";
    servoId2jointName[cLRFemurPin] = "thigh_lr";
    servoId2jointName[cLRTibiaPin] = "tibia_lr";
    servoId2jointName[cLMCoxaPin] = "c1_lm";
    servoId2jointName[cLMFemurPin] = "thigh_lm";
    servoId2jointName[cLMTibiaPin] = "tibia_lm";
    servoId2jointName[cLFCoxaPin] = "c1_lf";
    servoId2jointName[cLFFemurPin] = "thigh_lf";
    servoId2jointName[cLFTibiaPin] = "tibia_lf";

    joint_channels.resize(1); // adding empty space for unused servo 0
    for (int servoId=1; servoId<=18; ++servoId) {
      string jointName = "/phantomx/j_" + servoId2jointName[servoId] + "_position_controller/command";
      joint_channels.push_back( n.advertise<std_msgs::Float64>(jointName, 1) );
    }
  }

} *myRos = NULL;

void ax12Init(long baud) {
  cout << "ax12Init" << endl;

  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "Hexapod");

  myRos = new MyRosClass();

}
void ax12Finish() {
  std::cout << "ax12Finish - closing ax12 port" << std::endl;
}


int ax12GetRegister(int servoId, int regstart, int length) { 
 cout << "ax12GetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " length=" << length << endl;

 if (regstart==AX_PRESENT_POSITION_L && length==2) {
   if (!myRos || lastJointState_positions.empty()) {
     return 512;
   }
   string jointName = "j_" + myRos->servoId2jointName[servoId];
   static vector<int> servoId2PositionInJointStates(19);
   int index = servoId2PositionInJointStates[servoId];
   if (lastJointState_names[index] != jointName) {
     // index is not correct, try to set it up
     cout << "This message should only appear at the beginning: Setting up servoId2PositionInJointStates for " << jointName << endl;
     for (index=0; index<lastJointState_names.size(); ++index) {
       if (lastJointState_names[index] == jointName) {
	 servoId2PositionInJointStates[servoId] = index;
	 break;
       }
     }
     if (index==lastJointState_names.size()) {
       cout << "Error: joint name not found" << endl;
       exit(1);
     }
   }

   // Convert pos from gazebo units (radians) to ax12 units (0-1023 for -150deg to +150deg)
   double posRad = lastJointState_positions[index];
   const double PI = 3.14159265359;
   double posDeg = std::fmod(posRad/PI*180.0,180);
   if (posDeg < -150 || posDeg > 150) {
     cout << "ERROR: servo position out of range" << endl;
     posDeg /= 0;
   }
   double pos = posDeg/150 + 1;
   int posInt = std::nearbyint(pos * 512);
   if (posInt < 0) posInt=0;
   if (posInt > 1023) posInt=1023;
   return posInt;
 }
return 0;
}
void ax12SetRegister(int servoId, int regstart, int data, int length) {
  cout << "ax12SetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " data=" << data << " length=" << length << endl;
}

void ax12GroupSyncWrite(uint8_t bReg, uint8_t bVal, const uint8_t cPinTable[], unsigned int NUM_SERVOS) {
  cout << "ax12GroupSyncWrite" << endl;
}


void ax12GroupSyncWriteDetailed(uint8_t startAddr, uint8_t length, uint8_t bVals[], const uint8_t servoIds[], unsigned int NUM_SERVOS) {
  cout << "ax12GroupSyncWriteDetailed" << endl;

  if (ros::ok()) {
    if (startAddr==AX_GOAL_POSITION_L && length==2) {
      std_msgs::Float64 msg2;
      for (unsigned int i=0; i<NUM_SERVOS; ++i) {
	uint8_t servoId = servoIds[i];

	int posInt = bVals[2*i] + ( bVals[2*i+1] << 8 );
	// Convert pos from ax12 units (0-1023 for -150deg to +150deg) to gazebo units (radians)
	const double PI = 3.14159265359;
	double posRad = (posInt-512)*(PI*150.0/180.0/512.0);

	msg2.data = posRad;
	myRos->joint_channels[servoId].publish(msg2);
      }
      ros::spinOnce();

      //loop_rate.sleep();
    }

    else if (startAddr==AX_GOAL_POSITION_L && length==4) {
      std_msgs::UInt8MultiArray msg;

      // set up dimensions
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim[0].size = NUM_SERVOS*5;//vec1.size();
      msg.layout.dim[0].stride = 1;
      msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

      // copy in the data
      //msg.data.clear();
      //msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
      for (unsigned int i=0; i<NUM_SERVOS; ++i) {
        msg.data.push_back(servoIds[i]);
      }
      for (unsigned int i=0; i<NUM_SERVOS; ++i) {
        msg.data.push_back(bVals[4*i+0]);
        msg.data.push_back(bVals[4*i+1]);
        msg.data.push_back(bVals[4*i+2]);
        msg.data.push_back(bVals[4*i+3]);
      }
/*
      for (unsigned int i=0; i<NUM_SERVOS; ++i) {
        uint8_t servoId = servoIds[i];

        int posInt = bVals[4*i] + ( bVals[4*i+1] << 8 );
        int speedInt = bVals[4*i+2] + ( bVals[4*i+3] << 8 );
        // Convert pos from ax12 units (0-1023 for -150deg to +150deg) to gazebo units (radians)
        const double PI = 3.14159265359;
        double posRad = (posInt-512)*(PI*150.0/180.0/512.0);

        msg2.data = posRad;
        myRos->joint_channels[servoId].publish(msg2);
      }
*/
      myRos->allJointsPosAndSpeed.publish(msg);
      ros::spinOnce();

      //loop_rate.sleep();
    }
  }
}


#endif // ifndef USE_GAZEBO_SERVOS
