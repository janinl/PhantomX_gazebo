#include "ax12Serial.cpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>

#include "../../Phoenix_walk/mytypes.h"
#include "../../Phoenix_walk/Hex_Cfg.h"

vector<ros::Publisher> servo_status_channels;
ros::Publisher joint_states_pub;
bool emergencyStopActive = false;

/*
void callback(const std_msgs::Float64::ConstPtr& msg, int servoId, bool isReverse)
{
    ROS_INFO("I heard: servoId=%d [%f]", servoId, msg->data);

    // Convert pos from gazebo units (radians) to ax12 units (0-1023 for -150deg to +150deg)
    double posRad = msg->data;
    const double PI = 3.14159265359;
    double posDeg = std::fmod(posRad/PI*180.0,180);
    if (posDeg < -150 || posDeg > 150) {
        cout << "ERROR: servo position out of range" << endl;
        posDeg /= 0;
    }
    if (isReverse) posDeg = -posDeg;

    double pos = posDeg/150 + 1;
    int posInt = std::nearbyint(pos * 512);
    if (posInt < 0) posInt=0;
    if (posInt > 1023) posInt=1023;
    ROS_INFO(" => %d", posInt);

    ax12SetRegister(servoId, AX_GOAL_POSITION_L, posInt, 2);
}


void callback_allJointsPosAndSpeed(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    ROS_INFO("I heard: callback_allJointsPosAndSpeed");//servoId=%d [%f]", servoId, msg->data);

    int num_servos = msg->layout.dim[0].size / 5;
    const uint8_t *servoIds = &msg->data[0];
    const uint8_t *bVals = &msg->data[num_servos];

    // convert const uint8[] to non-const uint8[]
    uint8_t bVals2[100];
    if (num_servos*4 > 100) {
        cout << "ERROR: static array too small" << endl;
        exit(1);
    }
    for (int i=0; i<num_servos*4; ++i)
        bVals2[i] = bVals[i];

    ax12GroupSyncWriteDetailed(AX_GOAL_POSITION_L, 4, bVals2, servoIds, num_servos);
}
*/

const double PI = 3.14159265359;
double convertBvalsToRad(uint8_t *bVals)
{
    int posInt = bVals[0] + ( bVals[1] << 8 );
    // Convert pos from ax12 units (0-1023 for -150deg to +150deg) to radians
    double posRad = (posInt-512)*(PI*150.0/180.0/512.0);
    return posRad;
}
double convertBvalsToRadPerSec(uint8_t *bVals)
{
    int wSpeed = bVals[0] + ( (bVals[1] & 3) << 8 );
    // Convert pos from ax12 units (0-1023 for -150deg to +150deg) to radians
    double radiansPerSec = wSpeed * ((114 * 2 * PI) / (1023 * 60));
    return radiansPerSec;
}

void convertPosAndSpeedTo4bytes(double posInRad, double radiansPerSec, uint8_t *valArray, double lastPosInRad)
{
    long g_awGoalAXPos = 512 + ( posInRad / (PI*150.0/180.0/512.0) );
    if (g_awGoalAXPos<0) g_awGoalAXPos=0;
    if (g_awGoalAXPos>1023) g_awGoalAXPos=1023;
    long wSpeed = radiansPerSec * 1023 * 60 / (114 * 2 * PI); // 0x3ff=114rpm - same value for AX12 and AX18 //50 * abs(posInRad - lastPosInRad);
    if (wSpeed>500) wSpeed = 500; // todo: increase to 1023
    if (wSpeed<1) wSpeed = 1;
    valArray[0] = g_awGoalAXPos & 0xff;
    valArray[1] = ( g_awGoalAXPos >> 8 ) & 0xff;
    valArray[2] = wSpeed & 0xff;
    valArray[3] = wSpeed >> 8;
}

vector<string> jointNames = {
    "j_c1_lf", "j_c1_lm", "j_c1_lr", "j_c1_rf", "j_c1_rm", "j_c1_rr",
    "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr",
    "j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr"
};

double lastPositions[8] = {0,0,0,0,0,0,0,0};

void callback_jointGoals(const control_msgs::FollowJointTrajectoryGoal::ConstPtr& msg)
{
    std::cout << "callback_jointGoals" << std::endl;
    if (emergencyStopActive) {
        std::cout << "emergency stop active" << std::endl;
        return;
    }
    std::cout << "msg=" << *msg << std::endl;
    assert(jointNames.size() == msg->trajectory.joint_names.size());
    assert(msg->trajectory.points.size() == 1);
    assert(msg->trajectory.points[0].positions.size() == 18);
    assert(msg->trajectory.points[0].velocities.size() == 18);

    // convert to uint8[]
    const uint8_t servoIds[18] = {
        cLFCoxaPin, cLMCoxaPin, cLRCoxaPin, cRFCoxaPin, cRMCoxaPin, cRRCoxaPin,
        cLFFemurPin, cLMFemurPin, cLRFemurPin, cRFFemurPin, cRMFemurPin, cRRFemurPin,
        cLFTibiaPin, cLMTibiaPin, cLRTibiaPin, cRFTibiaPin, cRMTibiaPin, cRRTibiaPin
    };
    uint8_t bVals2[18*4];
    int num_servos = 18;
    for (int i = 0; i < 18; ++i)
    {
        assert(jointNames[i] == msg->trajectory.joint_names[i]);
        convertPosAndSpeedTo4bytes(msg->trajectory.points[0].positions[i], msg->trajectory.points[0].velocities[i], &bVals2[4*i], lastPositions[i]);
        lastPositions[i] = msg->trajectory.points[0].positions[i];
    }

    ax12GroupSyncWriteDetailed(AX_GOAL_POSITION_L, 4, bVals2, servoIds, num_servos);
}
/*
void getAndPublishNextOf18ServosData()
{
    static int currentServo = 1;

    uint8_t regstart = AX_PRESENT_POSITION_L;
    uint8_t length = 8;
    uint8_t outVal[8];
    uint32_t err;
    ax12GetRegister(currentServo, regstart, length, &err, outVal);

    std_msgs::Int16MultiArray msg;

    // set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 8; // pos, speed, load, voltage, temp, error
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "pos,vel,load,volt,temp,err";

    // copy in the data
    //msg.data.clear();
    int16_t pos = (outVal[1]<<8) | outVal[0];
    int16_t vel = (outVal[3]<<8) | outVal[2];
    if (vel>1023) vel=1024-vel;
    int16_t load = (outVal[5]<<8) | outVal[4];
    if (load>1023) load=1024-load;
    msg.data.push_back( pos );
    msg.data.push_back( vel );
    msg.data.push_back( load );
    msg.data.push_back( outVal[6] );
    msg.data.push_back( outVal[7] );
    msg.data.push_back( err );

    servo_status_channels[currentServo].publish(msg);
//    ax12Get18ServosData();

    // next servo
    currentServo = currentServo % 18 + 1;
}
*/

void getAndPublishAll18ServosData()
{
    uint8_t outData[18*8];
    int dxl_comm_results[18];
    uint8_t dxl_errors[18];
    int numberOfRetries[18];
    int totalNumberOfRetries;
    int servoIdIfError;

    ax12Get18ServosData(outData, dxl_comm_results, dxl_errors, numberOfRetries, totalNumberOfRetries, servoIdIfError);
    if (servoIdIfError>0) {
        // Immediately stop as many servos as possible
        cerr << "Big error detected: Could not read from servo " << servoIdIfError << ". Stopping all servos." << endl;
        emergencyStopActive = true;
        ax12EmergencyStop18Servos();
        cerr << "Big error detected: Could not read from servo " << servoIdIfError << ". Tried to stop all servos." << endl;
        cerr << "dxl_comm_results[servoIdIfError]=" << dxl_comm_results[servoIdIfError-1] << endl;
        cerr << "dxl_errors[servoIdIfError]=" << (unsigned int)dxl_errors[servoIdIfError-1] << endl;
    }
    if (totalNumberOfRetries>0) {
        // Temporary code, to check if this ever happens
        cerr << "Retries detected: " << totalNumberOfRetries << " total retries. Stopping all servos." << endl;
        emergencyStopActive = true;
        ax12EmergencyStop18Servos();
        for (int i=0; i<18; ++i) {
            if (numberOfRetries[i]>0) {
                cerr << "numberOfRetries[" << i << "]=" << numberOfRetries[i] << endl;
                cerr << "dxl_comm_results[" << i << "]=" << getDxlCommResultsErrorString(dxl_comm_results[i]) << endl;
                cerr << "dxl_errors[" << i << "]=" << getDxlErrorsErrorString(dxl_errors[i]) << endl;
            }
        }
    }

    // Publish joint_states
    if (!emergencyStopActive)
    {
        sensor_msgs::JointState joint_states;

        joint_states.name = jointNames;
        for (int i=0; i<18; ++i) {
            //double pos  = outData[8*i  ] + (outData[8*i+1] << 8);
            double pos = convertBvalsToRad(&outData[8*i]);
            //double vel  = outData[8*i+2] + ((outData[8*i+3] & 3) << 8);
            double vel = convertBvalsToRadPerSec(&outData[8*i+2]);
            //double load = outData[8*i+4] + ((outData[8*i+3] & 6) << 8);
            double load = convertBvalsToRadPerSec(&outData[8*i+4]);
            joint_states.position.push_back(pos);
            joint_states.velocity.push_back(vel);
            joint_states.effort.push_back(load);
        }
        joint_states_pub.publish(joint_states);
    }
}

/*
long unsigned int millis() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long unsigned int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    return ms;
}
*/

bool CtrlCPressed = false;
void signalHandler(int sig)
{
    cout << "Ctrl+C detected" << endl;
    CtrlCPressed = true;
    sleep(1);
}

int main(int argc, char **argv)
{
    ax12Init(1000000);

    signal(SIGINT, signalHandler);

//#define SPEED_TEST
#ifdef SPEED_TEST
    unsigned long lastTime = millis();
    int count = 0;
    while (1) {
        ++count;
        unsigned long currentTime = millis();
        cout << currentTime << endl;
        /*
            for (int servoId=1; servoId<=1; servoId++)
            {
              ax12GetRegister(servoId,AX_PRESENT_POSITION_L,8);
            }
        */
        ax12Get18ServosData();

        if (currentTime >= lastTime+10000) {
            cout << " *** Times per second: " << count*0.1 << endl;
            sleep(1);
            count = 0;
            lastTime = millis(); //currentTime;
        }
    }
#endif

    // Reset servo positions
    int betterServoOrder[18] = {3,4,9,10,15,16, 5,6,11,12,17,18, 1,2,7,8,13,14};
    for (int i=0; i<18; ++i) {
        int servoId = betterServoOrder[i];
        ax12SetRegister(servoId,AX_GOAL_POSITION_L,512,2);
        if (i==5 || i==11)
            sleep(1);
    }


    ros::init(argc, argv, "ax12_servos", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    vector<string> servoId2jointName;
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
    /*
        vector<ros::Subscriber> joint_channels;// = n.subscribe("/hexapd/hj_.../", 10, callback);
        joint_channels.resize(1); // adding empty space for unused servo 0
        servo_status_channels.resize(1); // adding empty space for unused servo 0
        for (int servoId=1; servoId<=18; ++servoId) {
          string jointName = "/phantomx/j_" + servoId2jointName[servoId] + "_position_controller/command";
          bool isReverse = false; //servoId2jointName[servoId].substr(0,5) == "tibia" || servoId2jointName[servoId].substr(0,2) == "c1";
          joint_channels.push_back( n.subscribe<std_msgs::Float64>(jointName, 10, boost::bind(&callback, _1, servoId, isReverse)) );

          string statusChanName = "/servo/" + to_string(servoId) + "/status";
          servo_status_channels.push_back( n.advertise<std_msgs::Int16MultiArray>(statusChanName, 2) );
        }
    */
    //ros::Subscriber allJointsPosAndSpeed(n.subscribe<std_msgs::UInt8MultiArray>("/phantomx/allJointsPosAndSpeed", 10, callback_allJointsPosAndSpeed));
    joint_states_pub = n.advertise<sensor_msgs::JointState>("/phantomx/joint_states", 1);
    ros::Subscriber jointGoalsSub(n.subscribe("/webbie1/joint_goals", 1, callback_jointGoals));

    while (ros::ok() && !CtrlCPressed) {
        ros::spinOnce();
        //getAndPublishNextOf18ServosData();
        if (!emergencyStopActive)
            getAndPublishAll18ServosData();
    }

    ax12Finish();
    ros::shutdown();
    ros::waitForShutdown();
    _exit(0); // Skip atexit stuff to hide a segfault :( maybe related to g++7 stdlib warnings...
    return 0;
}



