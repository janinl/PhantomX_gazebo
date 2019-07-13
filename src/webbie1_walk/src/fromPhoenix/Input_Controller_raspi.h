/*
class CommanderInputController : 
public InputController
{
public:
  CommanderInputController();        // A reall simple constructor...

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);

};
*/

void CommanderTurnRobotOff(void);


InputController g_InputController;

bool g_fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

static int last_position=0;
static ifstream *infile;

string find_new_text() {

  if (!infile) {
    infile = new ifstream("input.txt");
    if (!infile) return "";
    if (!infile->good()) { delete infile; infile=0; return ""; }
    infile->seekg(0,ios::end);
    last_position = infile->tellg();
  }
  infile->clear();

    infile->seekg(0,ios::end);
    cout << "file size: " << infile->tellg() << endl;

  // read file from last position
  infile->seekg( last_position,ios::beg);
  string val;
  if (getline( *infile, val)) {
    last_position = infile->tellg();
    return val;
  }
  return "";

}


// Commander: class to match the controller
/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif


/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander
{    
public:
  Commander() {} 
  //void begin(unsigned long baud);
  //int ReadMsgs();         // must be called regularly to clean out Serial buffer

  // joystick values are -125 to 125
  signed char rightV;      // vertical stick movement = forward speed
  signed char rightH;      // horizontal stick movement = sideways or angular speed
  signed char leftV;      // vertical stick movement = tilt    
  signed char leftH;      // horizontal stick movement = pan (when we run out of pan, turn body?)

  // buttons are 0 or 1 (PRESSED), and bitmapped
  unsigned char buttons;  // 
  unsigned char ext;      // Extended function set

    // Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET

private:
  // internal variables used for reading messages
  unsigned char vals[7];  // temporary values, moved after we confirm checksum
  int index;              // -1 = waiting for new packet
  int checksum;
};

Commander command = Commander();
static short   g_BodyYOffset = 100; 
static short   g_BodyYShift = 0;
static byte    ControlMode = 0;
static byte    HeightSpeedMode = 0;
//static bool  DoubleHeightOn = 0;
static bool    DoubleTravelOn = true;
static byte    bJoystickWalkMode = 0;
byte           GPSeq = 0;             //Number of the sequence
static byte    buttonsPrev = 0;
static byte    extPrev = 0;

enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, 
#ifdef OPT_SINGLELEG      
  SINGLELEGMODE, 
#endif
#ifdef OPT_GPPLAYER
  GPPLAYERMODE, 
#endif
  MODECNT};
enum {
  NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};

#define cTravelDeadZone 6      //The deadzone for the analog input from the remote


//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
/*
InputController::InputController()
{
}
*/

void InputController::Init()
{
}

void InputController::AllowControllerInterrupts(boolean fAllow)
{
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
string commandFromRosTopic;
void InputController::ControlInput(void)
{
  static bool activated = false;
  string cmd = find_new_text();
  
  if (cmd == "" && commandFromRosTopic != "") {
    cmd = commandFromRosTopic;
    commandFromRosTopic = "";
  }

//CommanderTurnRobotOff();
  // See if we have a new command available...
  if(cmd != ""){
    // Parse cmd to command
    command.buttons = 0;
    command.buttons |= (cmd.find("BUT_R1") != string::npos)?BUT_R1:0;
    command.buttons |= (cmd.find("BUT_R2") != string::npos)?BUT_R2:0;
    command.buttons |= (cmd.find("BUT_R3") != string::npos)?BUT_R3:0;
    command.buttons |= (cmd.find("BUT_L4") != string::npos)?BUT_L4:0;
    command.buttons |= (cmd.find("BUT_L5") != string::npos)?BUT_L5:0;
    command.buttons |= (cmd.find("BUT_L6") != string::npos)?BUT_L6:0;
    int leftVPos = cmd.find("leftV=");
    int leftHPos = cmd.find("leftH=");
    int rightVPos = cmd.find("rightV=");
    int rightHPos = cmd.find("rightH=");
    command.leftV = (leftVPos != string::npos)?stoi(cmd.substr(leftVPos+6)):0;
    command.leftH = (leftHPos != string::npos)?stoi(cmd.substr(leftHPos+6)):0;
    command.rightV = (rightVPos != string::npos)?stoi(cmd.substr(rightVPos+7)):0;
    command.rightH = (rightHPos != string::npos)?stoi(cmd.substr(rightHPos+7)):0;

    // My own direct commands
    if (cmd == "REST") {
      uint8_t valArray[36];
      uint8_t servoIds[18] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
      int poseSize = 18;
      for (int i=0; i<18; ++i) {
        unsigned int val = 512;
        valArray[2*i] = val & 0xff;
        valArray[2*i+1] = val >> 8;
      }
      ax12GroupSyncWriteDetailed(AX_GOAL_POSITION_L, 2, valArray, servoIds, poseSize);
CommanderTurnRobotOff();
activated=false;
 
      return;
    }
    if (cmd == "LEG1UP") {
      uint8_t valArray[36];
      uint8_t servoIds[18] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
      int poseSize = 18;
      for (int i=0; i<18; ++i) {
        unsigned int val = 512;
        if (i==16-1) val = 256;
        valArray[2*i] = val & 0xff;
        valArray[2*i+1] = val >> 8;
      }
      ax12GroupSyncWriteDetailed(AX_GOAL_POSITION_L, 2, valArray, servoIds, poseSize);
 
CommanderTurnRobotOff();
activated=false;
      return;
    }

    if (cmd.substr(0,6) == "SPEED=") {
      g_InControlState.SpeedControl = stoi(cmd.substr(6));
      cout << "Setting speed to " << g_InControlState.SpeedControl << endl;
      return;
    }

    if (cmd.substr(0,7) == "HEIGHT=") {
      g_BodyYOffset = stoi(cmd.substr(7)); // default is 35, good is 100
      cout << "Setting height to " << g_BodyYOffset << endl;
    }

    if (cmd.substr(0,11) == "LEG_HEIGHT=") {
      g_InControlState.LegLiftHeight = stoi(cmd.substr(11)); // default are 50 and 80, good is 80-100
      cout << "Setting leg height to " << g_InControlState.LegLiftHeight << endl;
    }


    // We received another command than our owns => activate walking
    activated = true;
//  }
  if (activated) {
    // If we receive a valid message than turn robot on...
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;

    if (!g_InControlState.fRobotOn ) {
      g_InControlState.fRobotOn = true;
      fAdjustLegPositions = true;
    }

    // [SWITCH MODES]

    // Cycle through modes...
    if ((command.buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000); 
      } 
      else {
        MSound( 1, 50, 2000);  
      }
#ifdef OPT_SINGLELEG      
      if (ControlMode != SINGLELEGMODE)
        g_InControlState.SelectedLeg=255;
      else {
        g_InControlState.SelectedLeg = 0;   // Select leg 0 when we go into this mode. 
        g_InControlState.PrevSelectedLeg = 255;
#ifdef DEBUG_SINGLELEG
        Serial.println("Single Leg Mode");  
#endif
      }
#endif
    }

    //[Common functions]
    //Switch Balance mode on/off 
    if ((command.buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound( 1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if ((command.buttons & BUT_L5) && !(buttonsPrev & BUT_L5)) {
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 100; // LJ:was 35
      fAdjustLegPositions = true;
      g_fDynamicLegXZLength = false;
    }

    // We will use L6 with the Right joystick to control both body offset as well as Speed...
    // We move each pass through this by a percentage of how far we are from center in each direction
    // We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
    // which give an idea, but only for those whoes robot has a speaker
    int lx = command.leftH;
    int ly = command.leftV;

    if (command.buttons & BUT_L6 ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      int delta = command.rightV/25;   
      if (delta) {
        g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
        fAdjustLegPositions = true;
      }

      // Also use right Horizontal to manually adjust the initial leg positions.
      sLegInitXZAdjust = lx/10;        // play with this.
      sLegInitAngleAdjust = ly/8;
      lx = 0;
      ly = 0;

      // Likewise for Speed control
      delta = command.rightH / 16;   // 
      if ((delta < 0) && g_InControlState.SpeedControl) {
        if ((word)(-delta) <  g_InControlState.SpeedControl)
          g_InControlState.SpeedControl += delta;
        else 
          g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);  
      }
      if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += delta;
        if (g_InControlState.SpeedControl > 2000)
          g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl); 
      }

      command.rightH = 0; // don't walk when adjusting the speed here...
    }

#ifdef DBGSerial
    if ((command.buttons & BUT_R3) && !(buttonsPrev & BUT_R3)) {
      MSound(1, 50, 2000);
      g_fDebugOutput = !g_fDebugOutput;
    }
#endif    
    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1))
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000);  
        } 
        else {
          MSound (2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound( 1, 50, 2000);  
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

      // Switch between Walk method 1 && Walk method 2
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
#ifdef cTurretRotPin
        if ((++bJoystickWalkMode) > 2)
#else
          if ((++bJoystickWalkMode) > 1)
#endif 
            bJoystickWalkMode = 0;
        MSound (1, 50, 2000 + bJoystickWalkMode*250);
      }

      //Walking
      switch (bJoystickWalkMode) {
      case 0:    
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
      case 1:
        g_InControlState.TravelLength.z = (command.rightV); //Right Stick Up/Down  
        g_InControlState.TravelLength.y = -(command.rightH)/4; //Right Stick Left/Right 
        break;
#ifdef cTurretRotPin
      case 2:
        g_InControlState.TravelLength.x = -lx;
        g_InControlState.TravelLength.z = -ly;

        // Will use Right now stick to control turret.
        g_InControlState.TurretRotAngle1 =  max(min(g_InControlState.TurretRotAngle1+command.rightH/5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
        g_InControlState.TurretTiltAngle1 =  max(min(g_InControlState.TurretTiltAngle1+command.rightV/5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
#endif
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x =  SmoothControl(((lx)*2/3), g_InControlState.BodyPos.x, SmDiv);
      g_InControlState.BodyPos.z =  SmoothControl(((ly)*2/3), g_InControlState.BodyPos.z, SmDiv);
      g_InControlState.BodyRot1.y = SmoothControl(((command.rightH)*2), g_InControlState.BodyRot1.y, SmDiv);

      //      g_InControlState.BodyPos.x = (lx)/2;
      //      g_InControlState.BodyPos.z = -(ly)/3;
      //      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_BodyYShift = (-(command.rightV)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (ly);
      g_InControlState.BodyRot1.y = (command.rightH)*2;
      g_InControlState.BodyRot1.z = (lx);
      g_BodyYShift = (-(command.rightV)/2);
    }
#ifdef OPT_GPPLAYER
    //[GPPlayer functions]
    if (ControlMode == GPPLAYERMODE) {
      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ((g_sGPSMController != 32767)  
          || (command.rightV > 16) || (command.rightV < -16))
        {
          // We are in speed modify mode...
          if (command.rightV >= 0)
            g_sGPSMController = map(command.rightV, 0, 127, 0, 200);
          else  
            g_sGPSMController = map(command.rightV, -127, 0, -200, 0);
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }
      }

      //Switch between sequences
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          if (GPSeq < 5) {  //Max sequence
            MSound (1, 50, 1500);  
            GPSeq = GPSeq+1;
          } 
          else {
            MSound (2, 50, 2000, 50, 2250);
            GPSeq=0;
          }
        }
      }
      //Start Sequence
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        }
        else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
          MSound (2, 50, 2000, 50, 2000);
        }
      }

    }
#endif // OPT_GPPLAYER

    //[Single leg functions]
#ifdef OPT_SINGLELEG      
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        MSound (1, 50, 2000);  
        if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

#if 0
      g_InControlState.SLLeg.x= (signed char)((int)((int)lx+128)/2); //Left Stick Right/Left
      g_InControlState.SLLeg.y= (signed char)((int)((int)command.rightV+128)/10); //Right Stick Up/Down
      g_InControlState.SLLeg.z = (signed char)((int)((int)ly+128)/2); //Left Stick Up/Down
#else
      // BUGBUG:: Need to figure out a decent range for these values... 
      g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
      g_InControlState.SLLeg.y = command.rightV / 3 - 20; //Right Stick Up/Down
      g_InControlState.SLLeg.z = ly; //Left Stick Up/Down
#endif
#ifdef DEBUG_SINGLELEG
      Serial.print(g_InControlState.SLLeg.x, DEC);
      Serial.print(",");
      Serial.print(g_InControlState.SLLeg.y, DEC);
      Serial.print(",");
      Serial.println(g_InControlState.SLLeg.z, DEC);
#endif
      // Hold single leg in place
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound (1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
      }
    }
#endif

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(command.rightH));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
      // User asked for manual leg adjustment - only do when we have finished any previous adjustment

        if (!g_InControlState.ForceGaitStepCnt) {
        if (sLegInitXZAdjust)
          g_fDynamicLegXZLength = true;

        sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
        // Handle maybe change angles...
        if (sLegInitAngleAdjust) 
          RotateLegInitAngles(sLegInitAngleAdjust);

        // Give system time to process previous calls
        AdjustLegPositions(sLegInitXZAdjust);
      }
    }    

    if (fAdjustLegPositions && !g_fDynamicLegXZLength)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev = command.buttons;
    extPrev = command.ext;
    //g_ulLastMsgTime = millis();
  }
  }
}

/*
boolean CommanderInputController::ProcessTerminalCommand(byte *psz, byte bLen)
{
}
*/





//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef OPT_SINGLELEG      
  g_InControlState.SelectedLeg = 255;
#endif
  g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

  g_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}

