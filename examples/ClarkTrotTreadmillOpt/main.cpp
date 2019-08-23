/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>


#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {5.52, 5.34, 5.95, 3.81, 3.84, 4.07, 5.71, 5.38}; // RML Ellie
#endif

#include <ThermalObserver.h>
ThermalObserver thermalObserver[8];
// Thermal Observer Class
// Used to monitor motor temps to prevent motor burnouts during long run times
class ThermalObserverPeripheral : public Peripheral
{public:
  int offFlag = 0;
  int sizzleCount = 0;
  float maxTemp = 0.0;
  float tempVal = 0.0;
  void begin(){
    for(int i = 0; i < P->joints_count; ++i)
      thermalObserver[i].assignMotor(i);}
  void update(){
    for(int i = 0; i < P->joints_count; ++i)
      thermalObserver[i].update();
    for(int i = 0; i < P->joints_count; ++i){
      if (thermalObserver[i].tempCore > tempVal) {
        tempVal = thermalObserver[i].tempCore;
      }
      if (thermalObserver[i].sizzleFlag) {
        sizzleCount =+ 1;
      }
    }
    if (sizzleCount > 0) offFlag = 1;
    else offFlag = 0;
    sizzleCount = 0;
    maxTemp = tempVal;
    tempVal = 0.0;
  }
};
// Initialize Thermal Observer
ThermalObserverPeripheral thermalObserverPeripheral;

#pragma pack(push, 1)
const char ALIGNMENT_WORD[2] = {'G', 'R'}; //Alignment chars used for data syncs

//Optimization Packet, these are the variables sent from the RasPi/Main Computer
struct OptPacket
{
  float fwdVel;
  float yaw;
  float var1;
  float var2;
  float var3;
  float var4;
  float var5;
  float var6;
  float var7;
};

// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.
struct SerialCommandPacket //Whole Command Packet which includes OptPacket (see above)
{
  uint32_t version;
  OptPacket behavior_command;
  uint16_t checksum;
};

struct SerialStatePacket
{
  char align[2];
  uint32_t millis, lastRX; // report last reception from the computer
  Vector3 euler; // some robot state
  uint16_t checksum;
};

struct tempPacket //Temperature Packet
{
  int flag;
  float temp;
};

struct SerialReturnCmdPacket //Data Sent back to RasPi/Main Computer (currently just temperature info)
{
  char align[2];
  uint32_t millis, lastRX; // report last reception from the computer
  tempPacket tempData; // Temperture Info
  uint16_t checksum;
};
#pragma pack(pop)

// Command Robot Peripheral
// This is the class that executes the communication between the RasPi and the Main Board
// This class has gerneric variable outputs seen in the OptPacket above (assumes they are all floats)
class CommandRobot : public Peripheral
{
public:

  void begin()
  {
    // Command by behavior
    C->mode = RobotCommand_Mode_BEHAVIOR;

    // Start behavior
    C->behavior.mode = BehaviorMode_RUN;

    // Stand still
    C->behavior.twist.linear.x = 0.0;
  }
  // Parser state
  // Goes from 0 to 1 to 2
  int numAlignmentSeen = 0;
  uint16_t rxPtr = 0;
  // Receive buffer and alignment
  const static uint16_t RX_BUF_SIZE = 100;

  SerialCommandPacket commandPacket;
  SerialStatePacket statePacket;
  SerialReturnCmdPacket rtnPacket;

  OptPacket cmdData;
  tempPacket rtnData;

  float xVel = 0.0;
  float zAng = 0.0;
  float zVel = 0.0;
  float curSpeed = 0.0;   // On/Off Command (ON = curSpeed>0.5)
  float curYaw = 0.0;     // Yaw Command
  float optVar1 = 2.0;    // Frequnecy
  float optVar2 = 50.0;   // Duty Factor
  float optVar3 = 0.12;   // Stroke Length
  float optVar4 = 40.0;   // Approach Angle
  float optVar5 = 180.0;  // Kp
  float optVar6 = 1.8;    // Kd
  float optVar7 = 0.0;    // not used


  // Keep track of our last state packet send
  const static uint32_t TX_EVERY_MS = 10;
  uint32_t lastTX = 0;

  // Helper function to calculate a simple sum-of-bytes checksum
  uint16_t bufChecksum(const uint8_t *buffer, uint16_t nbytes) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < nbytes; ++i) {
      sum += buffer[i];
    }
    return sum;
  }

  void update() { 
    // Character to store the latest received character
    uint8_t latestRX;

    // Loop through while there are new bytes available
    while (read(SERIAL_AUX_FILENO, &latestRX, 1) > 0)
    {
      if (numAlignmentSeen == 0 && latestRX == ALIGNMENT_WORD[0])
      {
        numAlignmentSeen = 1;
      }
      else if (numAlignmentSeen == 1 && latestRX == ALIGNMENT_WORD[1])
      {
        numAlignmentSeen = 2;
        rxPtr = 0;
      }
      else if (numAlignmentSeen == 2)
      {
        // Add the next byte to our memory space in serial_packet
        uint8_t *pSerialPacket = (uint8_t *)&commandPacket;
        pSerialPacket[rxPtr++] = latestRX; // post-increment rxPtr

        // Check if we have read a whole packet
        //printf("ptr %d %d\n", rxPtr, sizeof(SerialPacket));
        if (rxPtr == sizeof(SerialCommandPacket))
        {
          // Check checksum
          uint16_t checksum = bufChecksum(pSerialPacket, sizeof(SerialCommandPacket) - 2);
          if (commandPacket.checksum == checksum)
          {
            // Check we're processing the right version
            if (commandPacket.version == 1)
            {
              // Copy our BehaviorCmd into cmdData
              memcpy(&cmdData, &commandPacket.behavior_command, sizeof(BehaviorCmd));

              // Store last received time
              statePacket.lastRX = S->millis;

              //Save values to cmdRobot public variables
              curSpeed = cmdData.fwdVel;
              curYaw = cmdData.yaw;
              optVar1 = cmdData.var1;
              optVar2 = cmdData.var2;
              optVar3 = cmdData.var3;
              optVar4 = cmdData.var4;
              optVar5 = cmdData.var5;
              optVar6 = cmdData.var6;
              optVar7 = cmdData.var7; 
            }
          }
          // Reset
          numAlignmentSeen = rxPtr = 0;
        }
      }
    }
    // Send state
    if (S->millis - lastTX > TX_EVERY_MS) {
    	//Save values to return data struct (tempPacket)
		rtnData.flag = thermalObserverPeripheral.offFlag;
		rtnData.temp = thermalObserverPeripheral.maxTemp;
		//Construct and Send Return Packet
		lastTX = rtnPacket.millis = S->millis;
		memcpy(rtnPacket.align, ALIGNMENT_WORD, 2);
		memcpy(&rtnPacket.tempData, &rtnData, sizeof(tempPacket));
		rtnPacket.checksum = bufChecksum((const uint8_t *)&rtnPacket, sizeof(SerialReturnCmdPacket) - 2);
		write(SERIAL_AUX_FILENO, &rtnPacket, sizeof(rtnPacket));
    }
  }
};
// Initialize Command Robot
CommandRobot cmdRobot;

//Your Gait Goes Here and Below!----------------------------------------------------------
//Gait Modes
enum CTMode
{
	CT_SIT = 0,
	CT_WALK
};

class ClarkTrot : public ReorientableBehavior
{
public:
	CTMode mode = CT_SIT; //Current state within state-machine

	float exCmd;	// The commanded leg extension
	float angDes;	// The desired leg angle
	float angCmd;   // angle command
	float yawCmd;   // steering command
	float angNom;   // nominal foot angle
	float onCmd;    // On/Off command for walking gait

	float freq; //4.750; 	//Frequency of gait (Hz)
	float df; 				//Duty Factor (%)
	float strokeLen;//0.16; //Stroke Length (m)
	float beta; //60; 		//Approach Angle (deg)
	float Kp; //1.70; 		//Extension Proportional Gain
	float Kd; //0.018; 		//Extension Derivative Gain

	float extRetract = 0.065; 					//Extension Retraction Height (m)
	float gcl = 0.17; 							//min ground clearance (m)

	uint32_t tStart; 							//gait start time (ms)
	uint32_t cTime; 							//cycle time (ms)

	void begin() {
		mode = CT_SIT;			// Start behavior in SIT mode
		tStart = S->millis;		// Record the system time @ this transition
	}

	void update() {
		onCmd = cmdRobot.curSpeed;
		if(onCmd > 0.5) {
			mode = CT_WALK;
		}
		else {
			mode = CT_SIT;
		}

		C->mode = RobotCommand_Mode_LIMB;
		if (mode == CT_SIT) {
			for (int i = 0; i < P->limbs_count; ++i) {
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y + 5.0/180*M_PI : -S->imu.euler.y + 5.0/180*M_PI;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);
				limb[i].setGain(EXTENSION, 120, 3);
				// Set the leg extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.14);
			}
		}
		else if (mode == CT_WALK) {

			freq = cmdRobot.optVar1; //4.750; 	//Frequency of gait (Hz)
		 	df = cmdRobot.optVar2; 				//Duty Factor (%)
		 	strokeLen = cmdRobot.optVar3;//0.16; 	//Stroke Length (m)
 			beta = cmdRobot.optVar4; //60; 		//Approach Angle (deg)
 			Kp = cmdRobot.optVar5; //1.70; 		//Extension Proportional Gain
		 	Kd = cmdRobot.optVar6; //0.018; 		//Extension Derivative Gain

			float stanceExt = sqrt(pow(strokeLen/2,2)+pow(gcl,2));
			float stanceAng = atan((strokeLen/2)/gcl);

			float h4 = tan(beta*M_PI/180)*(strokeLen/3);
			float phi = atan((strokeLen*5/6)/(gcl-h4));
			float ext4 = sqrt(pow(strokeLen*5/6,2)+pow(gcl-h4,2));

			float matchFrac = (phi-stanceAng)/(stanceAng/(df/2));
			float resetFrac = 100 - df - matchFrac;
			float gclFrac = resetFrac/(phi+stanceAng)*stanceAng;

			cTime = (S->millis - tStart); //cycle time

			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				uint32_t legPhase = (i==0 || i==3) ? 0 : 1;

				yawCmd = (i==0 || i==1) ?  -cmdRobot.curYaw: cmdRobot.curYaw;

				angNom = isFront(i) ?  10.0/180*M_PI: 10.0/180*M_PI;

				float frac = (fmod(cTime + (legPhase*(1/freq)/2*1000), 1000/freq))*freq*0.1;

				if(frac <= df) {
					//stance phase (pt 1 to 2)
					angCmd = map(frac, 0, df, angNom+yawCmd-stanceAng, angNom+yawCmd+stanceAng);
					exCmd = stanceExt;
				}
			    else if(frac > df) {
			    	//flight phase
			    	if (frac < df+resetFrac){
			    		// Flight Leg Reset
			    		angCmd = map(frac, df, df+resetFrac, angNom+stanceAng, angNom-phi);
			    		if (frac < df+gclFrac) {
			    			// Ground Clearance (point 2 to 3)
			    			exCmd = map(frac, df, df+gclFrac, stanceExt, gcl-extRetract);
			    		}
			    		else if (frac > df+gclFrac) {
			    			//Approach Angle (pt 3 to 4)
			    			exCmd = map(frac, df+gclFrac, df+resetFrac, gcl-extRetract, ext4);
			    		}
			    	}
			    	else if(frac > df+resetFrac) {
			    		// Ground Speed Matching (pt 4 to 1)
			    		angCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, angNom-phi, angNom-stanceAng);
			    		exCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, ext4, stanceExt);
			    	}
			    }
			    limb[i].setGain(ANGLE, 2.2, .03);
				limb[i].setGain(EXTENSION, Kp, Kd);
			    limb[i].setPosition(ANGLE,angCmd);
			    limb[i].setPosition(EXTENSION,exCmd);
			}
		}
	}
	bool running() {return !(mode == CT_SIT);}
	void end() {mode = CT_SIT;}
};
//Initalize Your Gait
ClarkTrot cTrot;
//Your gait code ends here-------------------------------------------------------------
//(dont forget to add it to the behavior stack)

void debug()
{
	//printf("time : %lu\n", cTrot.cTime);
    //printf("On Command : %f\n", cTrot.onCmd);
    //printf("CurSpeed : %f\n", cmdRobot.curSpeed);
  	//printf("\n");
}

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif

	setDebugRate(1);

	// Disable joystick input
	JoyType joyType = JoyType_NONE;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

	//Setup Serial Communication through RasPi Headers
	SerialPortConfig cfgAux;
	cfgAux.baud = 115200;
	cfgAux.mode = SERIAL_8N1;
	ioctl(SERIAL_AUX_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfgAux);

	//Setup Serial Communication for mircoUSB/Debug
	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	//Add and Start Thermal Observer peripheral
	addPeripheral(&thermalObserverPeripheral);
	thermalObserverPeripheral.begin();
	//Add and Start Command Robot peripheral
	addPeripheral(&cmdRobot);
	cmdRobot.begin();

	// Remove all GR behaviors from Minitaur and add our behavior
	behaviors.clear();
  	behaviors.push_back(&cTrot);

	return begin();
}
