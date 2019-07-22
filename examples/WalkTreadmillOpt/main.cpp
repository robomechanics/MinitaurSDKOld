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
#include "OldWalk.h"

#include <ThermalObserver.h>
ThermalObserver thermalObserver[8];
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

ThermalObserverPeripheral thermalObserverPeripheral;


#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {0.93, 5.712, 3.777, 3.853, 2.183, 5.96, .675, 0.82}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

#pragma pack(push, 1)
// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.
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

struct SerialCommandPacket
{
  uint32_t version;
  OptPacket behavior_command;
  uint16_t checksum;
};

const char ALIGNMENT_WORD[2] = {'G', 'R'};

struct SerialStatePacket
{
  char align[2];
  uint32_t millis, lastRX; // report last reception from the computer
  Vector3 euler; // some robot state
  uint16_t checksum;
};

struct tempPacket
{
  int flag;
  float temp;
};

struct SerialReturnCmdPacket //Made to view commands the robot is using for debugging
{
  char align[2];
  uint32_t millis, lastRX; // report last reception from the computer
  tempPacket tempData; // behavior commands
  uint16_t checksum;
};
#pragma pack(pop)

/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
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

  // FROM COMMAND ROBOT------------------------------------------
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
  float curSpeed = 0.0;
  float curYaw = 0.0;
  float optVar1 = 0.0; //speed
  float optVar2 = 1.5; //stance height
  float optVar3 = 0.3; //Kp stance
  float optVar4 = 0.02; //Kd stance
  float optVar5 = 0.15; //TD open loop gain
  float optVar6 = 0.0;
  float optVar7 = 0.0;


  // Keep track of our last state packet send
  const static uint32_t TX_EVERY_MS = 10;
  uint32_t lastTX = 0;

  // Helper function to calculate a simple sum-of-bytes checksum
  uint16_t bufChecksum(const uint8_t *buffer, uint16_t nbytes)
  {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < nbytes; ++i)
    {
      sum += buffer[i];
    }
    return sum;
  }
  //---------------------------------------------------------------

  void update()
  { 
    //COMMAND ROBOT CODE----------------------------------------------
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
          //printf("Checksum %u %u.\n", checksum, serial_packet.checksum);

          // Reset
          numAlignmentSeen = rxPtr = 0;
        }
      }
    }

    // FROM COMMAND ROBOT----------------------------------------
    // Send state
    if (S->millis - lastTX > TX_EVERY_MS)
    {
      /*
      lastTX = statePacket.millis = S->millis;
      memcpy(statePacket.align, ALIGNMENT_WORD, 2);
      memcpy(&statePacket.euler, &S->imu.euler, sizeof(Vector3));
      statePacket.checksum = bufChecksum((const uint8_t *)&statePacket, sizeof(SerialStatePacket) - 2);
      write(STDOUT_FILENO, &statePacket, sizeof(statePacket));
      */
      rtnData.flag = thermalObserverPeripheral.offFlag;
      rtnData.temp = thermalObserverPeripheral.maxTemp;

      lastTX = rtnPacket.millis = S->millis;
      memcpy(rtnPacket.align, ALIGNMENT_WORD, 2);
      memcpy(&rtnPacket.tempData, &rtnData, sizeof(tempPacket));
      rtnPacket.checksum = bufChecksum((const uint8_t *)&rtnPacket, sizeof(SerialReturnCmdPacket) - 2);
      write(SERIAL_AUX_FILENO, &rtnPacket, sizeof(rtnPacket));

    }
    //-------------------------------------------------------------
  }
};

CommandRobot cmdRobot;


void OldWalk::signal(uint8_t sig) {

  if (sig == 1) {}
}

void OldWalk::update() {

	C->mode = RobotCommand_Mode_LIMB;

  // Walk code ------------------------------
  for(int i = 0; i<P->limbs_count; ++i)
  {
  	P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
  }
  


  // For inverted operation and roll/pitch recovery ---------------------------
  // 1) take it out of walking mode if not close to nominal
  if (isReorienting()) {
    this->mode = WM_REORIENT;
    return;
  }

  
  // for the rest of the code, either bInverted or bUpright is true; use either to check
  // if running in inverted mode.

  //extDes = map(C->behavior.pose.position.z, -1, 1, 1.0, 2.5);
  extDes = cmdRobot.optVar2; //1.5;
  float extDesDeadband = 0.009;
  speedDes = cmdRobot.optVar1;//cmdRobot.curSpeed; //map(cmdRobot.curSpeed, -1, 1, -1, 1);
  yawDes = cmdRobot.curYaw; //map(cmdRobot.curYaw, -1, 1, -0.07, 0.07);
  //if (mode == WM_SIT)
    // extDes = 0.8;

  // RELAX SIT based on if just standing
  if (fabsf(speedDes) < 0.3 && fabsf(yawDes) < extDesDeadband)
    relaxTimer++;
  else
    relaxTimer=0;
  // if (relaxTimer > 2000 && mode == WM_WALK)
  //   sit();
  // if (relaxTimer < 2000 && mode == WM_SIT && remoteEnableSignal)
  //   walk();

  // SIT MODE ------------------------------
  const uint32_t tSitStandAnim = 700;
  if (mode == WM_SIT || (mode == WM_WALK && S->millis - tstart < tSitStandAnim)) {
    // stand
    for (int i=0; i<4; ++i) {
      float ext = limb[i].getPosition(EXTENSION);
      limb[i].setGain(EXTENSION, 0.4);
      limb[i].setGain(ANGLE, 0.6, 0.015);
      float angDes = (i==0||i==2) ? 0.0 : 0.15;
      if (bInverted)
        angDes = (i==0||i==2) ? 0.15 : 0.0;
      if (mode == WM_WALK) {
        // move slowly to stand (extDes is far away)
        limb[i].setPosition(EXTENSION, map(S->millis-tstart, 0, tSitStandAnim, 0.6, extDes));
      } else if (S->millis - tstart < tSitStandAnim && ext > extDes + 0.05) {
        // stand to sit
        limb[i].setPosition(EXTENSION, ext-0.001);
      } else 
        limb[i].setPosition(EXTENSION, extDes);
      limb[i].setPosition(ANGLE, angDes - S->imu.euler.y);
    }
    // hitting remote switch moves to wait, and then it waits here 200ms before enabling
    // if (mode == WM_WAIT && S->millis - tstart > 200)
    //   mode = WM_WALK;
    return;
  }
  // Assume always running (or other behaviors can switch into it)
  if (S->millis - lastUpdate > 100) {
    init();
  }
  lastUpdate = S->millis;

  // SOME PARAMETERS HERE, SOME BELOW -------------------
  const float kSpeed = 0.004;//how the COM reacts to remote
  const float kYaw = 0.05;//how yaw control reacts to remote
  // flightLeg == -1 allows some limb to lift off
  // if flightLeg >= 0, all other limbs are inhibited from lifting off

  // Times
  const int tflight = 170, tminstance = 100;
  // Gains attitude control
  const float kPitchD = 0.06;
  const float kRoll = 1.5, kRollD = 0.02;
  // const float kRoll = 2.5, kRollD = 0.03;
  // Gains position control
  //const float kExtPStance = 0.3, kExtDStance = 0.02;
  float kExtPStance = cmdRobot.optVar3;
  float kExtDStance = cmdRobot.optVar4;
  const float kExtPFlight = 0.7, kExtDFlight = 0.005;//0.6//0.4 for lighter limbs
  const float kAngPFlight = 0.6, kAngDFlight = 0.01;// 0.2 for lighter limbs
  // Positions
  // float extDes = 2.5;//stance extension
  float extMin = 0.24; //from last opt run //0.3;//0.7//0.5 for light limbs minimum extension in retraction
  const float kPEPthresh = 0.2;//0.1;// (i==1 || i==3) ? 0.3 : 0.1; //0.3 and 0.1
  // Forces
  // const float kTDthresh = 5;//;
  float tdGain = cmdRobot.optVar5



  // Variables ----------------------------------------------------------------
  float uroll = 0;
  if (posRollMode) {
    uroll = -kRoll*S->imu.euler.x+ kRollD*S->imu.angular_velocity.x;
  } else {
    uroll = kRoll*S->imu.euler.x+ kRollD*S->imu.angular_velocity.x;
  }
  if (bInverted) {
    // roll goes back down to zero when upside down but the "sign" is different
    uroll = kRoll*S->imu.euler.x- kRollD*S->imu.angular_velocity.x;
    if (posRollMode)
      uroll = -kRoll*S->imu.euler.x- kRollD*S->imu.angular_velocity.x;
  }
  float uspeed = kSpeed*speedDes;
  // body pitch should conform to slope (add pitch damping), but roll should correct to 0. instead limbs should point vertically down
  float upitch = kPitchD*S->imu.angular_velocity.y;

  // Trot walk ----------------------------------------------
  // flightLeg will be -1, 0, or 1
  //float speedAccum = 0;
  //int numInStance = 0;

  for (int i=0; i<4; ++i) {
    bool bRear = (i==1 || i==3);
    if (bInverted)
      bRear = !bRear;
    bool bRight = (i>1);
    // IMPORTANT: set nominal limb angles
    // float angNom = bRear ? 0.1 : 0.1;
    //float angNom = bRear ? 0.1 : 0.0;
    float angNom = bRear ? cmdRobot.optVar4 : cmdRobot.optVar5;
    // // get positions
    // float ext = limb[i].getPosition(EXTENSION);
    // float extvel = limb[i].getVelocity(EXTENSION);

    // Diagonal pair
    // Normally two limbs in flight, but if stepping is activated only stepLeg is in flight
    if ((stepLeg == -1 && (flightLeg == i || flightLeg + i == 3)) || (stepLeg == i)) {
      // pair in flight
      // FLIGHT CONTROL
      frac = map(S->millis, tLO, tLO + tflight, 0.0, 1.0);
      // step over obstacle (IGNORE FOR NOW)
      // FIXME use remote for now

      limb[i].setGain(EXTENSION, kExtPFlight, kExtDFlight);
      // retract for flight path
      float extRetract = (extDes - extMin);
      // try to soften touchdown FIXME touchdown detection
      if (frac < 0.75) {
        // float sinarg = frac * PI;
        // try starting further along in the sin to get retract faster
        float sinarg = map(frac, 0.0, 1.0, 0.5, PI);
        limb[i].setPosition(EXTENSION, extDes - extRetract * arm_sin_f32(sinarg));
      }
      else {
        // No TD detection
        limb[i].setOpenLoop(EXTENSION, tdGain);// 0.05 on lighter limbs //0.15
      }
      // AEP based on PEP
      const float TDFRAC = 0.8;
      const float aep = -1.0*pep;//max(-1*pep, -1);
      // if this is the other one within the pair when a nextStepLeg is selected
      if (nextStepLeg+i == 3) {
        // for stepping over obstacles; ignore for now
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
      } else {
        // before was 
        absAngles[i] = (1-frac)*pep + frac*aep;
        // now want a retraction on liftoff to clear branches etc.
        // float pep2 = (pep > 0) ? pep + 0.2 : pep - 0.2;
        // float aep2 = (aep > 0) ? aep + 0.1 : aep - 0.1;
        // if (frac < 0.1)
        //   absAngles[i] = (0.1-frac)/0.1*pep + frac/0.1*pep2;
        // else if (frac < 0.6)
        //   absAngles[i] = (0.6-frac)/0.5*pep2 + (frac-0.1)/0.5*aep2;
        // if (frac < 0.6)
        //   absAngles[i] = (0.6-frac)/0.6*pep + frac/0.6*aep2;
        // else
        //   absAngles[i] = (TDFRAC-frac)/0.2*aep2 + (frac-0.6)/0.2*aep;
        // if (frac > 0.2 && frac < 0.3)
        //   absAngles[i] = (0.3-frac)/0.1*pep + (frac-0.2)/0.1*pep2;
        // else
        //   absAngles[i] = (TDFRAC-frac)/0.6*pep2 + (frac-0.2)/0.6*aep;
      }
      limb[i].setGain(ANGLE, kAngPFlight,kAngDFlight);//(frac < 0.5) ? kAngPFlight : 0.1);
      limb[i].setPosition(ANGLE, angNom + absAngles[i] - S->imu.euler.y);
      // TOUCHDOWN
      // limb inertial forces are too high for touchdown detection
      if (/*ur[i] > kTDthresh && */frac >= TDFRAC) {
        if (nextStepLeg != -1) 
          stepLeg = nextStepLeg;
        else {
          flightLeg = -1;
          // nextStepLeg, stepLeg will both be reset to -1 after the step
          nextFlightLeg = (i==0 || i==3) ? 1 : 0;// i == 0/3 -> 1, i == 1/2->0
          tTD = S->millis;
        }
      }
    } else {
      bool limbJustTouchedDown = !(nextFlightLeg == i || nextFlightLeg + i == 3);
      // accumulate speed for stance limbs
      // for limbs that just touched down wait 50ms after TD
      // if (S->millis - tTD > 50 || !limbJustTouchedDown) {
      //   // there is some lag between the flightLeg being set and the 
      //   // limb actually touching down (from looking at logs)
      //   speedAccum += limb[i].getSpeed(S->imu.euler.y);
      //   numInStance++;
      // }
      // Leg i is in stance. STANCE CONTROL - try to keep body level
      limb[i].setGain(ANGLE, 1.0, 0.01);
      // lean forward based on speedDes, but add some decay
      float uyaw = bRight ? -kYaw*yawDes : kYaw*yawDes;
      if (flightLeg == -1 || fabsf(absAngles[i] > 0.3)) uyaw = 0;
      if (S->millis - tTD > 25 || !limbJustTouchedDown)
        absAngles[i] += uspeed + uyaw - 0.001*absAngles[i];
      // TEST
      if (nextStepLeg != -1) {
        // move back
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
        // limb behind or across
        // else
        //   extDes = 2.8;
      }

      limb[i].setPosition(ANGLE, angNom + absAngles[i] - S->imu.euler.y);
      // apply to limbs correctly
      float rollCtrl = bRight ? uroll : -uroll;
      float pitchCtrl = bRear ? -upitch : upitch;

      if (nextStepLeg != -1) {
        // try to get limb to not EXTEND for roll control
        if (rollCtrl > 0)
          rollCtrl = 0;
        // pitchCtrl = 0;
      }

      float kOffset = (flightLeg == -1) ? 0 : 0.03;

      // get positions
      float ext = limb[i].getPosition(EXTENSION);
      float extvel = limb[i].getVelocity(EXTENSION);

      limb[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - ext) - kExtDStance*extvel + rollCtrl + pitchCtrl + kOffset);
    }
    // liftoff 
    if (flightLeg==-1 && S->millis - tTD > tminstance) {
      // based on yawDes or PEP
      if (fabsf(yawDes) > extDesDeadband || fabsf(absAngles[i]) > kPEPthresh) {
        flightLeg = nextFlightLeg;
        tLO = S->millis;
        pep = absAngles[i];
      }
    }
  }
}

void debug()
{

	// for (int i = 0; i < P->joints_count; ++i)
	// 	{
	// 		// Use setOpenLoop to exert the highest possible vertical force
	// 		printf("Motor %d command: %4.3f \n",i, joint[i].getOpenLoop());  
	// 	}

	//printf("Speed Des: %d \t", thermalObserverPeripheral.offFlag); 
	//printf("Yaw Des: %4.3f \t", thermalObserverPeripheral.maxTemp); 
	//printf("Ext Des: %4.3f \t", oldWalk.extDes);
	//printf("\n");  
}

OldWalk oldWalk;

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

	setDebugRate(20);

  // Disable joystick input
  JoyType joyType = JoyType_NONE;
  ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  // Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings

  SerialPortConfig cfgAux;
  cfgAux.baud = 115200;
  cfgAux.mode = SERIAL_8N1;
  ioctl(SERIAL_AUX_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfgAux);

  SerialPortConfig cfg;
  cfg.baud = 115200;
  cfg.mode = SERIAL_8N1;
  ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

  thermalObserverPeripheral.begin();
  addPeripheral(&thermalObserverPeripheral);
  // Create and add controller peripheral
  addPeripheral(&cmdRobot);
  cmdRobot.begin();

  // Remove all GR behaviors from Minitaur and add our behavior
  behaviors.clear();
  behaviors.push_back(&oldWalk);

	return begin();
}
