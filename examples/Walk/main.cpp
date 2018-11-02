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



#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

// State machine representation of behavior
enum FHMode
{
	FH_SIT = 0,
	FH_STAND,
	FH_LEAP,
	FH_LAND
};

/**
 * See "Getting started with the FirstHop behavior" in the documentation for a walk-through
 * guide.
 */
class FirstHop : public ReorientableBehavior
{
public:
	FHMode mode = FH_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float lastExtension; //float for storing limb extension during the last control loop
	float exCmd;				 //The commanded limb extension
	float extDes;				 //The desired limb extension
	float angDes;				 // The desired limb angle

	bool unitUpdated;

	//Maximum difference between commanded and actual limb extension
	const float maxDeltaExtCmd = 0.002;
	const float kExtAnimRate = 0.002; //Maximum rate (in m/s) of change in cmdExt

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		if(sig > 1)
		{
			mode = FH_LEAP;
			tLast = S->millis;
		}
			
	}

	void begin()
	{
		mode = FH_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		exCmd = 0.14;					//Set extension command to be 0.14m, the current value of extension
		lastExtension = 0.14; // Record the previous loop's extension; it should be 0.14m
	}

	void update()
	{
		if (isReorienting())
			return;
		C->mode = RobotCommand_Mode_LIMB;
		if (mode == FH_SIT)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear limbs (outward splay due to fore-displacement of front limbs
				// and aft-displacement of rear limbs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* limb angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the limb *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 3);
				// Set the limb extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.14);
			}
		}
		else if (mode == FH_STAND)
		{
			// C->behavior.pose.position.z can be commanded from the joystick (the left vertical axis by default)
			// We map this using map() to the desired limb extension, so that the joystick can be used to raise
			// and lower the standing height between 0.12 and 0.25 m
			extDes = map(C->behavior.pose.position.z, -1.0, 1.0, 0.11, 0.25);
			//If the commanded position is significantly lower than actual position,
			// and the behavior has just switched from SIT to STAND, then we smoothly
			// interpolate commanded positions between the last extension and the desired
			// extension, at the rate set by kExtAnimRate. This prevents the robot from
			// falling to quickly.
			if (S->millis - tLast < 250 && exCmd < extDes)
			{
				exCmd = exCmd + (extDes - lastExtension) * kExtAnimRate;
			}
			else
			{
				// After this initial period, or if the initial command is higher than
				// the actual initial position, we check to makes sure that the commanded
				// position is within maxDeltaExtCmd. If it is not, simply add or subtract
				// the max delta value until the difference is less than that. This prevents
				// from changing the extension faster than maxDeltaExtCmd*CONTROL_RATE m/s.
				if (extDes - exCmd > maxDeltaExtCmd)
				{
					exCmd = exCmd + maxDeltaExtCmd;
				}
				else if (exCmd - extDes > maxDeltaExtCmd)
				{
					exCmd = exCmd - maxDeltaExtCmd;
				}
				else
				{
					exCmd = extDes;
				}
			}
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Leg splay
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.01 : -S->imu.euler.y + 0.02;
				// Stiffen the angle gain linearly as a function of the extension
				// This way, more torque is provided as the moment arm becomes longer.
				limb[i].setGain(ANGLE, 0.8 + 0.2 * ((extDes - 0.12) / 0.13), 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 4);
				// The smoothly animated limb extension
				limb[i].setPosition(EXTENSION, exCmd);
			}
		}
		else if (mode == FH_LEAP)
		{
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				// Use setOpenLoop to exert the highest possible vertical force
				limb[i].setOpenLoop(EXTENSION, 2);  
				limb[i].setGain(ANGLE, 1.0, 0.03);
				limb[i].setPosition(ANGLE, -S->imu.euler.y);
				// After the mean limb angle passes 2.7 radians (note that we have changed the limb kinematics
				// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
				if (limb[i].getPosition(EXTENSION) > 2.7)
				{
					mode = FH_LAND;
					tLast = S->millis;
					unitUpdated = false;
				}
			}


			// C->mode = RobotCommand_Mode_JOINT;
			// for (int i = 0; i < P->joints_count; ++i)
			// {
			// 	// Use setOpenLoop to exert the highest possible vertical force
			// 	joint[i].setOpenLoop(1);  

			// }

			// for (int i = 0; i < P->limbs_count; ++i)
			// {
			// 	P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
			// 	// After the mean limb angle passes 2.7 radians (note that we have changed the limb kinematics
			// 	// to LimbParams_Type_SYMM5BAR_EXT_RAD) for this case, switch into a different mode (LAND)
			// 	if (limb[i].getPosition(EXTENSION) > 3 || (S->millis - tLast >= 500))
			// 	{
			// 		mode = FH_LAND;
			// 		tLast = S->millis;
			// 		unitUpdated = false;
			// 	}
			// }
			
		}
		else if (mode == FH_LAND)
		{

			for (int i = 0; i < P->limbs_count; ++i)
			{

				// This updates the parameters struct to switch back into meters as its units.
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Sets the commanded length for landing to 0.25 meters
				exCmd = 0.25;
				// Sets the desired limb angle to be facing downward plus a limb splay in the front
				// and back.
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;

				limb[i].setGain(ANGLE, 1.2, 0.03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 150, 5);
				limb[i].setPosition(EXTENSION, exCmd);

				// Use Limb::getForce for touchdown detection, and set a 20 millisecond
				// grace period so that the limbs can settle to their landing extension,
				// without their inertia triggering a false positive.

				if (limb[i].getForce(EXTENSION) > 40 && S->millis - tLast > 20)
				{
					mode = FH_STAND;
					tLast = S->millis;
					exCmd = 0.25;
					lastExtension = 0.25;
				}
			}
		}
	}

	bool running()
	{
		return !(mode == FH_SIT);
	}
	void end()
	{
		mode = FH_SIT;
	}
};







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

  extDes = map(C->behavior.pose.position.z, -1, 1, 1.0, 2.5);
  speedDes = map(C->behavior.twist.linear.x, -1, 1, -1, 1);
  yawDes = map(C->behavior.twist.angular.z, -1, 1, -0.07, 0.07);
  if (mode == WM_SIT)
    // extDes = 0.8;

  // RELAX SIT based on if just standing
  if (fabsf(speedDes) < 0.3 && fabsf(yawDes) < 0.05)
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
  const float kExtPStance = 0.3, kExtDStance = 0.02;
  // const float kExtPStance = 0.5, kExtDStance = 0.01;
  const float kExtPFlight = 0.7, kExtDFlight = 0.005;//0.6//0.4 for lighter limbs
  const float kAngPFlight = 0.6, kAngDFlight = 0.01;// 0.2 for lighter limbs
  // Positions
  // float extDes = 2.5;//stance extension
  const float extMin = 0.3;//0.7//0.5 for light limbs minimum extension in retraction
  const float kPEPthresh = 0.2;//0.1;// (i==1 || i==3) ? 0.3 : 0.1; //0.3 and 0.1
  // Forces
  // const float kTDthresh = 5;//;




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
  float speedAccum = 0;
  int numInStance = 0;

  for (int i=0; i<4; ++i) {
    bool bRear = (i==1 || i==3);
    if (bInverted)
      bRear = !bRear;
    bool bRight = (i>1);
    // IMPORTANT: set nominal limb angles
    // float angNom = bRear ? 0.1 : 0.1;
    float angNom = bRear ? 0.1 : 0.0;
    // get positions
    float ext = limb[i].getPosition(EXTENSION);
    float extvel = limb[i].getVelocity(EXTENSION);

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
        limb[i].setOpenLoop(EXTENSION, 0.15);// 0.05 on lighter limbs
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

      limb[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - ext) - kExtDStance*extvel + rollCtrl + pitchCtrl + kOffset);
    }
    // liftoff 
    if (flightLeg==-1 && S->millis - tTD > tminstance) {
      // based on yawDes or PEP
      if (fabsf(yawDes) > 0.05 || fabsf(absAngles[i]) > kPEPthresh) {
        flightLeg = nextFlightLeg;
        tLO = S->millis;
        pep = absAngles[i];
      }
    }
  }
}

OldWalk oldWalk;

void debug()
{

	// for (int i = 0; i < P->joints_count; ++i)
	// 	{
	// 		// Use setOpenLoop to exert the highest possible vertical force
	// 		printf("Motor %d command: %4.3f \n",i, joint[i].getOpenLoop());  
	// 	}
	printf("Speed Des: %4.3f \t", oldWalk.speedDes); 
	printf("Yaw Des: %4.3f \t", oldWalk.yawDes); 
	printf("Ext Des: %4.3f \t", oldWalk.extDes);
	printf("\n");  
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

	// Declare instance of our behavior
	FirstHop firstHop;

	// Add our behavior to the behavior vector (Walk and Bound are already there)
	behaviors.push_back(&firstHop);
	behaviors.push_back(&oldWalk);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	return begin();
}
