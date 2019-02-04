/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Raj Patel <raj.patel@ghostrobotics.io>, Tom Jacobs <tom.jacobs@ghostrobotics.io>, and Avik De <avik@ghostrobotics.io>
 */

#include <stdio.h>
#include <SDK.h>
#include <Motor.h>
#include "rmlLimb.h"

/* Jumping with tail using RML limb*/

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[9] =  {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

//remember to change the makefile if using rml limb!
rmlLimb leg[4]; // declare RML limb

int stateflag = 0; // used for debugging
enum jumpMode
{
    STAND,
    SQUAT,
    EXTEND,
    CONTRACT,
    PRELAND,
    LAND,
    POSTLAND
};

bool almostEq (float x, float y, float tol = 0.2){
    return abs(x-y) <= tol;
}

float avg(float list[], int n){
    float sum = 0;
    float result;
    for (int i=0; i<n; ++i){
        sum += list[i];
    }
    result = sum/float(n);
    return result;
}

class Starter : public Behavior {
public:
    jumpMode mode = STAND;
    int tCurrent;
    int standPrep = 0;
    int tStand;
    int squatPrep = 0;
    int squatReady = 0;
    int tSquatStart;
    int extendPrep = 0;
    int tExtend;
    int tSquat;
    int contractPrep = 0;
    int tContract;
    int landPrep = 0;
    int landCounterFull = 0;
    int hasLanded = 0;

    float xRef, yRef; // reference x and y values
    float ext0, ext1; // extensions of legs 0 and 1


    float standAng = 0.1;
    float standExt = HALF_PI;
    float squatAng = 0.7;
    float squatExt = 0.8;
    float contractAng = standAng;
    float contractExt = squatExt;
    float landAng = radians(-20);
    float landExt = 0.2; // Important, this one is in meters instead of angle

    //IMU arrays for averaging (low pass filter)
    static const int filterSize = 12;
    float ac_x[filterSize];
    float ac_z[filterSize];

    float tailVel;
    float forwardAccel;
    float upAccel;
    int whipTime = 0;
    float tailPos;
    float tau; // tail torque for open loop custom controls
    bool useTail = 1;

    // begin() is called once when the behavior starts
    void begin() {

        // Command limbs
        C->mode = RobotCommand_Mode_JOINT;
        mode = STAND;
        for (int i=0; i<4; ++i){
            P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
        }
    }

    // signal() is called when receiving a signal from the controller
    // void signal(uint32_t sig) {

    // }
    // update() is called once per loop while the behavior is running
    void update() {       
        for (int i=0; i<4; ++i){
            if (mode == LAND){
                P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
            } else {
                P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
            }
            leg[i].updateState();
        }
        tCurrent = S->millis;
        tailPos = joint[8].getPosition();
        tailVel = joint[8].getVelocity();
        // Starter when RUN
        if (C->behavior.mode == BehaviorMode_RUN){
            switch(mode){
                case STAND:
                    stateflag = 1;
                    if (standPrep == 0){
                        tStand = tCurrent;
                        standPrep = 1;
                    }
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE,1, 0.005);
                        leg[i].setGain(EXTENSION,.3, 0.005);
                        leg[i].setPosition(ANGLE,standAng);
                        leg[i].setPosition(EXTENSION,standExt);
                    }
                    if (useTail){
                        if (tailPos < PI/4){
                            joint[8].setGain(0.1, 0.006);
                            joint[8].setPosition(PI/2); //prepare tail
                        } else if (tailPos > PI/2){
                            joint[8].setOpenLoop(-0.25);
                        }else{
                            tau = 0.12-0.03*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                            joint[8].setOpenLoop(tau);
                        }
                    }
                    
                    //switch mode when ready
                    if (tCurrent - tStand > 2000) {// been standing a while now
                        mode = SQUAT;
                        for (int i=0; i<4; ++i){
                        }
                        standPrep = 0;
                    }
                        
                    break;

                case SQUAT:
                    stateflag = 2;
                    if (squatPrep == 0){ // start a timer
                        tSquatStart = tCurrent; 
                        squatPrep = 1;
                    }
                    for (int i = 0; i<4; ++i){ // desired leg positions
                        leg[i].setGain(ANGLE,1, 0.005);
                        leg[i].setGain(EXTENSION,.3, 0.005);
                        leg[i].setPosition(ANGLE,squatAng);
                        leg[i].setPosition(EXTENSION,squatExt);
                    }
                    if (useTail){
                        if (tailPos < PI/4){
                            joint[8].setGain(0.1, 0.006);
                            joint[8].setPosition(PI/2); //prepare tail
                        } else if (tailPos > PI/2){
                            joint[8].setOpenLoop(-0.25);
                        }else{
                            tau = 0.1-0.04*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                            joint[8].setOpenLoop(tau);
                        }
                    }
                    if (tCurrent - tSquatStart > 1000){ // been squatting a while
                        mode = EXTEND; // go to next phase
                        squatPrep = 0;
                        break;
                    }
                    
                    break;


                case EXTEND:
                /*
                starts flicking tail
                tail goes to some angle
                set position on leg joints
                want to have it so that robot is airborne when tail is upright
                */
                    if (extendPrep == 0){ // sets a time when extending starts
                        tExtend = tCurrent;
                        extendPrep = 1;
                    }

                    //if (tailPos < tailJumpAng){
                    if (true){ //if you want to do it without tail
                        // use setOpenLoop(1) to give maximum power
                        for (int i = 0; i<4; ++i){
                            leg[i].setOpenLoop(1);
                        }
                    }
                    
                    // tail
                    if (useTail){
                        joint[8].setOpenLoop(-0.5);   
                    }
                    
                    //exit condition 1: if front legs are nearly fully extended
                    if (leg[0].getPosition(EXTENSION)>2.8 ||
                        leg[1].getPosition(EXTENSION)>2.8){
                        mode = CONTRACT;
                        extendPrep = 0;
                    }
                    // exit condition 2:
                    if (tCurrent - tExtend > 400){
                        extendPrep = 0;
                        mode = CONTRACT;
                    }

                    break;

                case CONTRACT:
                /*
                take legs in
                keep flicking tail
                */
                    if (contractPrep == 0){
                        tContract = tCurrent;
                        contractPrep = 1;
                    }
                    if (useTail){
                        if (tailPos > (-PI/4)){ // not yet near limit
                            joint[8].setOpenLoop(-0.7);
                        } else if (tailPos < (-PI/2)){ // past limit
                            joint[8].setOpenLoop(0.3);
                        }
                        else{ // near limit
                            tau = -0.1-0.05*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                            joint[8].setOpenLoop(tau);
                        }
                        
                    }
                    // leg stuff
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE,0.5, 0.005);
                        leg[i].setGain(EXTENSION,.4);
                        leg[i].setPosition(ANGLE, contractAng);
                        leg[i].setPosition(EXTENSION, contractExt);
                    }

                    // condition to go to preland: leg has contracted in either angle or extension
                    if (almostEq(leg[0].getPosition(EXTENSION), contractExt) ||
                        almostEq(leg[1].getPosition(EXTENSION), contractExt)){
                        mode = LAND;
                        contractPrep = 0;
                    }

                    // other condition to preland
                    if (tCurrent - tContract > 500){
                        mode = PRELAND;
                        contractPrep = 0;
                    }
                    

                    break;

                case PRELAND:
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE, , 0.005);
                        leg[i].setPosition(ANGLE,landAng);
                        leg[i].setGain(EXTENSION,.4);
                        leg[i].setPosition(EXTENSION,landExt);
                    }
                    if (almostEq(leg[0].getPosition(EXTENSION), landExt) ||
                        almostEq(leg[1].getPosition(EXTENSION), landExt)){
                        mode = LAND;
                    }
                    break;


                case LAND:
                    // may use acceleration for determining whether robot landed
                    static int counter = 0;
                    ac_x[counter] = S->imu.linear_acceleration.x;
                    forwardAccel = avg(ac_x,filterSize);
                    ac_z[counter] = S->imu.linear_acceleration.z;
                    upAccel = avg(ac_z,filterSize);

                    if (counter == filterSize-1){
                        // filter is used to smoothen IMU accel readings
                        landCounterFull = 1;
                    }
                
                    // tries to extend legs forward to brace for landing
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE, , 0.005);
                        leg[i].setPosition(ANGLE,landAng);
                        leg[i].setGain(EXTENSION,.4);
                        leg[i].setPosition(EXTENSION,landExt);
                    }

                    // checks how much the leg has contracted
                    // front legs
                    ext0 = leg[0].getPosition(EXTENSION);
                    ang0 = leg[0].getPosition(ANGLE);
                    ext1 = leg[1].getPosition(EXTENSION);
                    ang1 = leg[1].getPosition(ANGLE);
                    // reference x and y are determined by landAng and landExt
                    xRef = landExt*fastcos(landAng);
                    yRef = landExt*fastsin(landAng);

                    float x0 = ext0*fastcos(ang0);
                    float y0 = ext0*fastsin(ang0);
                    float x1 = ext1*fastcos(ang1);
                    float y1 = ext1*fastsin(ang1);
                    float diff0 = fastsqrt((x0-xRef)*(x0-xRef) + (y0-yRef)*(y0-yRef));
                    float diff1 = fastsqrt((x1-xRef)*(x1-xRef) + (y1-yRef)*(y1-yRef));
                    
                    if (diff0 > 0.05 || diff1 > 0.05){
                        hasLanded = 1;
                    }

                    // is stable
                    if (hasLanded && (ang0 > 0 || ang1 > 0){
                        mode = POSTLAND;
                        landCounterFull = 0;
                        counter = 0;
                    }

                    // tail stuff
                    if (hasLanded){ // use tail to bring back body position
                        if (joint[8].getPosition() < 0){
                            joint[8].setOpenLoop(0.6);
                        } else{
                            joint[8].setGain(0.1, 0.006);
                            joint[8].setPosition(0);
                        } 
                    } else{ // keep swinging tail until it almost hits butt
                        tau = -0.2*logf((tailPos+PI/2)/(PI/2-tailPos));
                        joint[8].setOpenLoop(-0.7+tau);
                    }
                    counter = (counter+1)%filterSize;
               
                    break;
                case POSTLAND:
                    // forward momentum has been removed

                    // legs should just stand like normal
                    for (int i=0; i<4; ++i){
                        leg[i].setGain(ANGLE,1, 0.005);
                        leg[i].setGain(EXTENSION,.3, 0.005);
                        leg[i].setPosition(ANGLE,standAng);
                        leg[i].setPosition(EXTENSION,standExt);
                    }
                    // tail should try to go upright
                    joint[8].setGain(0.2,0.006);
                    joint[8].setPosition(0);
            }

        }
        // Stand when STOP
        else if (C->behavior.mode == BehaviorMode_STOP){
            stateflag = 0;
            hasLanded = 0;
            isStable = 0;
            standPrep = 0;
            squatPrep = 0;
            squatReady = 0;
            contractPrep = 0;
            landPrep = 0;
            extendPrep = 0;
            landCounterFull = 0;
            counter = 0;
            for (int i = 0; i<4; i++){
                leg[i].setGain(ANGLE,.9);
                leg[i].setGain(EXTENSION,.3);
                leg[i].setPosition(ANGLE,standAng);
                leg[i].setPosition(EXTENSION,standEx);
            }

            joint[8].setOpenLoop(0);
            /*
            if (tailPos > 0.1 && tailPos < PI/2){
                tau = -0.1*logf((tailPos+PI/2)/(PI/2-tailPos));
                joint[8].setOpenLoop(tau);
            } else if (tailPos > PI/2){
                tau = -0.5;
                joint[8].setOpenLoop(tau);
            }
            else{
                joint[8].setGain(0.1,0.006);
                joint[8].setPosition(0);
            }*/
        }

        for (int i=0; i<4; ++i){
            leg[i].updateCommand();
        }

    }

    // running() is called to determine whether the behavior is running or not
    // void running() {

    // }

    // end() is called when the behavior is stopped
    void end() {

    }
};


// debug() is called at the DEBUG_RATE
void debug() {
    float r, th, dr, dth, q0, q1, ang, ext, df; 
    float roll, pitch, yaw;
    printf("state = %d\n", stateflag);
    /*
    for (int i = 0; i < 4; ++i)
    {
        q0 = leg[i].q0; 
        q1 = leg[i].q1; 
        leg[i].FK(q0, q1, r, th); 

        dr = leg[i].getVelocity(EXTENSION);
        dth = leg[i].getVelocity(ANGLE);
        df = leg[i].getOpenLoop(0);
        printf("limb %d:, angle = %f, extension = %f, power = %f\n", i, th, r, df);
    }*/
    for (int i = 0; i<8; ++i){
        df = joint[i].getOpenLoop();
        printf("joint %d, power = %f\n", i, df);
    }
    pitch = S->imu.euler.y;
    printf("pitch = %f\n", pitch);
    printf(" \n");
}


// Main
int main(int argc, char *argv[]) {
#if defined(ROBOT_MINITAUR)
    // Create MINITAUR
    init(RobotParams_Type_MINITAUR, argc, argv);

    // Set motor zeros
    for (int i = 0; i < P->joints_count; ++i)
        P->joints[i].zero = motZeros[i];
#elif defined(ROBOT_MINITAUR_E)

    // Create MINITAUR_E
    init(RobotParams_Type_MINITAUR_E, argc, argv);

    // Set joystick type
    JoyType joyType = JoyType_FRSKY_XSR;
    ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif
    // Configure joints
#define NUM_MOTORS 9 // Or however many joints/motors are being used
    const float directions[NUM_MOTORS] = {1, 1, 1, 1, -1, -1, -1, -1, 1};
    P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
    for (int i = 0; i < P->joints_count; i++)
    {
      // Set zeros and directions
      P->joints[i].zero = motZeros[i];
      P->joints[i].direction = directions[i];
    }
    // Set the joint type; see JointParams
    P->joints[8].type = JointParams_Type_GRBL;

    // Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
    P->joints[8].address = 9;

    // If there is a gearbox the joint electronics doesn't know about, this could be > 1.
    // Do not set to 0.
    P->joints[8].gearRatio = 1;

    P->limbs_count = 0;
    for(int i = 0; i<4; ++i)
    {
        leg[i].Init(i);
    }

    // Uncomment to clear Bound and Walk behaviors
    behaviors.clear();

    // Create, add, and start Starter behavior
    Starter starter;
    behaviors.push_back(&starter);
    starter.begin();
    setDebugRate(2);
        
    // Run
    return begin();
}
