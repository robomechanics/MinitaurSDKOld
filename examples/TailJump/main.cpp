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
// problem right now: landing isn't consistent because pitch correction is not working well.
// solution 1: add extra front leg push off time to offset pitch problem : done
// solution 2: increase speed at which tail flicks back: done
// solution 3: alter land angle such that pitch is being taken into account: done

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[9] =  {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 4.14}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

//remember to change the makefile if using rml limb!
rmlLimb leg[4]; // declare RML limb

int stateflag = 0; // used for debugging
enum jumpMode
{
    KILL,
    SQUAT,
    EXTEND,
    CONTRACT,
    FLIGHT,
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
    jumpMode mode = SQUAT;
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
    int tailReachedEnd = 0;
    int tContract;
    int landPrep = 0;
    int landCounterFull = 0;
    int hasLanded = 0;
    int tailDone = 0;

    float xRef, yRef; // reference x and y values
    float ext0, ext1; // extensions of legs 0 and 1
    float ang0, ang1; // angles fo legs 0 and 1
    float x0, y0, x1, y1, diff0, diff1; // used to calculate leg end effector positions


    float standAng = 0.1;
    float standExt = HALF_PI;
    float squatAng = radians(21);
    float squatExt = 1.0;
    float contractAng = standAng;
    float contractExt = squatExt;
    float tailJumpAng = 0.70; 
    float tailJumpFrontAng = 0.85; 
    float landRefAng = radians(-30);
    float landAng; // calculated using pitch and landRefAng
    float landExt = 0.15; // Important, this one is in meters instead of angle

    //IMU arrays for averaging (low pass filter)
    static const int filterSize = 10;
    float ac_x[filterSize];
    float ac_z[filterSize];

    float tailVel;
    float forwardAccel;
    float upAccel;
    int whipTime = 0;
    float tailPos;
    float tau; // tail torque for open loop custom controls
    bool useTail = 1;

    //sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
    // which in turn forces the state machine into FH_LEAP
    void signal(uint32_t sig) {
        // left is 2, right is 3
        if (sig == 2){
            mode = KILL;
        }
        else if(sig == 3 && mode == SQUAT){
            mode = EXTEND;
            squatPrep = 0;
        }
            
    }


    // begin() is called once when the behavior starts
    void begin() {

        // Command limbs
        C->mode = RobotCommand_Mode_JOINT;
        mode = SQUAT;
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
            if (mode == FLIGHT || mode == LAND){
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

                case KILL:
                    stateflag = -1;
                    for (int i = 0; i<9; ++i){
                        joint[i].setOpenLoop(0);
                    }
                    break;
                case SQUAT:
                    stateflag = 2;
                    if (squatPrep == 0){ // start a timer
                        tSquatStart = tCurrent; 
                        squatPrep = 1;
                    }
                    for (int i = 0; i<4; ++i){ // desired leg positions
                        leg[i].setGain(ANGLE,1.5, 0.007);
                        leg[i].setGain(EXTENSION,.6, 0.005);
                        leg[i].setPosition(ANGLE,squatAng);
                        leg[i].setPosition(EXTENSION,squatExt);
                    }
                    if (useTail){
                        if (tailPos < PI/4){
                            joint[8].setGain(0.1, 0.006);
                            joint[8].setPosition(PI/2); //prepare tail
                        } else if (tailPos > PI/2){
                            joint[8].setOpenLoop(-0.15);
                        }else{
                            tau = 0.1-0.05*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                            joint[8].setOpenLoop(tau);
                        }
                    }
                    /*
                    if (tCurrent - tSquatStart > 1000){ // been squatting a while
                        mode = EXTEND; // go to next phase
                        squatPrep = 0;
                        break;
                    }
                    */
                    break;


                case EXTEND:
                /*
                starts flicking tail
                tail goes to some angle
                set position on leg joints
                want to have it so that robot is airborne when tail is upright
                */
                    stateflag = 3;
                    if (extendPrep == 0){ // sets a time when extending starts
                        tExtend = tCurrent;
                        extendPrep = 1;
                    }

                    for (int i = 0; i<4; ++i){
                        if (i==0 || i==2){ // front legs should do more work
                            if (tailPos < tailJumpAng){
                                leg[i].setOpenLoop(EXTENSION, 2.0); 
                            } else if (tailPos < tailJumpFrontAng){
                                leg[i].setOpenLoop(EXTENSION, 2.0);
                            }
                        } else { // back legs start later
                            if (tailPos < tailJumpAng){
                                leg[i].setOpenLoop(EXTENSION, 2.0);
                            }
                        }
                    }
                    
                    // tail
                    if (useTail){
                        joint[8].setOpenLoop(-1.0);   
                    }
                    
                    //exit condition 1: if back legs are nearly fully extended
                    if (leg[1].getPosition(EXTENSION)>2.8 ||
                        leg[3].getPosition(EXTENSION)>2.8){
                        mode = CONTRACT;
                        extendPrep = 0;
                    }
                    // exit condition 2:
                    if (tCurrent - tExtend > 700){
                        extendPrep = 0;
                        mode = CONTRACT;
                    }

                    break;

                case CONTRACT:
                /*
                take legs in
                keep flicking tail
                */
                    stateflag = 4;
                    if (contractPrep == 0){
                        tContract = tCurrent;
                        contractPrep = 1;
                    }

                    if (useTail){
                        if (tailPos < (-PI/2 + 0.3)){// past limit
                            tailReachedEnd = 1;
                        }
                        if (tailReachedEnd == 0){ //going forward
                            if (tailPos > (-PI/4)){ // not yet near limit
                                joint[8].setOpenLoop(-1.0);
                            } else{ // near limit
                                tau = -0.7-0.1*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                                joint[8].setOpenLoop(tau);
                            }
                        } else { // tail has reached end
                            joint[8].setOpenLoop(0.7); // swing tail forward
                        }
                    } 
                    // leg stuff
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE,0.5, 0.005);
                        leg[i].setGain(EXTENSION,.6);
                        leg[i].setPosition(ANGLE, contractAng);
                        leg[i].setPosition(EXTENSION, contractExt);
                    }

                    // condition to go to FLIGHT: leg has contracted in angle and extension
                    if (almostEq(leg[1].getPosition(EXTENSION), contractExt) &&
                        almostEq(leg[3].getPosition(EXTENSION), contractExt)){
                        mode = FLIGHT;
                        contractPrep = 0;
                    }

                    // other condition to FLIGHT
                    if (tCurrent - tContract > 500){
                        mode = FLIGHT;
                        contractPrep = 0;
                    }
                    

                    break;

                case FLIGHT:
                    stateflag = 5;
                    landAng = -S->imu.euler.y + landRefAng;
                    for (int i = 0; i<4; ++i){
                        leg[i].setGain(ANGLE, 0.7, 0.02);
                        leg[i].setPosition(ANGLE,landAng);
                        leg[i].setGain(EXTENSION,50, 0.02); // high because extension is in meters
                        leg[i].setPosition(EXTENSION,landExt);
                    }

                    // tail stuff
                    if (useTail){
                        if (tailPos < (-PI/2 + 0.3)){// past limit
                            tailReachedEnd = 1;
                        }
                        if (tailReachedEnd == 0){ //going forward
                            if (tailPos > (-PI/4)){ // not yet near limit
                                joint[8].setOpenLoop(-1.0);
                            } else{ // near limit
                                tau = -1.0-0.1*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                                joint[8].setOpenLoop(tau);
                            }
                        } else {
                            joint[8].setOpenLoop(0.7); // swing tail back forward
                        }
                    }

                    if (almostEq(leg[0].getPosition(EXTENSION), landExt, 0.02) &&
                        almostEq(leg[2].getPosition(EXTENSION), landExt, 0.02)){
                        mode = LAND;
                    }
                    break;


                case LAND:
                    stateflag = 6;
                    landAng = -S->imu.euler.y + landRefAng;
                    if (hasLanded == 0){
                        stateflag = 6;
                    } else {
                        stateflag = 7;
                    }
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
                        leg[i].setGain(ANGLE, 0.6, 0.03);
                        leg[i].setPosition(ANGLE,landAng);
                        leg[i].setGain(EXTENSION,60, 0.1);
                        leg[i].setPosition(EXTENSION,landExt);
                    }

                    // checks how much the leg has contracted
                    // front legs
                    ext0 = leg[0].getPosition(EXTENSION);
                    ang0 = leg[0].getPosition(ANGLE);
                    ext1 = leg[2].getPosition(EXTENSION);
                    ang1 = leg[2].getPosition(ANGLE);
                    // reference x and y are determined by landRefAng and landExt
                    xRef = landExt*fastcos(landAng);
                    yRef = landExt*fastsin(landAng);

                    x0 = ext0*fastcos(ang0);
                    y0 = ext0*fastsin(ang0);
                    x1 = ext1*fastcos(ang1);
                    y1 = ext1*fastsin(ang1);
                    diff0 = fastsqrt((x0-xRef)*(x0-xRef) + (y0-yRef)*(y0-yRef));
                    diff1 = fastsqrt((x1-xRef)*(x1-xRef) + (y1-yRef)*(y1-yRef));
                    
                    if (diff0 > 0.03 || diff1 > 0.03){
                        hasLanded = 1;
                    }


                    // is stable
                    if (hasLanded && (diff0 > 0.05 || diff1 > 0.05)){
                        mode = POSTLAND;
                        landCounterFull = 0; // reset flags
                        counter = 0;
                        hasLanded = 0;
                    }

                    // tail stuff
                    if (useTail){
                        if ((tailReachedEnd == 0 && tailPos < (-PI/2 + 0.3))|| hasLanded){// past limit or has landed
                            tailReachedEnd = 1;
                        }
                        if (tailReachedEnd == 0){ //swing tail back normally
                            if (tailPos > (-PI/4)){ // not yet near limit
                                joint[8].setOpenLoop(-1.0);
                            } else{ // near limit
                                tau = -1.0-0.1*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                                joint[8].setOpenLoop(tau);
                            }
                        } else { // tail has swung all the way back
                            if (tailDone == 0 && tailPos < radians(60)){ // 
                                joint[8].setOpenLoop(0.7); // swing tail back forward
                            } else {
                                tailDone = 1;
                                joint[8].setGain(0.3, 0.006);
                                joint[8].setPosition(0);
                            }
                        }
                    }
                    counter = (counter+1)%filterSize;
                    break;

                case POSTLAND:
                    stateflag = 8;
                    // forward momentum should have been removed

                    // legs should just stand like normal
                    for (int i=0; i<4; ++i){
                        leg[i].setGain(ANGLE,1, 0.005);
                        leg[i].setGain(EXTENSION,.3, 0.005);
                        leg[i].setPosition(ANGLE,standAng);
                        leg[i].setPosition(EXTENSION,standExt);
                    }
                    // tail should try to go upright
                    if (useTail){
                        joint[8].setGain(0.4,0.01);
                        joint[8].setPosition(0);
                    }
                    break;
            }

        }
        // Stand when STOP
        else if (C->behavior.mode == BehaviorMode_STOP){
            mode = SQUAT;
            stateflag = 0;
            hasLanded = 0;
            standPrep = 0;
            squatPrep = 0;
            squatReady = 0;
            contractPrep = 0;
            landPrep = 0;
            extendPrep = 0;
            landCounterFull = 0;
            hasLanded = 0;
            tailReachedEnd = 0;
            for (int i = 0; i<4; i++){
                leg[i].setGain(ANGLE,.9, 0.005);
                leg[i].setGain(EXTENSION,.6, 0.005);
                leg[i].setPosition(ANGLE,standAng);
                leg[i].setPosition(EXTENSION,standExt);
            }

            if (useTail){
                joint[8].setGain(0.5, 0.005);
                joint[8].setPosition(0);
            }
            
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
    printf("tailPos = %f\n", joint[8].getPosition());
    /*
    float landRefAng = radians(-20);
    float landExt = 0.17; // Important, this one is in meters instead of angle
    float ext0, ext1, ang0, ang1, xRef, yRef, diff0, diff1, x0, y0, x1, y1;
    ext0 = leg[0].getPosition(EXTENSION);
    ang0 = leg[0].getPosition(ANGLE);
    ext1 = leg[2].getPosition(EXTENSION);
    ang1 = leg[2].getPosition(ANGLE);
    // reference x and y are determined by landRefAng and landExt
    xRef = landExt*fastcos(landRefAng);
    yRef = landExt*fastsin(landRefAng);

    x0 = ext0*fastcos(ang0);
    y0 = ext0*fastsin(ang0);
    x1 = ext1*fastcos(ang1);
    y1 = ext1*fastsin(ang1);
    diff0 = fastsqrt((x0-xRef)*(x0-xRef) + (y0-yRef)*(y0-yRef));
    diff1 = fastsqrt((x1-xRef)*(x1-xRef) + (y1-yRef)*(y1-yRef));

    if (stateflag == 6 || stateflag == 7){
        printf("diff0 = %f,\tdiff1 = %f\n", diff0, diff1);
    }
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
    }
    for (int i = 0; i<8; ++i){
        df = joint[i].getOpenLoop();
        printf("joint %d, power = %f\t", i, df);
    }
    printf(" \n");*/
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
    setDebugRate(50);
        
    // Run
    return begin();
}
