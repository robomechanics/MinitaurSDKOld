/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Raj Patel <raj.patel@ghostrobotics.io>, Tom Jacobs <tom.jacobs@ghostrobotics.io>, and Avik De <avik@ghostrobotics.io>
 */

#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[9] =  {5.200, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

enum jumpMode
{
    STAND,
    SQUAT,
    EXTEND,
    CONTRACT,
    LAND
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
    int tLast;
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
    int hasLanded = 0;
    int isStable = 0;

    float standInAng = PI/2 + 0.1;
    float standOutAng = PI/2 - 0.1;
    float squatAngle = PI/4;
    float squatInsideRef = PI/2;
    float squatOutsideRef = PI/4;
    float squatInsideAng = PI/2;
    float squatInFrontAng = squatInsideRef - 0.1;
    float squatOutFrontAng = squatOutsideRef + 0.1;
    float squatOutsideAng = PI/4;
    float extendInsideAng = 2.8;
    float extendOutsideAng = 1;
    float contractInAng = squatInsideAng;
    float contractOutAng = squatOutsideAng;
    float landInRearAng = standInAng;
    float landOutRearAng = standOutAng;
    float ang0, ang1, ang4, ang5, leftFrontAng, rightFrontAng, frontAng;

    //IMU arrays for averaging (low pass filter)
    static const int filterSize = 12;
    float ac_x[filterSize];
    float ac_z[filterSize];

    float landInFrontAng = standInAng - 0.5;
    float landOutFrontAng = standOutAng + 0.5;
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
        tLast = S->millis;
    }

    // signal() is called when receiving a signal from the controller
    // void signal(uint32_t sig) {

    // }

    // update() is called once per loop while the behavior is running
    void update() {
        int insideMot[] = {1,3,4,6};
        int outsideMot[] = {0,2,5,7};
        int frontMot[] = {0,1,4,5};
        int backMot[] = {2,3,6,7};
        int frontInMot[] = {1,4};
        int frontOutMot[] = {0,5};
        int rearOutMot[] = {2,7};
        int rearInMot[] = {3,6};
        float diff1, diff0;
        float sign1, sign0;
        
        tCurrent = S->millis;
        tailPos = joint[8].getPosition();
        tailVel = joint[8].getVelocity();
        // Starter when RUN
        if (C->behavior.mode == BehaviorMode_RUN){
            switch(mode){
                case STAND:
                    if (standPrep == 0){
                        tStand = tCurrent;
                        standPrep = 1;
                    }
                    for (int i = 0; i<4; ++i){
                        joint[insideMot[i]].setGain(0.42, 0.006);
                        joint[insideMot[i]].setPosition(standInAng);

                        joint[outsideMot[i]].setGain(0.42, 0.006);
                        joint[outsideMot[i]].setPosition(standOutAng);
                    }
                    if (useTail){
                        if (tailPos < PI/4){
                            joint[8].setGain(0.1, 0.006);
                            joint[8].setPosition(PI/2); //prepare tail
                        } else if (tailPos > PI/2){
                            joint[8].setOpenLoop(-0.3);
                        }else{
                            tau = 0.15-0.03*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                            joint[8].setOpenLoop(tau);
                        }
                    }
                    
                    //switch mode when ready
                    if (tCurrent - tStand > 2000) {// been standing a while now
                        mode = SQUAT;
                        standPrep = 0;
                        extendPrep = 0;
                    }
                        
                break;

                case SQUAT:
                    if (squatPrep == 0){ // start a timer
                        tSquatStart = tCurrent; 
                        squatPrep = 1;
                    }
                    for (int i = 0; i<4; ++i){ // desired leg positions
                        /*ang0 = joint[insideMot[i]].getPosition();
                        ang1 = joint[outsideMot[i]].getPosition();
                        diff0 = ang0 - squatInsideRef;
                        sign0 = (diff0 >= 0)? 1:-1;
                        diff1 = ang1 - squatOutsideRef;
                        sign1 = (diff1 >= 0)? 1:-1;
                        // gradually change to squat position
                        if (abs(diff0) > 0.5 && squatReady == 0){
                            squatInsideAng = ang0 - 0.07*(sign0);
                        } else if (abs(diff1) > 0.2&& squatReady == 0){
                            squatInsideAng = ang0 - 0.05*(sign0);
                        } else{
                            squatInsideAng = squatInsideRef;
                            if (squatReady == 0) {squatReady = 1;}
                        }
                        if (abs(diff1) > 0.5 && squatReady == 0){
                            squatOutsideAng = ang1 - 0.07*(sign1);
                        } else if (abs(diff1) > 0.2 && squatReady == 0){
                            squatOutsideAng = ang1 - 0.05*(sign1);
                        } else{
                            squatOutsideAng = squatOutsideRef;
                            if (squatReady == 0) {squatReady = 1;}
                        }*/
                        if (i<2){
                            joint[frontInMot[i]].setGain(0.7, 0.015);
                            joint[frontInMot[i]].setPosition(squatInsideAng);
                            joint[frontOutMot[i]].setGain(0.7,0.015);
                            joint[frontOutMot[i]].setPosition(squatOutsideAng);
                        }

                        joint[outsideMot[i]].setGain(0.7,0.015);
                        joint[outsideMot[i]].setPosition(squatOutsideAng);

                        if (useTail){
                            if (tailPos < PI/4){
                                joint[8].setGain(0.1, 0.006);
                                joint[8].setPosition(PI/2); //prepare tail
                            } else if (tailPos > PI/2){
                                joint[8].setOpenLoop(-0.3);
                            }else{
                                tau = 0.15-0.03*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                                joint[8].setOpenLoop(tau);
                            }
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
                start flicking tail
                wait for some small time
                set position on leg joints
                want to have it so that robot is airborne when tail is upright
                */
                    if (extendPrep ==0){
                        tExtend = tCurrent;
                        extendPrep = 1;
                    }

                    
                    if (tCurrent-tExtend > whipTime){
                        // use setOpenLoop(1) to give maximum power
                        for (int i = 0; i<4; ++i){
                        joint[frontMot[i]].setOpenLoop(1);
                        joint[backMot[i]].setOpenLoop(0.6);
                        }
                    }
                    
                    // tail
                    if (useTail){
                        if (tailPos < 0){
                            joint[8].setOpenLoop(0);
                        } else{
                            joint[8].setOpenLoop(-0.7); //make sure the tail swings upwards first
                        }
                        
                    }
                    

                    float q0,q1;
                    for (int i = 0; i<4; ++i){
                        q0 = joint[insideMot[i]].getPosition();
                        q1 = -joint[outsideMot[i]].getPosition();
                        float diff = q1-q0;
                        if (diff<0){diff = diff + 2*PI;}
                        if (diff < 0.40){
                            landPrep = 0;
                            extendPrep = 0;
                            mode = CONTRACT;
                            break;
                        } 
                    }
                    if (tCurrent - tExtend - whipTime > 400){
                            landPrep = 0;
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
                    
                    for (int i = 0; i<4; ++i){
                        joint[insideMot[i]].setGain(0.5, 0.008);
                        joint[insideMot[i]].setPosition(contractInAng);

                        joint[outsideMot[i]].setGain(0.5,0.008);
                        joint[outsideMot[i]].setPosition(contractOutAng);

                        if (almostEq(joint[insideMot[i]].getPosition(), contractInAng)){
                            mode = LAND;
                            landPrep = 0;
                            break;
                        }
                    }
                    if (tCurrent - tContract > 500){
                        mode = LAND;
                        contractPrep = 0;
                    }
                    

                break;

                case LAND:
                    static int counter = 0;
                    ac_x[counter] = S->imu.linear_acceleration.x;
                    forwardAccel = avg(ac_x,filterSize);
                    ac_z[counter] = S->imu.linear_acceleration.z;
                    upAccel = avg(ac_z,filterSize);
                    ang0 = joint[0].getPosition();
                    ang1 = joint[1].getPosition();
                    ang4 = joint[4].getPosition();
                    ang5 = joint[5].getPosition();
                    leftFrontAng = (ang0-ang1)/2.0;
                    rightFrontAng = (ang4-ang5)/2.0;
                    frontAng = (leftFrontAng+rightFrontAng)/2.0;
                    if (landPrep == 0 && almostEq(ang0, landOutFrontAng) && almostEq(ang1, landInFrontAng)){
                        landPrep = 1;
                    }
                    if (landPrep == 1 && upAccel <= -18.0 && forwardAccel < -2.5){
                        hasLanded = 1;
                    }
                    if (hasLanded && frontAng > -0.05){ 
                        isStable = 1;
                    }
                    if (isStable){ // switch to regular stand after landing stabilizes
                        for (int i = 0; i<4; ++i){
                            joint[insideMot[i]].setGain(0.55, 0.01);
                            joint[insideMot[i]].setPosition(standInAng);

                            joint[outsideMot[i]].setGain(0.55, 0.01);
                            joint[outsideMot[i]].setPosition(standOutAng);
                        }
                        if (useTail){ // don't do much with the tail after stable
                            joint[8].setGain(0.4, 0.01);
                            joint[8].setPosition(0);
                            //joint[8].setOpenLoop(0); //for debugging
                        }
                    } else {
                        for (int i=0;i<2;i++){
                            joint[frontInMot[i]].setGain(0.6, 0.011);
                            joint[frontInMot[i]].setPosition(landInFrontAng);
                            joint[frontOutMot[i]].setGain(0.6,0.011);
                            joint[frontOutMot[i]].setPosition(landOutFrontAng);

                            joint[rearInMot[i]].setGain(0.6, 0.011);
                            joint[rearInMot[i]].setPosition(landInRearAng);
                            joint[rearOutMot[i]].setGain(0.6, 0.011);
                            joint[rearOutMot[i]].setPosition(landOutRearAng);
                        }
                        if (useTail && !hasLanded){ // continue what contract was doing
                            if (tailPos > (-PI/4)){ // not yet near limit
                                joint[8].setOpenLoop(-0.5);
                            } else if (tailPos < (-PI/2)){ // past limit
                                joint[8].setOpenLoop(0.3);
                            }
                            else{ // near limit
                                tau = -0.1-0.05*logf((tailPos+PI/2)/(PI/2-tailPos))-0.02*tailVel;
                                joint[8].setOpenLoop(tau);
                            }
                        } else if (useTail && hasLanded){ //try to recover
                            joint[8].setGain(0.4,0.01);
                            joint[8].setPosition(0);
                            
                        }

                    }
                    counter = (counter+1)%filterSize;
                    
                break;
            }

        }
        // Stand when STOP
        else if (C->behavior.mode == BehaviorMode_STOP){
            hasLanded = 0;
            isStable = 0;
            standPrep = 0;
            squatPrep = 0;
            squatReady = 0;
            contractPrep = 0;
            landPrep = 0;
            extendPrep = 0;
            for (int i = 0; i<4; ++i){
                joint[insideMot[i]].setGain(0.42, 0.006);
                joint[insideMot[i]].setPosition(standInAng);

                joint[outsideMot[i]].setGain(0.42, 0.006);
                joint[outsideMot[i]].setPosition(standOutAng);
            }

            //joint[8].setOpenLoop(0);
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
            }
            /*
            if (tailPos > (-PI/4)){ // not yet near limit
                joint[8].setGain(0.4, 0.02);
                joint[8].setPosition(-PI/2);
            } else if (tailPos < (-PI/2)){
                joint[8].setOpenLoop(0.4);
            }
            else{
                tau = -0.2-0.05*logf((tailPos+PI/2)/(PI/2-tailPos));
                joint[8].setOpenLoop(tau);
            }*/
            
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
    /*
    for(int i = 0; i<8; i++){
        float pos = joint[i].getPosition();
        printf("Motor %d: %f\t", i, pos);
        if (i==3){printf("\n");}
    }*/

    
    //printf("Tail: %f\n", joint[8].getPosition());
    float q0 = joint[6].getPosition();
    float q1 = -joint[7].getPosition();
    float squatInsideRef = PI/2;
    float squatOutsideRef = PI/4;
    float diff = q1-q0;
    if (diff<0){diff = diff + 2*PI;}
    float a_x = S->imu.linear_acceleration.x;
    float a_y = S->imu.linear_acceleration.y;
    float a_z = S->imu.linear_acceleration.z;
    float ang0 = joint[0].getPosition();
    float ang1 = joint[1].getPosition();
    float ang4 = joint[4].getPosition();
    float ang5 = joint[5].getPosition();
    float leftFrontAng = (ang0-ang1)/2.0;
    float rightFrontAng = (ang4-ang5)/2.0;
    float frontAng = (leftFrontAng+rightFrontAng)/2.0;
    float tailPos = joint[8].getPosition();
    float tailVel = joint[8].getVelocity();
    
    //printf("q0 = %f\t -q1 = %f\t, diff = %f\n",q0,-q1,diff);
    //printf("X: %f\t Y:%f\t Z: %f\t\n", a_x, a_y, a_z);
    //printf("4 = %f\t5=%f\t, insideRef = %f\t, outsideRef = %f\n", ang4, ang5, squatInsideRef, squatOutsideRef);
    // printf("diff 4 = %f\t diff 5 = %f\n", ang4-squatInsideRef, ang5-squatOutsideRef);
    printf("tail = %f\tv = %f\tupAccel = %f\tforwardAccel = %f\n", tailPos, tailVel, a_z, a_x);
    
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


    // Uncomment to clear Bound and Walk behaviors
    behaviors.clear();

    // Create, add, and start Starter behavior
    Starter starter;
    behaviors.push_back(&starter);
    starter.begin();
    setDebugRate(100);
        
    // Run
    return begin();
}
