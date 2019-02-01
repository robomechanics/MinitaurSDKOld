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

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[9] =  {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

rmlLimb leg[4]; 
//remember to change the makefile if using rml limb!
int state; // used for debugging

enum jumpMode
{
    STAND,
    WAIT,
    LEAP,
    RETRACT,
    TUCKED,
    AB, // airborne
    LANDED,
    POSTLANDED
};

class Starter : public Behavior {
public:
    jumpMode mode = WAIT;
    bool tddetf;
    bool tddetr;
    float tRet;
    float tEventRear;
    float ex0;
    float an0;
    uint32_t tstart;
    uint32_t tCurrent;
    float tEvent;
    float uroll;
    float tarAng;
    float tarAng2;
    float tarExt;
    float tarExt2;
    bool td= false;

    float tFHcomp;
    float fAngPos;
    float trackpullLeft;
    float trackpullRight;
    float tTakeOff;
    float tOnWall;
    float fExtPos;
    float cor;

    // begin() is called once when the behavior starts
    void begin() {

        // Command limbs
        C->mode = RobotCommand_Mode_JOINT;
        mode = WAIT;
        state = -1;
        tstart = S->millis;
        ex0=0;
        an0=0;
        for (int i=0;i<4;++i){
            ex0=ex0+leg[i].getPosition(EXTENSION)/4.0;
            an0 = an0+leg[i].getPosition(ANGLE)/4.0;
        } 
    }

    // signal() is called when receiving a signal from the controller
    // void signal(uint32_t sig) {

    // }

    // update() is called once per loop while the behavior is running
    void update() {
        C->mode = RobotCommand_Mode_JOINT;
        tCurrent = S->millis;
        float legAng = S->imu.euler.y + 0.2;
        for(int i = 0; i<4; ++i)
        {
            P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
            leg[i].updateState();
        }
        // Starter when RUN
        if (C->behavior.mode == BehaviorMode_RUN){
            switch(mode){
                case STAND:
                    if(ex0>0.8){
                    ex0 = ex0-.002;
                    }else{
                        ex0 = 0.8;
                    }

                    if(an0 > legAng+.01){
                        an0 = an0 - .002;
                    }else if(an0<legAng-.01){
                        an0 = an0+.002;
                    }else{
                        an0 = legAng;
                    }

                    for (int i=0; i<4; ++i) {
                        leg[i].setGain(ANGLE,.9);
                        leg[i].setGain(EXTENSION,.3); //was 12
                        leg[i].setPosition(ANGLE,an0);
                        leg[i].setPosition(EXTENSION,ex0);
                    }
                    state = 1;
                break;

                case WAIT:
                    if(ex0>0.8){
                        ex0 = ex0-.002;
                    }else{
                        ex0 = 0.8;
                    }

                    if(an0 > legAng+.05){
                        an0 = an0 - .0005;
                    }else if(an0<legAng-.05){
                        an0 = an0+.0005;
                    }else{
                        an0 = legAng;
                        mode = LEAP;
                    }

                    for (int i=0; i<4; ++i) {
                        leg[i].setGain(ANGLE,.9);
                        leg[i].setGain(EXTENSION,.3); //was 12
                        leg[i].setPosition(ANGLE,an0);
                        leg[i].setPosition(EXTENSION,ex0);
                    }
                    state = 2;
                break;

                case LEAP:
                    leg[0].setOpenLoop(EXTENSION,1);
                    leg[2].setOpenLoop(EXTENSION,1);

                    leg[0].setPosition(ANGLE,legAng);
                    leg[2].setPosition(ANGLE,legAng);
                    

                    //FRONT LEGS
                    //leg[0].setPosition(EXTENSION,.7);
                    //leg[2].setPosition(EXTENSION,.7);
                    //leg[0].setPosition(ANGLE,-1.57);
                    //leg[2].setPosition(ANGLE,-1.57);
                    //HIND LEGS
                    leg[1].setOpenLoop(EXTENSION,0.3);
                    leg[3].setOpenLoop(EXTENSION,0.3);
                    leg[1].setPosition(ANGLE,legAng);
                    leg[3].setPosition(ANGLE,legAng);

                    //TERMINATION
                    if(leg[1].getPosition(EXTENSION)>2.8 || leg[3].getPosition(EXTENSION)>2.8){
                        tarAng2 = leg[1].getPosition(ANGLE);
                        mode = RETRACT;
                        tEvent = tCurrent;
                        
                    }
                    state = 3;
                break;

                case RETRACT:
                    tarAng = -1.5;
                    leg[0].setGain(ANGLE,.1);
                    leg[2].setGain(ANGLE,.1);
                    leg[0].setPosition(EXTENSION,0.8);
                    leg[2].setPosition(EXTENSION,0.8);
                    leg[1].setPosition(EXTENSION,0.6);
                    leg[3].setPosition(EXTENSION,0.6);
                    leg[1].setPosition(ANGLE,tarAng2);
                    leg[3].setPosition(ANGLE,tarAng2);
                    leg[0].setPosition(ANGLE,tarAng2);
                    leg[2].setPosition(ANGLE,tarAng2);
                    if(leg[1].getPosition(EXTENSION)<.64){
                        tEvent = tCurrent;
                        mode = TUCKED; 
                    }
                    state = 4;
                break;

                case TUCKED:
                    leg[1].setPosition(EXTENSION,0.55);
                    leg[3].setPosition(EXTENSION,0.55);
                    leg[1].setPosition(ANGLE,-.8);
                    leg[3].setPosition(ANGLE,-.8);
                    leg[0].setPosition(EXTENSION,0.8);
                    leg[2].setPosition(EXTENSION,0.8);
                    leg[0].setPosition(ANGLE,.4);
                    leg[2].setPosition(ANGLE,.4);
                    // if(leg[0].getPosition(EXTENSION)<.6){
                    //  leg[0].setPosition(ANGLE,.4);
                    //  leg[2].setPosition(ANGLE,.4);
                    // }

                    if(leg[1].getPosition(ANGLE)<-.6){
                        tEvent = tCurrent;
                        mode = AB;
                        tarAng = -0.1;//-.7;
                        tarAng2 = -0.1;//-.7;
                        tarExt = .55;
                        tarExt2 = .2;
                    }
                    state = 5;
                break;

                case AB:
                    if(tCurrent-tEvent<75){
                    //FRONT
                    leg[0].setGain(ANGLE,.5);
                    leg[2].setGain(ANGLE,.5);
                    leg[0].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7); // was -1.2
                    leg[2].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7);               
                    //BACK
                    leg[1].setPosition(EXTENSION,1.4);
                    leg[3].setPosition(EXTENSION,1.4);

                    }else if(tCurrent-tEvent>75 && tCurrent-tEvent<100){
                    leg[0].setGain(ANGLE,.9);
                    leg[2].setGain(ANGLE,.9);

                    leg[0].setPosition(EXTENSION,.9);
                    leg[2].setPosition(EXTENSION,.9);
                    leg[0].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7); // was -1.2
                    leg[2].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7);       
                    // leg[0].setPosition(ANGLE,-0.7);//-.7);
                    // leg[2].setPosition(ANGLE,-0.7);//-.7);
                    }else if(tCurrent-tEvent>100){
                        tEvent = tCurrent;
                        mode = LANDED;
                        tddetf = false;
                        tddetr = false;
                        tarAng = -.8;
                    }
                    state = 6;

                break;

                case LANDED:
                    if(tddetf == true && tddetr == true){
                        tEvent = tCurrent;
                        mode = POSTLANDED;
                    }
                    //Front LEG//////////////////
                    // TD condition:
                    if(leg[0].getPosition(EXTENSION)<.6 || leg[2].getPosition(EXTENSION)<.6){
                        tddetf=true;
                    }
                    //PRIOR
                    if(tddetf==false){
                        leg[0].setPosition(EXTENSION,.9);
                        leg[2].setPosition(EXTENSION,.9);

                        leg[0].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7); // was -1.2
                        leg[2].setPosition(ANGLE,-.5-S->imu.euler.y);//-0.6-S->imu.euler.y);//-.7);       
                    //POST
                    }else{
                        leg[0].setGain(EXTENSION,.6);
                        leg[2].setGain(EXTENSION,.6);
                        leg[0].setPosition(EXTENSION,1.3);
                        leg[2].setPosition(EXTENSION,1.3);
                        leg[0].setPosition(ANGLE,-.4);
                        leg[2].setPosition(ANGLE,-.4);
                    }   
                    //FRONT LEG////////////////////
                    //TD condition
                    if(leg[1].getPosition(EXTENSION)<1.1 || leg[3].getPosition(EXTENSION)<1.1){
                        tddetr = true;
                        tEventRear = tCurrent;
                    }
                    //PRIOR
                    if(tddetr==false){
                        leg[1].setPosition(EXTENSION,1.4);
                        leg[3].setPosition(EXTENSION,1.4);
                        leg[1].setPosition(ANGLE,-0.1);//-.7);
                        leg[3].setPosition(ANGLE,-0.1);//-.7);
                    //POST
                    }else{
                        if(tarAng>.2){
                            tarAng = .2;
                        }else{
                            tarAng=tarAng+PI/400;
                        }
                        leg[1].setPosition(EXTENSION,1.4);
                        leg[3].setPosition(EXTENSION,1.4);
                        leg[1].setPosition(ANGLE,tarAng);
                        leg[3].setPosition(ANGLE,tarAng);
                        if(tCurrent-tEventRear>300){
                            mode = POSTLANDED;
                        }
                    }
                    state = 7;
                break;

                case POSTLANDED:
                    if(ex0>0.8){
                        ex0 = ex0-.002;
                    }else{
                        ex0 = 0.8;
                    }

                    if(an0 > legAng+.01){
                        an0 = an0 - .002;
                    }else if(an0<legAng-.01){
                        an0 = an0+.002;
                    }else{
                        an0 = legAng;
                    }

                    for (int i=0; i<4; ++i) {
                        leg[i].setGain(ANGLE,.9);
                        leg[i].setGain(EXTENSION,.3); //was 12
                        leg[i].setPosition(ANGLE,an0);
                        leg[i].setPosition(EXTENSION,ex0);
                    }
                    if(tEvent-tCurrent>200){
                        mode = STAND;
                    }
                    state = 8;
                break;
            }

        }
        // Stand when STOP
        else if (C->behavior.mode == BehaviorMode_STOP){
            if(ex0>0.8){
                ex0 = ex0-.002;
            }else{
                ex0 = 0.8;
            }

            if(an0 > legAng+.01){
                an0 = an0 - .002;
            }else if(an0<legAng-.01){
                an0 = an0+.002;
            }else{
                an0 = legAng;
            }

            for (int i=0; i<4; ++i) {
                leg[i].setGain(ANGLE,.9);
                leg[i].setGain(EXTENSION,.3); //was 12
                leg[i].setPosition(ANGLE,an0);
                leg[i].setPosition(EXTENSION,ex0);
            }
            state = 0;
        }

        for(int i = 0; i<4; ++i) {
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
    printf("state = %d\n", state);
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
    SerialPortConfig cfg;
    cfg.baud = 115200;
    cfg.mode = SERIAL_8N1;
    ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);
        
    // Run
    return begin();
}
