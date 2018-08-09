#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

void debug() {
    // printf("%u\t%.2f\n", S->millis, S->imu.euler.z);
    // printf("%u\t%.2f\n", S->millis, joint[1].getRawPosition());
    printf("%.4f\n", S->millis, joint[1].getRawPosition());
}

int main(int argc, char * argv[]) {
    init(RobotParams_Type_MINITAUR, argc, argv);
    setDebugRate(10);
    return begin();
}