/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include "simple.pb.h"

/**
 * @brief Get the MotorModelParams object
 * 
 * @param type 
 * @return const MotorModelParams* 
 */
const MotorModelParams *getMotorModelParams(MotorType type);
/**
 * @brief Get PWM -> torque using a motor model
 * 
 * @param pwm supplied PWM command
 * @param vel velocity of the motor
 * @param params 
 * @return float 
 */
float modelTorqueFromPWM(float pwm, float vel, const MotorModelParams *params);
