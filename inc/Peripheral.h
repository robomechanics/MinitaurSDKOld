/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Peripheral_h
#define Peripheral_h

/** @addtogroup Behavior Behavior
 *  @{
 */

//
/**
 * @brief Class for integrating peripheral sensors, tools, etc.
 */
class Peripheral {
public:
  /**
   * @brief Function called to initialize the peripheral
   */
  virtual void begin() = 0;
  /**
   * @brief Function called repeatedly (at CONTROL_RATE) to update the peripheral
   */
  virtual void update() = 0;
};
/** @} */ // end of addtogroup

#endif
