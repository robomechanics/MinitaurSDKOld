/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef SDK_h
#define SDK_h

#include "simple.pb.h"
//#include "MCUClass.h"
#include "DebugOutput.h"
#include "SMath.h"

#include "Peripheral.h"
#include "Behavior.h"
// #include "JoyBase.h"

#include <vector>
#if defined(_MSC_VER)
typedef int64_t useconds_t;
#else
#include <sys/types.h>//for useconds_t
#endif

/** @addtogroup SDK Basic SDK operations
 *  @{
 */

/**
 * Defined separately for each platform, in units of Hz
 */
extern const int CONTROL_RATE;

/**
 * Pointer to the global RobotState
 */
extern RobotState *S;
/**
 * Pointer to the global RobotCommand
 */
extern RobotCommand *C;
/**
 * Pointer to the global RobotParams
 */
extern RobotParams *P;

// for _write, special FILE*
//#define LOGGER_FILENO								3

/**
 * @brief Value of BehaviorCmd::mode that should stop the behavior
 */
#define BehaviorMode_STOP		(0)
/**
 * @brief Value of BehaviorCmd::mode that should start the behavior. Higher values can be customized for each behavior
 */
#define BehaviorMode_RUN		(1)

/**
 * @brief Globally defined vector of behavior (pointer)s that the user can append their own behaviors to
 * @details By default, this comes prepopulated with some number of behaviors (e.g. bound and walk for Minitaur; subject to grow)
 */
extern std::vector< Behavior *> behaviors;

/**
 * @brief Low-level control time (in microseconds)
 */
extern uint32_t llControlTime;

/**
 * @brief Initialize the SDK for a particular robot platform
 * 
 * @param type Robot type (see RobotParams::Type)
 * @param argc Number of command line arguments (pass from main())
 * @param argv Command line arguments (pass from main())
 * @return True if succeeded
 */
bool init(RobotParams_Type type, int argc, char *argv[]);

/**
 * @brief Commences the various control loops and tasks.
 * @details This should be the *last* function the user calls at the end of main. It never returns.
 * @return Never returns
 */
int begin();


// Native handles to platforms
//FIXME another solution is some kind of interface, but this seems more flexible
// interface needs: init(),
//MCUClass * mcu();

/**
 * @brief Add to list of peripherals
 */
void addPeripheral(Peripheral *);

/**
 * @brief Enable or disable the safety shutoff heuristic. Leave on for a robot; can turn off for testing individual joints
 * @param flag True to enable
 */
void safetyShutoffEnable(bool flag);
/**
 * @brief Enable or disable the soft start init procedure
 * @details This behavior initializes the robot limbs in a way that avoid self-intersection and positions them for nominal operation when the robot first turns on
 * 
 * @param flag True to enable
 */
void softStartEnable(bool flag);

void * obc();

void * sim();

/**
 * @brief Return handle to Joystick used on the MCU
 */
// JoyBase * joystick();


//#if defined(ARM_MATH_CM4)
/**
 * @brief Microsecond sleep function. 
 * @details **Warning:** This kind of delay *must not* be used in Behavior::update(). This function
 * works the same in Unix-like systems and the MCU.
 * @param us Delay duration in microseconds
 */
extern "C" int usleep(useconds_t us);

// TODO eventually the internal implementations should also use these
#if !(defined(ARM_MATH_CM4) || defined(ARCH_obc))
/**
 * @brief Actuation update (critical, hard real-time)
 * @details Should be called by the implementation
 */
int sdkUpdateA();
/**
 * @brief Behavior and other task update (less critical, but still should be called as often as possition)
 * @details Should be called by the implementation
 */
int sdkUpdateB();
#endif

/** @} */ // end of addtogroup


/** @addtogroup IO Low-level input/output with file descriptors
 *  @{
 */

/**
 * Enum to store file descriptors on the MCU (for use with ioctl, write, read, ...).
 * unistd.h defines STDIN=0, STDOUT=1, STDERR=2. See https://en.wikipedia.org/wiki/File_descriptor
 * On the MCU all three of these point to the USB programming port; on the computer the
 */
enum MCUFD {
	/**
	 * Control the used LED on the mainboard (command > 0 turns it on)
	 */
	LED_USER_FILENO=3,
	/**
	 * Control the 12V/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_12V1_FILENO,
	/**
	 * Control the 24V/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_24V1_FILENO,
	/**
	 * Control the V+/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_VIN1_FILENO,
	/**
	 * Control the Serial2 port (refer to MCU-specific information in the documentation)
	 */
	SERIAL2_FILENO,
	/**
	 * Control the user SPI port (refer to MCU-specific information in the documentation)
	 */
	SPI_USER_FILENO,
	/**
	 * Control the user I2C port (refer to MCU-specific information in the documentation)
	 */
	I2C_USER_FILENO,
	/**
	 * Control the logger (refer to MCU-specific information in the documentation). Use write() to
	 * set some user-specified data for logging. As an example, if you have attached a user
	 * peripheral (such as an added sensor), you can append its readings to the logged data.
	 * The maximum size of this data is 32 bytes at the moment.
	 *
	 * Use ioctl() to control whether the onboard microSD card (when running on the MCU) is
	 * logging data (1 starts, 0 stops).
	 */
	LOGGER_FILENO,
	/**
	 * Control joystick sensitivities using ioctl()
	 */
	JOYSTICK_FILENO,
	/**
	 * Control ADC's for analog measurements
	 */
	ADC_FILENO,
};

/**
 * @brief MCU hardware device control function
 * @details This is meant to emulate userspace ioctl on Unix devices. For more information,
 * see https://www.gnu.org/software/libc/manual/html_node/IOCTLs.html or http://www.makelinux.net/ldd3/chp-6-sect-1.
 * 
 * At this point only digital I/O is represented, but in the future ADC, SPI, I2C, etc. will be added
 *
 * @param filedes An element of enum MCUFD, or STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO
 * @param command Command to send device (see MCUFD for device-specific effect)
 * @param args Optional argument to command (usually a struct)
 *
 * @return [description]
 */
extern "C" int ioctl(int filedes, int command, void *args = NULL);
//#endif

/**
 * For some ioctl() files (see MCUFD) such as I2C, the cmd can be to read or write. Use
 * cmd = IOCTL_CMD_RD to read.
 */
#define IOCTL_CMD_RD	(0)
/**
 * For some ioctl() files (see MCUFD) such as I2C, the cmd can be to read or write. Use
 * cmd = IOCTL_CMD_WR to write.
 */
#define IOCTL_CMD_WR	(1)
/**
 * Set joystick sensitivity. args is a float[2] = {speed_sens, yaw_sens}
 */
#define IOCTL_CMD_JOYSTICK_SET_SENS	(0)
/**
 * Set joystick type to one of the enum JoyType
 */
#define IOCTL_CMD_JOYSTICK_SET_TYPE	(1)
/**
 * Read an ADC pin (args is a {uint16_t=pin, uint16_t=result} tuple)
 */
#define IOCTL_CMD_ADC_READ	(0)

/** @} */ // end of addtogroup


/** @addtogroup Debug Debugging support
 *  @{
 */

/**
 * @brief Set the rate at which the debug() user function is called
 * @param hz Rate in Hz
 */
void setDebugRate(int hz);

/** @} */ // end of addtogroup



#endif
