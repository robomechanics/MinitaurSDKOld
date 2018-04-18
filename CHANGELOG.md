# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

The version numbers are referenced to the git tags.

## [Unreleased]

## [0.1.17] - 2018-04-16

### Added
- Better kinematic velocity estimate (accounts for body twist)
- Improved bound initialization, leg angle offsets, turning control
- Improved walk leg angle offsets, higher damping, speedDes rate limiter
- CommandOverEthernet example custom state copy callback

## Fixes
- PWM duty = -1 bug on grble0.1
- CommandOverEthernet example error checking

## [0.1.16] - 2018-04-04

### Added
- CommandOverEthernet example
- Ethernet support through file I/O

### Fixed
- Joystick range (command limiter) bug fixed

## [0.1.15] - 2018-04-02

### Added
- Aux serial port access for users

### Fixed
- Position unwrap bug in high-gear-ratio instances

## [0.1.14] - 2018-03-15

### Added
- ReadRobot example (faster logging using Peripheral)

### Fixed
- Bad static libs

## [0.1.13] - 2018-03-14

### Added
- Microsecond clock time on the microcontroller
- Ability to configure serial port settings for the programming micro USB port
- Other internal changes not ready for release

## [0.1.12] - 2018-02-26

### Added
- Support and documentation for user I2C
- Support and documentation for user digital I/O

## [0.1.11] - 2018-02-13
### Added
- Full support for joysticks in simulation
- Gait work for NGR
- Support for Minitaur E new variant with upgraded electronics
- CI-built ARM libs
- Full support for Taranis Q X7 joystick

## [0.1.10] - 2018-01-25
### Fixed
- microsecond counter overflow bug causing behavior updates to stop after 71 minutes

### Added
- support for gamepad joysticks in simulations (partial) #38
- support for Dynamixel protocol 1 (partial)
- support for Dynamixel protocol 2 (full)
- support for async Dynamixel update in PWM or POSITION modes #31
- posture controller support for 2DOF legs
- posture controller support for leg pairs with 3DOF legs
- S.bus driver full support
- ioctl support for body LED lighting
- ioctl support for switching radio receiver
- Gait work in progress

## [0.1.9] - 2017-12-14
### Added
- posture controller (internal)
- sim implementation (internal)
- ROS translation updates
- various other updates

## [0.1.8] - 2017-11-29
### Fixed
- Better VN100 parameters, switched to hardware filter

### Added
- improved urdf (internal)
- compiles with Visual C++ compiler
- Sbus driver partial #20
- SmartPort driver partial #21
- "None" architecture external physics engine support partial
- PWM FrSKY support (Taranis Q7)
- Lowpass for pose z control using the remote
- Slightly increased default yaw sensitivity

## [0.1.7] - 2107-11-21
### Fixed
- issue #17 - may interfere with mbm (to test)

## [0.1.6] - 2017-11-15
### Fixed
- updated FirstHop example with analog reading

### Added
- new robot type (internal)
- ioctl can now read analog sensors

### Known issues
- ioctl(ADC_FILENO, ...) cannot be called from Peripheral::update(). Call from debug() for now.

## [0.1.5] - 2017-11-14
### Fixed
- Documentation related to Python installation

### Added
- FrSKY X7 remote option
- way for users to change remote type between init() and begin()
- started b3 integration (internal)
- new urdf (internal)
- JoyType_NONE option so users can supply their own BehaviorCmd
- HexapodGait example: tutorial coming soon

## [0.1.4] - 2017-11-07
### Fixed
- docs image of joystick had axes 2, 3 flipped #11 a6f4109
- some warnings in MCUClass.cpp compilation 314f9a5
- cmake build flags for mbm were actually still wrong; now fixed and tested c229d9b
- JoySerial button mapping from Android app was reversed b499fea

### Added
- Walk minor adaptations on mbm
- Low pass filter for JoySerial inputs from Android app
- SoftStart for Minitaur E

## [0.1.3] - 2017-11-07
### Added
- HexapodGait example and documentation
- Better joystick documentation #11
- More ioctl functionality
- Internal cmake build for mbm, relay mode, eeprom settings

## [0.1.2] - 2017-11-05
### Fixed
- Peripheral::begin() called in begin() #8

### Added
- ioctl initial implementations for joystick, I2C, SPI (alpha)
- Docs update
- internal changes

## [0.1.1] - 2017-11-04
### Added
- Changelog is back (git tag notes won't make it to users)
- Docs update

## [0.1] - 2017-11-04
### Added
- Everything before this
