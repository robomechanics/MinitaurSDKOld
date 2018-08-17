/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <unistd.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

/* 
*
*  Command Robot
*  
*  This example shows you how to command the robot's behavior using a USB 
*  serial link.
*
*  This example uses the higher-level Peripheral rather than a Behavior,
*  so that it is able to make use of the Minitaur's built in walk Behavior.
*  Only one Behavior can run at a time, and any number of Peripherals can
*  run concurrently with the currently running Behavior, so we use this to
*  control the robot by commanding the built in walk behavior.
* 
*  See the SDK docs for more information:
*  http://ghostrobotics.gitlab.io/SDK/index.html
* 
*/

float voltage, current;

#pragma pack(push, 1)
// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.
struct SerialCommandPacket
{
	float voltage, current;
};

const char ALIGNMENT_WORD[2] = {'G', 'R'};

#pragma pack(pop)

class CommandRobot : public Peripheral
{
public:
	void begin()
	{
		// Command by behavior
		C->mode = RobotCommand_Mode_BEHAVIOR;
	}

	// Parser state
	// Goes from 0 to 1 to 2
	int numAlignmentSeen = 0;
	uint16_t rxPtr = 0;

	// Receive buffer and alignment
	const static uint16_t RX_BUF_SIZE = 100;
	SerialCommandPacket packet;

	void update()
	{
		// Character to store the latest received character
		uint8_t latestRX;

		// Loop through while there are new bytes available
		while (read(SERIAL_AUX_FILENO, &latestRX, 1) > 0)
		{
			if (numAlignmentSeen == 0 && latestRX == ALIGNMENT_WORD[0])
			{
				numAlignmentSeen = 1;
			}
			else if (numAlignmentSeen == 1 && latestRX == ALIGNMENT_WORD[1])
			{
				numAlignmentSeen = 2;
				rxPtr = 0;
			}
			else if (numAlignmentSeen == 2)
			{
				// Add the next byte to our memory space in serial_packet
				uint8_t *pSerialPacket = (uint8_t *)&packet;
				pSerialPacket[rxPtr++] = latestRX; // post-increment rxPtr

				// Check if we have read a whole packet
				//printf("ptr %d %d\n", rxPtr, sizeof(SerialPacket));
				if (rxPtr == sizeof(SerialCommandPacket))
				{

					// Copy voltage and current readings
					voltage = packet.voltage;
					current = packet.current;
					//printf("Checksum %u %u.\n", checksum, serial_packet.checksum);

					// Reset
					numAlignmentSeen = rxPtr = 0;
				}
			}
		}
	}
};

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
#else
#error "Define robot type in preprocessor"
#endif

	// Disable joystick input
	JoyType joyType = JoyType_NONE;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	// cfg.baud = 230400;
	cfg.mode = SERIAL_8N1;
	ioctl(SERIAL_AUX_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	// Create controller peripheral
	CommandRobot commandRobot;
	commandRobot.begin();

	setDebugRate(100);

	// Add it
	addPeripheral(&commandRobot);

	// Remove bound behavior from Minitaur (first element of behaviors vector),
	// so we're left with only walk behavior
	behaviors.clear();

	// Go
	return begin();
}

void debug()
{
	char buf[32];
	snprintf(buf, 32, "V:%.5f C:%.5f\n", voltage, current);
	printf("%s", buf);
}