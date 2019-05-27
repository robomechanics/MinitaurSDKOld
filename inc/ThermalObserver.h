#pragma once

#ifndef THERMALOBSERVER_H
#define THERMALOBSERVER_H

class ThermalObserver {
private:

public:
	float R1;
	float C1;
	float R2;
	float C2;

	float dt;
	float tempCore;
	float tempCase;
	float tempAmbient;
	float tempThreshold;

	int motIndex;
	float Ra;
	float kt;
	bool sizzleFlag;

	ThermalObserver() 
	{
		dt = 0.001;						// update rate
		Ra = 0.2;					// winding resistance
		kt = 0.0954;				// motor torque constant
		tempAmbient = 22.0;			// declare ambient temperature
		tempThreshold = 80.0;
		tempCore = tempAmbient;		// initialize core temperature
		tempCase = tempAmbient;		// initialize case temperature
		
		R1 = 0.4907;				// core thermal resistance
		C1 = 88.6299;				// core thermal mass
		R2 = 1.4339;				// case thermal resistance
		C2 = 481.4714;				// case thermal mass
		sizzleFlag = false;
	};

	void assignMotor(int i)
	{
		motIndex = i;
	}

	void update(float current)		// input current
	{
		float I = current*current*Ra;

		float dtempCore = I / C1 + tempCase / (R1*C1) - tempCore / (R1*C1);
		float dtempCase = tempCore / (R1*C2) + tempAmbient / (R2*C2) - tempCase*(1 / (R1*C2) + 1 / (R2*C2));

		tempCore = tempCore + dtempCore * dt;
		tempCase = tempCase + dtempCase * dt;
		
		if ((tempCore >= tempThreshold) || (tempCase >= tempThreshold))
		{
			sizzleFlag = true;
			// for (int i = 0; i < P->joints_count; ++i)
			// {
			// 	C->joints[i].mode = JointMode_OFF;
			// }	
		}
		else {
			sizzleFlag = false;
		}
	}

	void update()
	{
		if (joint[motIndex].getOpenLoopMode() == JointMode_CURRENT)
		{
			update(joint[motIndex].getOpenLoop());
		}
		else
		{
			float current = (S->batt.voltage * joint[motIndex].getOpenLoop() - kt * joint[motIndex].getVelocity()) / Ra;
			update(current);
		}
	}
};


#endif
