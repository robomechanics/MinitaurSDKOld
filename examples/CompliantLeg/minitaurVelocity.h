#include <math.h>
#include <vector>
#include <Motor.h>
#include <ReorientableBehavior.h>
using namespace std;

class minitaurVelocity
{
private:
	int numMotors = 8;
	int numSamplesPos = 10;
	int numSamplesVel = 25;
	int indexPos = 0;
	int indexVel = 0;
	bool posFilled = false;
	bool velFilled = false;
	vector<uint32_t> tRecord;
	vector<float> posRecord;
	vector<float> velRecord;
	void initVectors();
	float median(vector<float> *vec,int start,int length);
public:
	minitaurVelocity();
	void init();
	void init(int numMotors);
	void init(int _numMotors,int _numSamplesPos, int _numSamplesVel);
	void updateVelocity();
	vector<float> filteredVel;
};