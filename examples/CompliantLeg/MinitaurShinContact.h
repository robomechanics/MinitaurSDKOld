#include <math.h>
#include <SMath.h>
using namespace std;

class MinitaurShinContact
{
private:
	float l1;
	float l2;
	float w;
public:
	MinitaurShinContact(float _l1, float _l2, float _w);
	double findLc(double q1,double q2,double q1Dot,double q2Dot);
	void LcToXY(double q1,double q2, double lc,float &x, float &y);
	void SPCM(double q1,double q2,double q1Hat,double q2Hat,float &xResult, float &yResult);
	void qsToXYshin(double q1,double q2, double &x1, double &y1,double &x2, double &y2);
};