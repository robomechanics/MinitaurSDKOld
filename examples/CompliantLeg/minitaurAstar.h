#include <queue>
#include <vector>
#include <math.h>
#include <SMath.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

class minitaurAstar
{
private:
	double *Q1; //list of motor 1 angular position
	double *Q2; //list of motor 2 angular position
	int sizeQ1;
	int sizeQ2;
	double l1;
	double l2;
	double lf;
	double w;
	int numNeighbors = 8;
	int neighborQ1[8] = {1,1,0,-1,-1,-1,0,1};
	int neighborQ2[8] = {0,-1,-1,-1,0,1,1,1};

public:
	minitaurAstar(double *_Q1, double *_Q2, int _sizeQ1, int _sizeQ2, double _l1, double _l2, double _lf, double _w);
	vector<int> search(double startQ1, double startQ2, double goalQ1, double goalQ2,double a, double b);
	vector<int> searchXY(double startQ1, double startQ2, double goalX, double goalY,double a,double b);
	vector<int> backTrace(int lastIndex,int startIndex, vector<int>* parents);
	double heuristic(int q1Index,int q2Index, int goalQ1Index, int goalQ2Index);
	double distance(double x1,double y1, double x2, double y2);
	int QindexToIndex(int q1Index, int q2Index);
	int QtoIndex(double q1, double q2);
	void indexToQindex(int index, int &q1Index, int &q2Index);
	void indexToQ(int index, double &q1, double &q2);
	void forwardKinematics(double q1, double q2,double &toeX, double &toeY, 
		double &x1, double &y1 ,double &x2, double &y2 ,double &x3, double &y3, double &x4, double &y4);
	void inverseKinematics(double toeX, double toeY, double &q1, double &q2);
	bool legCollisionCheck(double x1, double y1 ,double x2, double y2 ,double x3, double y3, double x4, double y4,double a, double b);
	bool lineCollisionCheck(double x1,double y1, double x2, double y2,double a, double b);
}; 