#include "minitaurAstar.h"
#define PI 3.141592654
#define highCostMultiplier 1000

struct node{
	int index;
	double f;
};

class fCompare
{
	public:
	bool operator() (const node& lhs, const node& rhs) const
	{
		return ((lhs.f) > (rhs.f));
	}
};

minitaurAstar::minitaurAstar(double *_Q1, double *_Q2, int _sizeQ1, int _sizeQ2, double _l1, double _l2, double _lf, double _w)
{
	Q1 = _Q1;
	Q2 = _Q2;
	sizeQ1 = _sizeQ1;
	sizeQ2 = _sizeQ2;
	l1 = _l1;
	l2 = _l2;
	lf = _lf;
	w = _w;
}

vector<int> minitaurAstar::search(double startQ1, double startQ2, double goalQ1, double goalQ2,double a,double b)
{
	int thisQ1Index, thisQ2Index;
	int startIndex = QtoIndex(startQ1,startQ2);
	int goalIndex = QtoIndex(goalQ1,goalQ2);
	int goalQ1Index, goalQ2Index;
	indexToQindex(goalIndex, goalQ1Index, goalQ2Index);
	double goalX,goalY,toeX,toeY,x1,y1,x2,y2,x3,y3,x4,y4;
	forwardKinematics(goalQ1,goalQ2,goalX,goalY,x1,y1,x2,y2,x3,y3,x4,y4);
	vector<int> parents;
	vector<double> g;
	vector<bool> CLOSED;
	for(int i = 0; i <= sizeQ1*sizeQ2; i++)
	{
		g.push_back(-1);
		CLOSED.push_back(false);
		parents.push_back(-1);
	}
	g[startIndex] = 0;
	parents[startIndex] = startIndex;
	priority_queue<node,vector<node>,fCompare> OPEN;
	node toOPEN;
	toOPEN.index = startIndex;
	indexToQindex(startIndex, thisQ1Index, thisQ2Index);
	//toOPEN.f = g[startIndex] + heuristic(Q1[thisQ1Index],Q2[thisQ2Index],Q1[goalQ1Index],Q2[goalQ2Index]);
	forwardKinematics(startQ1,startQ2,toeX,toeY,x1,y1,x2,y2,x3,y3,x4,y4);
	toOPEN.f = g[startIndex] + heuristic(toeX,toeY,goalX,goalY);
	OPEN.push(toOPEN);


	while ((OPEN.top().index!=goalIndex) && !OPEN.empty())
	{
		// put into CLOSED
		int thisIndex = OPEN.top().index;
		CLOSED[thisIndex] = true;
		OPEN.pop();
		// loop through neighbors
		for(int i = 0; i < numNeighbors; i++)
		{
			indexToQindex(thisIndex, thisQ1Index, thisQ2Index);
			int newQ1Index = thisQ1Index + neighborQ1[i];
			int newQ2Index = thisQ2Index + neighborQ2[i];
			if (newQ1Index < sizeQ1 && newQ1Index >= 0 && newQ2Index < sizeQ2 && newQ2Index >=0)
			{
				//double toeX,toeY,x1,y1,x2,y2,x3,y3,x4,y4;
				forwardKinematics(Q1[newQ1Index],Q2[newQ2Index],toeX,toeY,x1,y1,x2,y2,x3,y3,x4,y4);
				double newg;
				if (legCollisionCheck(x1,y1,x2,y2,x3,y3,x4,y4,a,b))
				{
					newg = g[thisIndex] + distance(toeX,toeY,goalX,goalY)*highCostMultiplier;
					//newg = g[thisIndex] + distance(Q1[newQ1Index],Q2[newQ2Index],Q1[thisQ1Index],Q2[thisQ2Index])*highCostMultiplier;
				}
				else
				{
					newg = g[thisIndex] + distance(toeX,toeY,goalX,goalY);
					//newg = g[thisIndex] + distance(Q1[newQ1Index],Q2[newQ2Index],Q1[thisQ1Index],Q2[thisQ2Index]);
				}
				int newIndex = QindexToIndex(newQ1Index, newQ2Index);
				if ((newg < g[newIndex] || g[newIndex] < 0) && !CLOSED[newIndex])
				{
					g[newIndex] = newg;
					parents[newIndex] = thisIndex;
					toOPEN.index = newIndex;
					//toOPEN.f = g[newIndex] + heuristic(Q1[newQ1Index],Q2[newQ2Index],Q1[goalQ1Index],Q2[goalQ2Index]);
					toOPEN.f = g[newIndex] + heuristic(toeX,toeY,goalX,goalY);
					OPEN.push(toOPEN);
				}
				/*
				if (!legCollisionCheck(x1,y1,x2,y2,x3,y3,x4,y4,a,b))
				{
					int newIndex = QindexToIndex(newQ1Index, newQ2Index);
					double newg = g[thisIndex] + distance(newQ1Index,newQ2Index,thisQ1Index,thisQ2Index);
					if ((newg < g[newIndex] || g[newIndex] < 0) && !CLOSED[newIndex])
					{
						g[newIndex] = newg;
						parents[newIndex] = thisIndex;
						toOPEN.index = newIndex;
						toOPEN.f = g[newIndex] + heuristic(newQ1Index,newQ2Index,goalQ1Index,goalQ2Index);
						OPEN.push(toOPEN);
					}
				}*/
			}
		}
	}
	if (OPEN.top().index == goalIndex)
	{
		return backTrace(OPEN.top().index,startIndex,&parents);
	}
	else
	{
		vector<int> noPath;
		noPath.push_back(startIndex);
		return noPath;
	}
}

vector<int> minitaurAstar::searchXY(double startQ1, double startQ2, double goalX, double goalY,double a,double b)
{
	double goalQ1,goalQ2;
	inverseKinematics(goalX, goalY, goalQ1, goalQ2);
	return search(startQ1,startQ2,goalQ1,goalQ2,a,b);
}

vector<int> minitaurAstar::backTrace(int lastIndex,int startIndex, vector<int>* parents)
{
	vector<int> path;
	int tempIndex = lastIndex;
	while (tempIndex != startIndex)
	{
		path.insert(path.begin(),tempIndex);
		tempIndex = (*parents)[tempIndex];
	}
	path.insert(path.begin(),startIndex);
	return path;
}

double minitaurAstar::heuristic(int q1Index,int q2Index, int goalQ1Index, int goalQ2Index)
{
	return distance(q1Index,q2Index,goalQ1Index,goalQ2Index);
}

double minitaurAstar::distance(double x1,double y1, double x2, double y2)
{
	double diff1 = x1-x2;
	double diff2 = y1-y2;
	return sqrt(diff1*diff1 + diff2*diff2);
	//return max(abs(diff1),abs(diff2));
}

int minitaurAstar::QindexToIndex(int q1Index, int q2Index)
{
	return sizeQ1*q2Index+q1Index;
}

int minitaurAstar::QtoIndex(double q1, double q2)
{
	int i = 0;
	while ((Q1[i] < q1) && (i+1 < sizeQ1))
		i++;
	if (i > 0)
	{
		if (fabs(Q1[i] - q1) > fabs(q1 - Q1[i-1]))
			i = i-1;
	}
	int j = 0;
	while ((Q2[j] < q2) && (j+1 < sizeQ2))
		j++;
	if (j > 0)
	{
		if (fabs(Q2[j] - q2) > fabs(q2 - Q2[j-1]))
			j = j-1;
	}
	return QindexToIndex(i,j);
}

void minitaurAstar::indexToQindex(int index, int &q1Index, int &q2Index)
{
	q1Index = index%sizeQ1;
	q2Index = floor(index/sizeQ1);
}

void minitaurAstar::indexToQ(int index, double &q1, double &q2)
{
	int q1Index,q2Index;
	indexToQindex(index,q1Index,q2Index);
	q1 = Q1[q1Index];
	q2 = Q2[q2Index];
}

void minitaurAstar::forwardKinematics(double q1, double q2,double &toeX, double &toeY, 
	double &x1, double &y1 ,double &x2, double &y2 ,double &x3, double &y3, double &x4, double &y4)
{
	double t1 = 2*PI-q1-q2;
	double l3 = 2*l1*sin(t1/2);
	double t2 = 2*asin(l3/2/l2);
	double t3 = (2*PI-t1-t2)/2;
	double t4 = q1-t3+PI/2;
	//double t5 = -q2-PI/2+t3;

	x1 = 0; y1 = 0;
	x2 = x1 + l1*sin(q1);
	y2 = y1 + l1*cos(q1);
	x3 = x1 - l1*sin(q2);
	y3 = y1 + l1*cos(q2);
	x4 = x2 + l2*cos(t4);
	y4 = y2 - l2*sin(t4);

	toeX = x2 + (l2+lf)*cos(t4);
	toeY = y2 - (l2+lf)*sin(t4);
}

void minitaurAstar::inverseKinematics(double toeX, double toeY, double &q1, double &q2)
{
	int numTests = 1000;
	double jointX,jointY,r,theta,t1,thisQ1,thisQ2,thisToeX,thisToeY, minDistSquared, thisDistSquared;
	double x1,y1,x2,y2,x3,y3,x4,y4;
	for (int i = 0; i < numTests; i++)
	{
		jointX = toeX + lf*cos(i*2*PI/numTests);
		jointY = toeY + lf*sin(i*2*PI/numTests);
		r = sqrt(jointX*jointX+jointY*jointY);
		theta = atan2(jointX,jointY);
		t1 = 2*acos((l1*l1+r*r-l2*l2)/(2*l1*r));
		thisQ1 = theta-t1/2;
		thisQ2 = 2*PI-q1-t1;
		forwardKinematics(thisQ1, thisQ2,thisToeX, thisToeY, x1,y1,x2,y2,x3,y3,x4,y4);
		thisDistSquared = (thisToeX - toeX)*(thisToeX - toeX) + (thisToeY - toeY)*(thisToeY - toeY);
		if ((thisDistSquared < minDistSquared) || (i==0))
		{
			minDistSquared = thisDistSquared;
			q1 = thisQ1;
			q2 = thisQ2;
		}
	}
}

bool minitaurAstar::legCollisionCheck(double x1, double y1 ,double x2, double y2 ,double x3, double y3, double x4, double y4,double a, double b)
{
	// fist link
	double theta = atan2(y2-y1,x2-x1);
	double xp1 = x1 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	double yp1 = y1 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	double xp2 = x1 - w/2*cos(theta) + w/2*cos(theta-PI/2);
	double yp2 = y1 - w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x2 + w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp1 = y2 + w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp2 = x2 + w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp2 = y2 + w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x1 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y1 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	// second link
	theta = atan2(y3-y1,x3-x1);
	xp1 = x1 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y1 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	xp2 = x1 - w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp2 = y1 - w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x3 + w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp1 = y3 + w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp2 = x3 + w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp2 = y3 + w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x1 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y1 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;

	// third link
	theta = atan2(y4-y2,x4-x2);
	xp1 = x2 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y2 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	xp2 = x2 - w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp2 = y2 - w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x4 + lf*cos(theta) + w/2*cos(theta-PI/2);
	yp1 = y4 + lf*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp2 = x4 + lf*cos(theta) + w/2*cos(theta+PI/2);
	yp2 = y4 + lf*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x2 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y2 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;

	// fourth link
	theta = atan2(y4-y3,x4-x3);
	xp1 = x3 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y3 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	xp2 = x3 - w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp2 = y3 - w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x4 + w/2*cos(theta) + w/2*cos(theta-PI/2);
	yp1 = y4 + w/2*sin(theta) + w/2*sin(theta-PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp2 = x4 + w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp2 = y4 + w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	xp1 = x3 - w/2*cos(theta) + w/2*cos(theta+PI/2);
	yp1 = y3 - w/2*sin(theta) + w/2*sin(theta+PI/2);
	if (lineCollisionCheck(xp1,yp1,xp2,yp2,a,b))
		return true;
	return false;
}


bool minitaurAstar::lineCollisionCheck(double x1,double y1, double x2, double y2,double a, double b)
{
	if ((x1 >= a && y1 <= b) || (x2 >= a && y2 <=b))
	{
    	return true;
	}
	if (x1 <= a && y1 >= b)
	{
	    return false;
	}
	else if (x1 < a)
	{
		if (y2 <= b)
		{
	        return false;
		}
	    else if (y2 > (b-y1)/(a-x1)*(x2-x1)+y1)
	    {
	        return false;
	    }
	    else
	    {
	        return true;
	    }
	}
	else
	{
	    if (x2 >= a)
	    {
	        return false;
	    }
	    else if (y2 > (b-y1)/(a-x1)*(x2-x1)+y1)
	        return false;
	    else
	    {
	        return true;
	    }
	}
}