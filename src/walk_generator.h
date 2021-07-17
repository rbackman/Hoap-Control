

#ifndef WALK_GENERATOR
#define WALK_GENERATOR

#include <gsim/sn_group.h>
#include <gsim/sn_model.h>
#include <gsim/sn_manipulator.h>
#include <gsim/gs_model.h>
#include "app_main.h"
#include "utilities.h"
#include "foot.h"
#include "curve.h"
#include <gsim/se_dcdt.h>

class rootFix{
public:
GsVec amt;
int t;
int step;
rootFix(GsVec a,int tm){amt.set(a);t=tm;}
};
#define ROOT_TO_LEFT 0
#define RIGHT_MOVE 1
#define ROOT_TO_RIGHT 2
#define LEFT_MOVE 3
#define WAIT 4
#define START 5
#define STOP 6

class WalkGenerator
{
	public:
		float footH;
int curCv;

	std::vector<Curve*> cvs;
	SnManipulator* _GoalManip;
	SnManipulator* _Goal1Out;
	SnManipulator* _Goal2In;
	GsVec goalLeftOffset,goalRightOffset;
	std::vector<GsPolygon> floorPoly;

	SeDcdt* dcdtobj;
	bool dcdtMade;
	GsPolygon domain;
GsPolygon path;
GsPolygon channel;
int pause;
int resume;
int lastIt;
float lastH;
bool drawTraj;
int curGoal;
bool inc; //continuous play back or stop at each key;
bool run; //for starting and stopping animation
int state; //0:Root->LeftFoot 1: RightFoot->move 2: Root->RightFoot 3:LeftFoot->move move Right 2: 
int prevState; //the state before;
int returnState;  //the state to return to if running incremental

bool stateInit;
int step;		//current step Left and Right
int steps;		//number of steps

int frames;		//number of interpolations per step
int t;			// time specific for single step (ie right moves in t frames)

double startTime,endTime;
double period;
double dt;
bool playing;
float fps;
float rootStartH;	//
float walkDist;		//total distance covered by walk
float angle;	//for turning

float stepHeight;
float h;		//current height
float dh;		//change in height per iteration
int time;		//current time 
int length;	//time duration of clip

GsVec leftRootAdjust,rightRootAdjust,restRootAdjust,allRootAdjust;
GsVec left, nextLeft, prevLeft;
GsVec right,nextRight, prevRight;
GsVec root, prevRoot, nextRoot;
GsVec rootStart,rootEnd;
GsVec leftFootStart,rightFootStart;
float footOffsetDist;
float rootStartAngle;

SnGroup* stepGroup; //group to add models to scenegraph
GsModel* leftfootMod;
GsModel* rightfootMod;
GsModel* rootMod;

std::vector<rootFix*> adjust;
std::vector<Foot*> leftSteps;
std::vector<Foot*> rightSteps;
std::vector<Foot*> rootSteps;
std::vector<FootGoal*> goals;
std::vector<FootGoal*> intGoals;

void init();
void adjustHip(int dir);
void reset();
WalkGenerator(SnGroup* root);
void clearSteps();
void addStep(int manip, GsVec pos, float angle);
void updateSteps();
void clearGoals(){goals.clear();}
void addGoal();
void makeGoal(GsVec pos, float angle);
void makeWalk();
void playWalk(bool setKeys = false);
int getFixI(int time);
void resetHip();
void flatten();
void search();
void getPoints ( GsModel* m, GsPolygon& pol, GsMat mat);
void exportMesh();
};



#endif //WALK_GENERATOR
