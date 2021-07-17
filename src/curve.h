// Ensure the header file is included only once in multi-file projects
// Pragma once is a non-standard but widely supported preprocessor directive
# pragma once
#include "app_main.h"

// A vector

#define BPIECES 1

#define LEGRANGE 0
#define BEZIER 1
#define BSPLINEQUAD 2
#define BSPLINECUBIC 3
#define BEZIERPIECES 4
class Curve;
class Track;
class Jnt;

//DOF
enum{
RX,RY,RZ,PX,PY,PZ
};

class Tang //tangent 
{
	public :
	int idx;
	GsVec* pt;		//curve pt for reference
	GsVec in;	    //in direction
	GsVec out;      //out direction
	GsVec	pi;     //in point
	GsVec	po;     //out point
	bool locked;    //in dir = out dir
	bool free;		//free = no tangent control
	bool flat;		//tang = 0
	int sharp;		// limit to prevent overlap
	int	tangSelected; //0 1:INtang 2:outTang
	Tang(GsVec* point);
};

class Curve
 { public :
   std::vector<GsVec> c; //curve points
   std::vector<GsVec> p; //control points
    std::vector<Tang*> t; //tangents
   std::vector<Curve*> subCvs; //for piecwise
   bool vis;  /*!determines if curve will be drawn*/
   GsVec cCol; /*!color for curve*/ 
   GsVec cpCol;/*!color for control poly*/ 

    void setTime(float kt);
    int first;  /*!for initialization*/
	int flat;  /*! sort curve to keep it a function*/
	unsigned selection;  //the current selected control vertex
	int divPerPair;		 //measure of curve resolution
	int curveMode;		 //current method for curve generation
	int closed;
	int selectionState; //0:not selected 1:pt selected 2: tangent selected
	int controlPoly;
	float curveTime;

	float T; //parametric curve length
	int dof;
	GsString jntName;
	KnJoint* j;
	KnSkeleton* skel;
	Track* m;
GsVec eval_bezier   ( float t);
GsVec eval_lagrange ( float t);
GsVec eval_bspline ( float u,int k);
    /*! Curve constructor n*/
 Curve();
	Curve (Track* motion, KnJoint* j,int dof);

   bool sortX();
   void update();
   void createTang(int i);
   void  initTang();
   void makeSub();
 };

