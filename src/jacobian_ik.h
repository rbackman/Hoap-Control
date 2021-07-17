#pragma once

#include "app_main.h"
#include <gsim/gs_matn.h>

#define NUMJ 50
#define LENGTH 60


struct Jdof
{
int dof;
KnJoint* j;
};
	class Jacobian
{
public:
	enum{
PSEUDO,
TRANSPOSE,
DAMPED,
	}Method;
float maxV;

	bool avoidCol;
	GsArray<Jdof> jnts;
	int m;
	int n;
	GsMatn J;
	GsMatn Jcol;
	GsMatn theta;
	GsMatn eff;
	GsVec s; 
	GsVec endJ;
	GsVec dEnd;
	GsVec v;
	GsVec crs;

	GsMatn Jt;
	GsMatn JJt;
	GsMatn JJt_inv;
	GsMatn j_plus; //Jt(JJt_inv)

	//Damped
	GsMatn JJTPG;
 GsMatn Idt;
	//collision 
 	int closestJ;
	float falloff; //for collision check
	float closestD;
	GsVec closestPoint;
 		GsMatn ImJpiJ; //I-J(psedoI)J
		GsMatn JpiJ;
		GsMatn Ucol;  //J0(I-Je+Je)
		GsMatn UcolT; //(J0(I-Je+Je)).transpose
		GsMatn PIO;
		GsMatn col_pseudoI;
		GsMatn JoJpi;
		GsMatn PIO_inv;
		GsMatn JoJpiXe; 
		GsMatn Xo_JoJpiXe;
GsMatn wts;
Jacobian();
	void evaluate();
	void reset();

};

GsVec GetClosetPoint(GsVec A, GsVec B, GsVec P, bool segmentClamp);
