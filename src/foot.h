#pragma once

#include <gsim/sn_transform.h>
#include <gsim/sn_model.h>
#include <gsim/sn_group.h>
class Foot{
public:
	bool adjusted;
	SnGroup* g;
	SnTransform* tfm;
	SnModel* model;

	GsVec c,fl,fr,bl,br;
	GsVec offset;
	float angle;
	Foot(GsVec ct);

void Rotate(float deg);
void Translate(GsVec pos);
void draw();
};

class FootGoal{
public:
	SnGroup* modg;
	SnGroup* g;
	SnTransform* tfm;
	Foot* leftFoot;
	Foot* rightFoot;
	GsVec pos;
	float angle;
	
	void Rotate(float deg);
	void Translate(GsVec pos);
	FootGoal(GsVec p,float a);
};

