#ifndef WALK_CONTROLER
#define WALK_CONTROLER
#include "app_main.h"
class WalkControler
{
public: 
	enum
	{
		RootToLeft,
		RootToRight,
		RightMove,
		LeftMove,
		RightToRest,
		LeftToRest,
		RestToLeft,
		RestToRight,
		Rest
	};

	
	float _stepTime;
	float _period;
	float _distance;
	float _stepHeight;
	float _velocity;
	GsVec _direction;
	float _time;
	float _hipHeight;
	float _hipDistance;
	int _step; //see enum 
	bool _running;
	bool _stop;
	int _fps;
	float _resolution;

	void updateParams();
	WalkControler();
 void start();


	void evaluate(float t);
	void evaluate(int step, float t);
	
};
#endif WALK_CONTROLER