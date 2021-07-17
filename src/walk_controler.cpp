#include "walk_controler.h"
#include "app_main_win.h"

WalkControler::WalkControler()
{

	_stepTime = 1;
	_period = 1;
	_distance = 1;
	_stepHeight = 1;
	_velocity = 1;
	_step=Rest;
	_direction.set(1,0,0);
	_running = false;
	
}

 void WalkControler::start()
 {
		_running = true;
		float timeStep = 0;
		bool nextMove = false;
		double endTime = 0;
		double startTime = 0;
		double timeDif = 0;

		_step = RestToLeft;
		_stop = false;

		while(_running)
		{
			startTime = get_time();
			switch(_step)
			{
			case RestToLeft:
				if(nextMove){nextMove=false;_step=RightMove;}
				else	evaluate(RestToLeft,timeStep);
			break;
			case RestToRight:
				if(nextMove){nextMove=false;_step=LeftMove;}
				else	evaluate(RestToRight,timeStep);
			break;
			case RightMove:
				if(nextMove){nextMove=false;if(_stop)_step = LeftToRest; else _step=RootToRight;}
				else	evaluate(RightMove,timeStep);
			break;
			case LeftMove:
				if(nextMove){nextMove=false; if(_stop)_step = RightToRest; else _step=RootToLeft;}
				else	evaluate(LeftMove,timeStep);
			break;

			case RootToLeft:
				if(nextMove){nextMove=false; if(_stop)_step = LeftToRest; else _step=RightMove; }
				else	evaluate(RootToLeft,timeStep);
			break;
			case RootToRight:
				if(nextMove){nextMove=false; if(_stop)_step = RightToRest; else _step=LeftMove;}
				else	evaluate(RootToRight,timeStep);
			break;

			case RightToRest:
				if(nextMove){nextMove=false;_step=Rest; _running = false;}
				else	evaluate(RightToRest,timeStep);
			break;
			case LeftToRest:
				if(nextMove){nextMove=false;_step=Rest; _running=false;}
				else	evaluate(LeftToRest,timeStep);
			break;
			}
			timeStep+=_resolution;
			if(timeStep>=1){timeStep=0;nextMove=true;}
					fltk::check();
			endTime=get_time();
			timeDif = endTime-startTime;

			if(timeDif<_period)
				wait(_period - timeDif);
			else gsout<<"ran out of time\n";
			
			App->timeslider->draw();
			App->mainwin->ui_viewer->_kns->update();
				
			App->mainwin->ui_viewer->redraw();

		}
 }
void WalkControler::updateParams()
{
	_fps = (int)App->mainwin->ui_cont_fps->value();
	_period = 1.0f/(float)_fps;
	_stepHeight = (float)App->mainwin->ui_cont_stepHeight->value();
	_hipHeight = (float)App->mainwin->ui_cont_hipHeight->value();
	_hipDistance = (float)App->mainwin->ui_cont_hipDist->value();
	_distance = (float)App->mainwin->ui_cont_stepDistance->value();
	_resolution = 1.0f/(float)App->mainwin->ui_cont_res->value();
}
	void WalkControler::evaluate(int step, float t)
	{
float cp;
	_time = t*gspi;
	updateParams();
KnIk::Result res;
		switch(step)
		{
			case RootToLeft:
			{
					cp = _hipDistance*sin(_time-gspi/2);
					res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;
			case RootToRight: 			
			{
					cp = _hipDistance*sin(_time+gspi/2);
					res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;
			case RightMove:
				{
					cp = _stepHeight*sin(_time);
					res = setIK(0,App->rightFootStart + GsVec(0,cp,t*_distance),RIGHT_IK);
					res = setIK(0,App->leftFootStart + GsVec(0,0,-t*_distance),LEFT_IK);
					//	setIK(0,App->rootStart + GsVec(App->leftFootStart.x,_hipHeight, t*_distance ),ROOT_IK);

				}break;
			case LeftMove:
			{
					cp = _stepHeight*sin(_time);
					res = setIK(0,App->leftFootStart + GsVec(0,cp,t*_distance),LEFT_IK);
			res = setIK(0,App->rightFootStart + GsVec(0,0,-t*_distance),RIGHT_IK);
			}break;
			case RightToRest: 
			{
				cp = _hipDistance*sin(_time/2-gspi/2);
				res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;
			case LeftToRest:
			{
				cp = _hipDistance*sin(_time/2+gspi/2);
				res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;

			case RestToLeft:
			{
				cp = _hipDistance*sin(_time/2);
				res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;
			case RestToRight:
			{
				cp = _hipDistance*sin(_time/2-gspi);
				res = setIK(0,App->rootStart + GsVec(cp,_hipHeight,0),ROOT_IK);

			}break;
		}
		if(res != 0
			) gsout<< KnIk::message(res)<<gsnl;
	}

	void WalkControler::evaluate(float t)
	{
	_step = App->mainwin->ui_cont_state->focus_index();
	evaluate(_step,t);

	
	}