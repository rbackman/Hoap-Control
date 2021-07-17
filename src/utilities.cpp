#include "utilities.h"

#include "app_main.h"
#include "app_main_win.h"
#include <time.h>
#ifdef WIN32
#include <sys/types.h>
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

GsVec interp(GsVec start, GsVec end , float t)
{
return start + (end - start)*t;;
}
float interp(float start,float end, float t)
{
return start + (end-start)*t;
}

GsVec midpoint(GsVec a, GsVec b)
{
	return GsVec((a.x+b.x)/2.0f , (a.y+b.y)/2.0f , (a.z+b.z)/2.0f );
}
float rnd(float j)
{
return (float)(((int)(j*1000))/1000.0);
}
long int fact(int n)
 {
  if (n<=1)
	return(1);
  else
	n=n*fact(n-1);
	return(n);
 }


KnIk::Result updateIK()
{
		KnIkManipulator* ikm;
			SnGroup* g = App->mainwin->ui_viewer->_ikmanips;
			int start = 2;
			int errors = 0;
			KnIk::Result res;
		if(App->armIKActive)start = 0;
		for(int j= start; j<4;j++)
			   {
					 ikm = (KnIkManipulator*)g->get(j); //update feet ik with new root loc
					 res = ikm->update();
					 if(res) errors++;
			   }
		if(errors>1)gsout<<errors<<" errors found in IK  ";
		return res;
}
KnIk::Result setIK(float deg,GsVec pos,int IK)
{
	SnGroup* g = App->mainwin->ui_viewer->_ikmanips;
	KnIkManipulator* ikm;
KnIk::Result res;
	switch(IK)
	{
	case LEFT_IK:
		{ ikm = (KnIkManipulator*)g->get(2);	
		  ikm->manip()->mat().roty(gs_torad(deg));
		  ikm->manip()->mat().setrans(pos);
		
		 res = ikm->update();
	

		} break;
	case RIGHT_IK:
		{
		ikm = (KnIkManipulator*)g->get(3);	
		ikm->manip()->mat().roty(gs_torad(deg));
		ikm->manip()->mat().setrans(pos);
		
		 res = ikm->update();
		//  gsout<<"RIGHT_IK" << KnIk::message(res)<<gsnl;
		}
	break;
	case ROOT_IK:
		{
		GsMat gm;
		gm.roty(gs_torad(deg));
		gm.setrans(pos);
		App->sk->joint("Hips")->pos()->value(pos);
		App->sk->joint("Hips")->euler()->value(1,gs_torad(deg));
		App->mainwin->ui_viewer->_RootIK->mat().set(gm);
		
		App->mainwin->ui_viewer->_RootIK->update();

		res =  updateIK();

		}break;
	}
	App->mainwin->ui_viewer->_kns->update();
	App->mainwin->ui_viewer->redraw();
return res;
}

float interp_cubic ( float t, float tmin, float tmax )
 {
   t = (t-tmin)/(tmax-tmin);    // normalize t to [0,1]
   t=-(2.0f*(t*t*t)) + (3.0f*(t*t));  // cubic spline
   return t*(tmax-tmin) + tmin; // scale back
 }

double get_time ()
{
  #ifdef WIN32
   // if better precision is needed in Windows, use QueryPerformanceCounter
   _timeb tp;
   _ftime_s(&tp);
   return 0.001*(double)tp.millitm + (double)tp.time;
  #else
   timeval tp;
   if ( gettimeofday(&tp,0)==-1 ) return 0;
   return 0.000001*(double)tp.tv_usec + (double)tp.tv_sec;
  #endif
} 
void wait ( double seconds )
{
  clock_t endwait;
  endwait = clock () + (clock_t)(seconds * CLOCKS_PER_SEC) ;
  while (clock() < endwait) {}
}