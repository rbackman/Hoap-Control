
# include <fltk/gl.h>

# include <stdlib.h>
# include <stdio.h>
# include <math.h>
#include <time.h>

#include <fltk/events.h>
# include "time_slider.h"
# include "app_fluid.h"
#include "app_main.h"
#include "app_main_win.h"

const float AppXmin = -1.0;
const float AppXmax = 1.0;
const float AppYmin = -1.0;
const float AppYmax = 1.0;

void wait ( int seconds )
{
  clock_t endwait;
  endwait = clock () + seconds * CLOCKS_PER_SEC ;
  while (clock() < endwait) {}
}
TimeSlider::TimeSlider ( int x, int y, int w, int h, const char *l )
            : fltk::GlWindow (x, y, w, h, l)
 {
	W = w;
	H = h;
	fps = 30;
	currentTime = 0;
	frameTog = true;
	timeline = new Track();
	IKtimeline = new Track();
	timeline->load("../data/solution.sm");
	IKtimeline->load("../data/solution.sm");
	size = W*H;
	pixels = new float[size*3];
	move=0; moveKey =0;
 }
/*
void TimeSlider::resize(int w,int h)
{
	printf("resize");
	W=w;H=h;
	fltk::GlWindow ::resize(w,h);
}*/
void TimeSlider::play()
{
	startTime = App->mainwin->ui_startFrame->value();
	endTime  = App->mainwin->ui_endFrame->value();
	fps = (int)App->mainwin->ui_timefps->value();
	int endFrame = timeline->frames();
	float dt =  1.0f/(float)fps;
	if(frameTog)dt = 1.0f;
	
	for(double t = startTime ; t < endTime ; t+=dt)
			{
				if(App->mainwin->ui_playIK->value())
				{
					IKtimeline->apply((float)t,KnMotion::Linear,&endFrame);
					App->mainwin->orientIK();
				}
				else
				{
					timeline->apply((float)t/60.0f,KnMotion::Linear,&endFrame);
					printf("time is %g\n",(float)t/60.0f);
				}
				wait((1.0f/(float)fps));
				currentTime = t;
				
				App->mainwin->updateSliders();
				if(App->realtime)
				{
					App->mainwin->sendPosture(); 
				}
				App->viewer->update_robot ();
		
				App->viewer->redraw();
				fltk::check();
				draw();
			}

}
GsVec TimeSlider::win2scene ( int x, int y )
 {
	 int w = W;
	 int h = H;
   return GsVec ( float(AppXmin) + float(AppXmax-AppXmin)*float(x)/float(w),
               float(-AppYmin) - float(AppYmax-AppYmin)*float(y)/float(h),0 );
 }

void TimeSlider::drawSquare(int x,int y, int w, int h,float r,float g, float b)
{
	x-=(int)w/2;
y-=(int)h/2;
	for(int it =x;it<x+w;it++)
	{
		for(int j=y;j<y+h;j++)
		{
			setPixel(it,j,r,g,b);
		}
	}
}
float TimeSlider::winToTime(int x)
{
float mTime = (float)x;
startTime  = App->mainwin->ui_startFrame->value();
endTime = App->mainwin->ui_endFrame->value();


if(!frameTog)
{
mTime =  (float)((endTime-startTime)/W)*x;
}
return mTime;
}
int TimeSlider::timeToWin(float tm)
{
	int pixTime = 0;

	startTime  = App->mainwin->ui_startFrame->value();
	endTime = App->mainwin->ui_endFrame->value();

	if(!frameTog)
	{
	//	dt = 1.0f/(float)fps;
		pixTime = (int)(( startTime + tm/(endTime-startTime) )*W);
	//	printf("pixTime %d\n",pixTime);
	}
	else
	{
		pixTime =(int)(startTime + tm);
	}
	return pixTime;
}

void TimeSlider::appMouseFunc ( int x, int y ) // mouse clicks
 {
	 int endFrame = 0;
	 if(x<0)x=0;
  mseTime = win2scene(x,y);
  // printf("mouse click: %f %f x,y: %d, %d\n",m.x,m.y,x,y); 
 	  	 

	currentTime = winToTime(x);
	  
	   if(App->mainwin->ui_playIK->value())
	   {
		   IKtimeline->apply((float)currentTime,KnMotion::Linear,&endFrame);
			App->mainwin->orientIK();
	   }
	   else
	   {
		timeline->apply((float)currentTime,KnMotion::Linear,&endFrame);
		 App->mainwin->ui_curFrame->value(currentTime);
	   }
	   App->sk->land();
		App->mainwin->ui_viewer->update_robot();
		App->mainwin->matchIK();
		App->mainwin->updateSliders();
	  App->mainwin->ui_viewer->redraw();
	  fltk::check();
 //  draw();
 }
void TimeSlider::appMotionFunc ( int x, int y ) // mouse dragging
 {
	  int endFrame = 0;
	  if(x<0)x=0;
	// printf("motion func\n");
	  	 
	  currentTime = winToTime(x);
	  timeline->apply((float)currentTime,KnMotion::Linear,&endFrame);
	  if(move)
	  {
		  timeline->keytime(moveKey,(float)currentTime);
	  }
	  App->sk->land();


	  App->mainwin->ui_curFrame->value(currentTime);
	  App->mainwin->updateSliders();
	  App->mainwin->ui_viewer->_kns->update();
	  App->mainwin->matchIK();
	  App->mainwin->ui_viewer->redraw();
	  fltk::check();
	
 }
void TimeSlider::ikKey(double kt)
{
	int frameSet = 0;
	KnPosture* p = new KnPosture(App->ikskel);
		p->get();
		for(unsigned it=0; it < timeline->frames()-1; it++)
		{
			if(IKtimeline->frames()==0)break;
			if(kt>IKtimeline->keytime(it) && kt < IKtimeline->keytime(it+1))
			{
				IKtimeline->insert_frame(it+1,(float)kt,p);
				frameSet=1;
			}
		}
		if(!frameSet)
		{
		//	printf("frame set to end\n");
			IKtimeline->insert_frame(timeline->frames(),(float)kt,p);
		}
}

void TimeSlider::ikKey()
{
	if(!App->mainwin->ikCreated){gsout<<"ik has not been created yet\n";return;}
	Track* t = App->nla->getTrack();
	if(!t->IKTrack){printf("not an IK track\n");return;}
	KeyGroup* kg = App->graph->getKeyGroup();

	double kt = currentTime;
	int frameSet = 0;
	App->viewer->updateIKSkel();
	KnPosture* p = new KnPosture(App->ikskel);
	p->get();
	IKtimeline = t;

	for(unsigned it=0; it < t->frames()-1; it++)
	{
		if(t->frames()==0)break;
		if(kt>t->keytime(it) && kt < t->keytime(it+1))
		{
			t->insert_frame(it+1,(float)kt,p);
			frameSet=1;
		}
	}
	if(!frameSet) //set to end
	{
		t->insert_frame(t->frames(),(float)kt,p);
	}
	App->nla->updateCurves();

}
void TimeSlider::jointKey(double kt)
{
	int frameSet = 0;
	KnPosture* p = new KnPosture(App->sk);
		p->get();
		for(unsigned it=0; it < timeline->frames()-1; it++)
		{
			if(timeline->frames()==0)
			{
				gsout<<"frames = 0"<<gsnl;
				break;
				
			}
			if(kt>timeline->keytime(it) && kt < timeline->keytime(it+1))
			{
				timeline->insert_frame(it+1,(float)kt,p);
				frameSet=1;
			}
		}
		if(!frameSet)
		{
			printf("frame set to end\n");
			timeline->insert_frame(timeline->frames(),(float)kt,p);
		}
			App->nla->updateCurves();
}

void TimeSlider::jointKey()
{
	double kt = currentTime;
	int frameSet = 0;
	KeyGroup* kg = App->graph->getKeyGroup();
		
		KnPosture* p = new KnPosture(App->sk);
	//	printf("posture contains: ");
		//	p->output(gsout,1,1);
	//	}
//gsout<<"\n";

		for(unsigned i=0;i<kg->jnts.size();i++)
	{
		//add only joints in kg to posture
	}
	
		p->get();
		Track* t = App->nla->getTrack();
		if(t->IKTrack){printf("IK track selected\n");return;}
	
		for(unsigned it=0; it < timeline->frames()-1; it++)
		{
			if(timeline->frames()==0)break;
			if(kt>timeline->keytime(it) && kt < timeline->keytime(it+1))
			{
				timeline->insert_frame(it+1,(float)kt,p);
				
				frameSet=1;
			}
		
		}
		if(!frameSet)
		{
			printf("frame set to end\n");
			timeline->insert_frame(timeline->frames(),(float)kt,p);
			
		}
		if(t->empty)
			t->empty=false;
		App->nla->updateCurves();
}

void TimeSlider::circlePoints(int cx, int cy, int x, int y,float r,float g, float b)
    {   
        if (x == 0) {
            setPixel(cx, cy + y,r,g,b);
            setPixel( cx, cy - y,r,g,b);
            setPixel( cx + y, cy,r,g,b);
            setPixel( cx - y, cy,r,g,b);
        } else 
        if (x == y) {
            setPixel(cx + x, cy + y,r,g,b);
            setPixel( cx - x, cy + y,r,g,b);
            setPixel( cx + x, cy - y,r,g,b);
            setPixel( cx - x, cy - y,r,g,b);
        } else 
        if (x < y) {
            setPixel(cx + x, cy + y,r,g,b);
            setPixel( cx - x, cy + y,r,g,b);
            setPixel( cx + x, cy - y,r,g,b);
            setPixel( cx - x, cy - y,r,g,b);
            setPixel( cx + y, cy + x,r,g,b);
            setPixel( cx - y, cy + x,r,g,b);
            setPixel( cx + y, cy - x,r,g,b);
            setPixel( cx - y, cy - x,r,g,b);
        }
    }

void TimeSlider::circleMidpoint(int xCenter, int yCenter, int radius, float r,float g, float b)
{
    int x = 0;
    int y = radius;
    int p = (5 - radius*4)/4;
    circlePoints(xCenter, yCenter, x, y,r,g,b);
    while (x < y) {
        x++;
        if (p < 0) {
            p += 2*x+1;
        } else {
            y--;
            p += 2*(x-y)+1;
        }
        circlePoints(xCenter, yCenter, x, y,r,g,b);
    }
}
void TimeSlider::setPixel(int x,int y, float r,float g,float b)
{
	if(x>0 && x<W && y>0 && y<H)
	{
	pixels[size*3-3*(y*W+(W-x))]   = r;
	pixels[size*3-3*(y*W+(W-x))+1] = g;
	pixels[size*3-3*(y*W+(W-x))+2] = b;
	}
}

void TimeSlider::draw ()
 {
	 //----- init OpenGL if needed -----
  // if ( !valid() )
 //   { 
		glViewport ( 0, 0,w(),h());
		glEnable ( GL_DEPTH_TEST );
		glCullFace ( GL_BACK );
		glFrontFace ( GL_CCW );
		glPointSize ( 6 );
		glLineWidth ( 2 );
  //  }
		//W = w();
		//H = h();

	//----- Clear Background -----
	glClearColor ( 0.7f, 0.7f, 0.7f, 1 );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glRasterPos2d(-1,-1);
	flood(0,0,1);



	drawSquare((int)timeToWin((float)currentTime),(int)(H/2),(int)(H/2),(int)(H-5),1.0f,1.0f,0.0f);

	IKtimeline = App->nla->getTrack();



	for(unsigned it=0;it< timeline->frames();it++)
   {
	   
	   float x = timeline->keytime(it);
	   int tm = timeToWin(x);
	   drawSquare(tm,H/2,H/5,H-5,1.0f,0.0f,0.0f);
   }
	
      for(unsigned it=0;it< IKtimeline->frames();it++)
   {
	   float x = IKtimeline->keytime(it);
	    int tm = timeToWin(x);

	
	   drawSquare(tm,H/2,H/5,H-5,0.0f,1.0f,0.0f);
	   circleMidpoint(tm,H/2,H/2-10,0.0,1,0);
   }
   //draw to frame buffer
	glDrawPixels(W,H,GL_RGB,GL_FLOAT,pixels);
	fltk::GlWindow::redraw();
   //----- Fltk will then automatically flush and swap buffers -----
}
void TimeSlider::flood(float r,float g,float b)
{
	for(int it=0;it<w();it++)
	{
		for(int j=0;j<h();j++)
		{
		setPixel(it,j,r,g,b);
		}
	}
}
int eve = 0;
int TimeSlider::handle ( int ev )
{
int drw = 1;
kt = 1;
	switch(ev)
	{		
	case fltk::PUSH :	
		appMouseFunc (fltk::e_x,fltk::e_y);
		draw();
		return 1;
		break;
	case fltk::DRAG :
		appMotionFunc (fltk::e_x,fltk::e_y);
		draw();
		break;
	case fltk::MOUSEWHEEL:
		draw();
		break;
	default: drw=0;break;
	}

   return fltk::GlWindow::handle(ev);
 }
