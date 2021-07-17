
# include "app_foot_viewer.h"
#include "app_main_win.h"
#include "draw_primitives.h"
# include <fltk/gl.h>
#include <gsim/gs_ogl.h>
# include <stdlib.h>
# include <stdio.h>
# include <math.h>
# include <algorithm>
#include <gsim/gs_string.h >
#include <fltk/events.h>



const float AppXmin = -5.0;
const float AppXmax = 5.0;
const float AppYmin = -5.0;
const float AppYmax = 5.0;

void AppFootViewer::clearCurves()
{
	cvs.clear();
	drawCurves=false;
}

AppFootViewer::AppFootViewer ( int x, int y, int w, int h, const char *l )
            : FlViewer ( x, y, w, h, l )
 {
	 mseGrid.set(0,0,0);
	mse.set(0,0,0);
	stepStart.set(0,0,0);
	stepEnd.set(0,0,0);
   W = w;
   H = h;
   gridH = 45;
   gridW = 45;
  setMode=0;
   pickprec = 0.04f;
   message1 = "           "; 
   message2 = "           ";
  drawCurves = 0;
 }

void AppFootViewer::drawCircle(float rad, GsVec pos)
{
	int n=20;
	 float a=0.0, da = (float)(2*3.1415/n);
	glBegin ( GL_LINE_LOOP );
      while ( n-- )
         { 
			
			 GsVec r(rad*cos(a),rad*sin(a),0);
			 glVertex2f ( pos.x + r.x, pos.y + r.y);
           a+=da;
         }
      glEnd();
    
}


GsVec AppFootViewer::win2scene ( int x, int y )
 {
	 int w = W;
	 int h = H;
   return GsVec ( float(AppXmin) + float(AppXmax-AppXmin)*float(x)/float(w),
               float(-AppYmin) - float(AppYmax-AppYmin)*float(y)/float(h),0 );
 }



void AppFootViewer::appMouseFunc ( int x, int y ) // mouse clicks
 {
	mseStart.set(win2scene(x,y));

   // Check if a vertex is being selected, ie, if mouse click
   // is very close to a vertex:
	mseGrid.set((float)x,(float)y,0);



	if(setMode==0){

		Foot* rf = new Foot(mseStart);
		rf->offset.set(0.1f,0,0);
		keyfeet.push_back(rf);
		Foot* lf = new Foot(mseStart);
		lf->offset.set(-0.1f,0,0);
		keyfeet.push_back(lf);
		setMode=1;
		stepStart.set(mseStart);
	}
	else if(setMode==1)
	{
		
		stepEnd.set(mseStart);

		Foot* rf = new Foot(mseStart);
		rf->offset.set(0.1f,0,0);
		keyfeet.push_back(rf);
		Foot* lf = new Foot(mseStart);
		lf->offset.set(-0.1f,0,0);
		keyfeet.push_back(lf);
		
		setMode=2;
	}
	else if(setMode==2)
	{

		keyfeet.clear();
		setMode=0;
		
	}

   printf("mouse %f %f  ",mse.x,mse.y); // to debug
 

	 
 }
void AppFootViewer::appMotionFunc ( int x, int y ) // mouse dragging
 {
	  int key = -1;
	
	 mse.set(win2scene(x,y));

	 GsVec dm = mseStart-mse;
	 if(dm.len()>0.01){
		 dm.normalize();
	float dotp = dot(dm,GsVec(0,1,0));

		 float angle = gs_todeg(acos(dotp));
		 float dirdot = dot(dm,GsVec(-1,0,0));
	if(dirdot<0)angle = 180.0f - angle;

			if(setMode==1){
				
				//feet[0]->c.set(mse);
				keyfeet[0]->angle = angle;
				//feet[1]->c.set(mse);
				keyfeet[1]->angle = angle;
				stepStartDir.set(mse);
			}
			else if(setMode==2)
			{
				drawCurves = true;
				//feet[2]->c.set(mse);
				keyfeet[2]->angle = angle;
			//	feet[3]->c.set(mse);
				keyfeet[3]->angle = angle;
				stepEndDir.set(mse);
				cvs.clear();
				Curve* c = new Curve();
				c->curveMode = BEZIER;
				c->p.push_back(stepStart);
				c->p.push_back(stepStartDir);
				
				GsVec endD = stepEndDir - stepEnd;
				c->p.push_back(stepEnd-endD);
				c->p.push_back(stepEnd);
				c->initTang();
				c->controlPoly = 0;
				c->divPerPair = 2;
				cvs.push_back(c);
				draw();
				laySteps();
			}
	 }
	//	 App->timeslider->draw();
	//   App->viewer->update_robot();

	
	 

 }


void AppFootViewer::drawGrid ()
{

	glLineWidth(3);
	for(int it = 0; it < H; it+= gridH)
	{
		GsString lb;
		lb << " " << it ;
		GsVec p = win2scene(5,it-3);
		glDrawString(lb, p.x, p.y, GsColor(0.5f,0.5f,0.5f));
		glColor3f(0.2f,0.2f,0.2f);
		drawLine( win2scene(0,it),win2scene(W,it) );
		glLineWidth(1);
	}
	

}
void AppFootViewer::init ( )
 {
	App->foot = this;
   FlViewer::view_all();

 }
void AppFootViewer::laySteps()
{
	feet.clear();
	App->walk->init();

	for(unsigned i=0;i<cvs.size();i++)
	{
		Curve* c = cvs[i];
		{
			for(unsigned j=1;j<c->c.size()-1;j++)
			{
				Foot* fl = new Foot(c->c[j]);
				Foot* fr = new Foot(c->c[j]);

				GsVec dpc = c->c[j+1]-c->c[j-1];
				GsVec gp(dpc.y,0,dpc.x);


				dpc.normalize();
				float angle = gs_todeg(acos(dot(dpc,GsVec(0,1,0))));
				float xdot = dot(dpc,GsVec(1,0,0));
				if (xdot > 0)
					angle = 360 - angle;

			//	App->walk->addStep(ROOT_IK,(GsVec(-0.08,3,0.05)+gp)*50,angle);
			//	App->walk->addStep(ROOT_IK,(GsVec(0.08,3,0.05)+gp)*50,angle);
				App->walk->addStep(LEFT_IK,(GsVec(-0.08f,0,0.05f)+gp)*100,angle);
				App->walk->addStep(RIGHT_IK,(GsVec(0.08f,0,-0.05f)+gp)*100,angle);

				fl->angle = angle;
				fr->angle = angle;
				fl->offset.set(-0.08f,0.05f,0);
				fr->offset.set(0.08f,-0.05f,0);
				feet.push_back(fl);
				feet.push_back(fr);
			}
		}
	}
	App->viewer->redraw();
}

void AppFootViewer::draw ()
 {
	 //----- init OpenGL if needed -----
	if ( !valid() )
	{ 
	  glViewport ( 0, 0, W, H );
	  glEnable ( GL_DEPTH_TEST );
	  glCullFace ( GL_BACK );
	  glFrontFace ( GL_CCW );
	  glPointSize ( 6 );
	  glLineWidth ( 1 );	
	}
		 //----- Clear Background -----
	glClearColor ( 1, 1, 1, 1 );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glMatrixMode(GL_MODELVIEW);
	glScale(0.2f);
	glDisable(GL_LIGHTING);

	for(unsigned i=0;i<keyfeet.size();i++)
	{
		keyfeet[i]->draw();
		if(i==1)drawCircle(0.2f,keyfeet[1]->c);
		if(i==2)drawCircle(0.2f,keyfeet[2]->c);
	}
		for(unsigned i=0;i<feet.size();i++)
	{
		feet[i]->draw();
	}

	glLineWidth(3);
	for(unsigned i=0;i<cvs.size();i++)
	{
		drawCurve(cvs[i]);
	}

	drawCircle(0.05f,stepStart);
	drawCircle(0.05f,stepStartDir);
	drawCircle(0.05f,stepEnd);
	drawCircle(0.05f,stepEndDir);

	glColor3f(1.0f,0.0f,0.0f);
	if(feet.size()>3){
	glLineWidth(1);
		drawLine(feet[0]->c,feet[2]->c);
	glLineWidth(1);
	}
		

//	drawGrid();

	glDrawString ( message1, mse.x,mse.y + 0.04f , GsColor::blue );
	glDrawString ( message2, mse.x,mse.y+  0.005f, GsColor::blue );

   //----- Fltk will then automatically flush and swap buffers -----
   	fltk::GlWindow::redraw();
}


int AppFootViewer::handle ( int ev )
{
int drw = 1;
//printf("%c\n",(char)fltk::event_key());
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
	case fltk::KEY:
		printf("key");
		break;
	default: drw=0;break;
	}
	if(drw) 
	{
	//	draw();
	}

   return fltk::GlWindow::handle(ev);
 }

