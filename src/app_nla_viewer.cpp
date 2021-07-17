
# include "app_nla_viewer.h"
# include "app_main_win.h"

# include "draw_primitives.h"
# include <fltk/gl.h>
# include <gsim/gs_ogl.h>
# include <stdlib.h>
# include <stdio.h>
# include <math.h>
# include <algorithm>
# include <gsim/gs_string.h >
# include <fltk/events.h>
# include "utilities.h"


const float AppXmin = -1.0;
const float AppXmax = 1.0;
const float AppYmin = -1.0;
const float AppYmax = 1.0;

AppNLAEditor::AppNLAEditor ( int x, int y, int w, int h, const char *l )
            : FlViewer ( x, y, w, h, l )
 {
	mseTrack.set(-1,-1,-1);
   W = w;
   H = h;
   gridW = 45;
   trackH = 50;
   pickprec = 0.04f;
   message1 = "Welcome to the NLA editor";
   message2 = "Have a Nice Day   ";
   curTrack=0;

 }
Track* AppNLAEditor::getTrack()
{
	if(tracks.size()==0){printf("no tracks\n");return 0;}
return tracks[curTrack];
}
GsVec AppNLAEditor::win2scene ( int x, int y )
 {
	 int w = W;
	 int h = H;
   return GsVec ( float(AppXmin) + float(AppXmax-AppXmin)*float(x)/float(w),
               float(-AppYmin) - float(AppYmax-AppYmin)*float(y)/float(h),0 );
 }



void AppNLAEditor::appMouseFunc ( int x, int y ) // mouse clicks
 {
	 GsVec mseTrack = win2scene(x,y);

   // Check if a vertex is being selected, ie, if mouse click
   // is very close to a vertex:
	 for(unsigned i = 0 ; i < this->tracks.size() ; i++)
	 {
tracks[i]->hit(GsVec((float)x,(float)y,0));
	 }

   printf("mouse %f %f  ",mseTrack.x,mseTrack.y); // to debug
 
  //check to see if mouse pos is intersecting track
	 
 }
void AppNLAEditor::appMotionFunc ( int x, int y ) // mouse dragging
 {
	  int key = -1;
	  mseTrack.set(0,0,0);
	  mseTrack = win2scene(x,y);

	//if track selected move x pos to correspond to mouse

	 for(unsigned i = 0 ; i < this->tracks.size() ; i++)
	 {
		if(tracks[i]->washit)
		{
		tracks[i]->move(GsVec((float)x,(float)y,0));	
		}
	 }

		 App->timeslider->draw();
		 App->viewer->update_robot();

	
	 

 }


std::vector<GsString> getSelected()
{
std::vector<GsString> sel;

// get all tracks selected
gsout<<"not yet implemented\n";
return sel;
}



void AppNLAEditor::loadTracks()
{
	printf("load tracks\n");
	//load all tracks
	
	App->mainwin->ui_TrackListNla->add_leaf("fart");

}

void AppNLAEditor::drawGrid ()
{

	glLineWidth(3);
	for(int it = 0; it < H; it+= trackH)
	{
		GsString lb;
		lb << " " << it ;
		GsVec p = win2scene(5,it-3);
		glDrawString(lb, p.x, p.y, GsColor(0.5f,0.5f,0.5f));
		glColor3f(0.2f,0.2f,0.2f);
		drawLine( win2scene(0,it),win2scene(W,it) );
		glLineWidth(1);
	}
	
	glColor3f(1,1,0);
	drawLine( win2scene((int)App->timeslider->currentTime,0),win2scene((int)App->timeslider->currentTime,H) );
}
void AppNLAEditor::updateCurves()
{
Track* t = getTrack();
t->cvs.clear();
	for(unsigned i = 0;i< t->kg->jnts.size();i++)
	{
		for(unsigned j=0;j<t->kg->jnts[i]->dof.size();j++)
		{
			//gsout << t->kg->jnts[i]->name << " jnt name\n";
		//	gsout <<t->kg->jnts[i]->dof[j]<<" dpf\n";
			//gsout <<t->kg->sk->name() <<" is skel name\n";
				Curve* c = App->graph->makeCurve(t,t->kg->sk->joint(t->kg->jnts[i]->name),t->kg->jnts[i]->dof[j]);
				t->cvs.push_back(c);
		}
	}
	if(!t->empty) t->end = t->last_keytime();
}
void AppNLAEditor::init ( )
 {
   App->nla = this;
   FlViewer::view_all();
  App->graph->makeTrack();
 }
int AppNLAEditor::trackSelected()
{
	int sz = App->mainwin->ui_TrackList->children();
	for(int i=0;i<sz;i++)
	{
		if(App->mainwin->ui_TrackList->child(i)->selected())
		{
			printf("track %d selected\n",i);
			curTrack = i;
			return i;
		}
	}	
	return 0;
}
void AppNLAEditor::addTrack(Track* t)
{
	tracks.push_back(t);
}

void AppNLAEditor::trackList()
{
	GsString track = App->mainwin->ui_TrackList->child(trackSelected())->label();
	curTrack = trackSelected();
	App->timeslider->timeline = getTrack();
	App->mainwin->ui_currentTrack->text(track);
	gsout<<track<<" selected";
	App->graph->channelSel();
}
void AppNLAEditor::drawTrack (Track* t)
{
	int bx = (int)t->start;
	int by = (int)t->timeChannel*trackH + 5;
	int tx = (int)t->start + (int)t->end;
	int ty =  (int)t->timeChannel* trackH + trackH - 5;

	GsVec b = win2scene(bx,by);
	GsVec tp = win2scene(tx,ty);
	GsVec mid = midpoint(b,tp);
	b.z=0.2f;
	tp.z = b.z;

	glColor3f(t->visCol.r,t->visCol.g,t->visCol.b);
	glDrawBox(b,tp); 
	glDrawString(t->name(), mid.x , mid.y, GsColor::yellow );	
}

void AppNLAEditor::draw ()
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
	glDisable(GL_LIGHTING);

	for(unsigned i=0;i<tracks.size();i++)
	{
	drawTrack(tracks[i]);
	}
	drawGrid();

	glDrawString ( message1, mseTrack.x,mseTrack.y + 0.04f , GsColor::blue );
	glDrawString ( message2, mseTrack.x,mseTrack.y+  0.005f, GsColor::blue );

   //----- Fltk will then automatically flush and swap buffers -----
   	fltk::GlWindow::redraw();
}

void AppNLAEditor::insertTrack()
{
printf("insert track\n");
}

int AppNLAEditor::handle ( int ev )
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
		draw();
	}

   return fltk::GlWindow::handle(ev);
 }

