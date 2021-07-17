
# include "app_graph_viewer.h"
#include "app_main_win.h"

#include "draw_primitives.h"
# include <fltk/gl.h>
#include <gsim/gs_ogl.h>
# include <stdlib.h>
# include <stdio.h>
# include <math.h>
#include <algorithm>
#include <gsim/gs_string.h >
#include <fltk/events.h>
const float AppXmin = -1.0;
const float AppXmax = 1.0;
const float AppYmin = -1.0;
const float AppYmax = 1.0;

AppGraphViewer::AppGraphViewer ( int x, int y, int w, int h, const char *l )
            : FlViewer ( x, y, w, h, l )
 {
   c = new Curve;
   c->flat = 0;
   c->closed = 0;
   c->controlPoly = 1;
   c->selection = 9999; // value >pol.size() means no selection
   c->divPerPair = 2;
   c->T = 1;
   c->flat = 1;
   c->curveMode = BSPLINEQUAD;//LEGRANGE; //

   pickprec = 0.04f;
   mseGraph.set(-1,-1,-1);
   W = w;
   H = h;
   message1 = "                              ";
   message2 = "                              ";
   Origin = H/2;
   gridW = 45;
   gridH = 45;
   emptyTimeline = true;
   emptyTrack = true;
   
 }

GsVec AppGraphViewer::win2sceneGraph ( int x, int y )
 {
	 int w = W;
	 int h = H;
   return GsVec ( float(AppXmin) + float(AppXmax-AppXmin)*float(x)/float(w),
               float(-AppYmin) - float(AppYmax-AppYmin)*float(y)/float(h),0 );
 }
GsVec AppGraphViewer::scene2winGraph(int x, int y)
{
//implement this
	return GsVec(0,0,0);
}
void AppGraphViewer::appMouseFunc ( int x, int y ) // mouse clicks
 {
   GsVec m = win2sceneGraph(x,y);
   // Check if a vertex is being selected, ie, if mouse click
   // is very close to a vertex:
  // printf("mouse %f %f  ",m.x,m.y); // to debug
   unsigned it;
   int done = 0;
   Track* t = App->nla->getTrack();
   if(t)
   if(!t->empty)
   {
	   cvs = t->cvs;
   		for(unsigned j=0;j<cvs.size();j++)
		{ if(!cvs[j]->vis)break;

				if(cvs[j]->selectionState)
			{
				int sel = cvs[j]->selection;
				if(      dist( m,  cvs[j]->t[sel]->pi ) < pickprec ) { cvs[j]->t[sel]->tangSelected = 1; cvs[j]->selectionState = 2; return;} //tang selected
				else if( dist( m,  cvs[j]->t[sel]->po ) < pickprec ) { cvs[j]->t[sel]->tangSelected = 2; cvs[j]->selectionState = 2; return;}
				else cvs[j]->t[sel]->tangSelected = 0;
				
			}
		}
	   for(unsigned j=0;j<cvs.size();j++)
	   {
			cvs[j]->selectionState = 0;
		}
		for(unsigned j=0;j<cvs.size();j++)
	   {if(!cvs[j]->vis)break;
		
			 // printf("curve %d \n",j);
			   std::vector<GsVec>& p = cvs[j]->p;
			   
			   for ( it=0; it<p.size(); it++ )
			   {
				   if ( dist(p[it],m)<=pickprec )
				   {
					// printf("curve %d pt %d selected\n",j,it);
					cvs[j]->selectionState = 1;
					cvs[j]->selection = it;
					done = 1;break;
				   }
			   }
		   if(done)break; 

	   }
   }	 
}

void AppGraphViewer::appMotionFunc ( int x, int y ) // mouse dragging
 {
	int key = -1;
	mseGraph.set(0,0,0);
	mseGraph = win2sceneGraph(x,y);

	 Track* t = App->nla->getTrack();
	if(t) //see if there are any valid tracks
	if(!t->empty)
   {
	  // gsout<<"t is not empty\n";
	   cvs = t->cvs;
		for(unsigned j=0;j<cvs.size();j++)
	   {
		   if(!cvs[j]->vis)break;
		   switch(cvs[j]->selectionState)
		   {
		   case TANG_EDIT://selState 2 is tangent edit mode
		   {
			  Tang* tg = cvs[j]->t[ cvs[j]->selection ];
			  int tsel = tg->tangSelected;
			  int psel = cvs[j]->selection;
			  printf("curve: %d sel %d : tgSel:%d \n",j,psel,tsel);
			  if(tsel ==0)break;
			  else if (tsel==1)
			  {
				  tg->in =  cvs[j]->p[psel]-mseGraph  ; 
				  tg->pi = mseGraph; 
			  }
			  else if(tsel==2)
			  {
				  tg->po = mseGraph;
				  tg->out = mseGraph - cvs[j]->p[psel] ; 
			  }
			  
		   }break;
		   case POINT_EDIT: // there is a selection
		{ 
			std::vector<GsVec>& p = cvs[j]->p;
			
			p[cvs[j]->selection] = mseGraph;
			for(unsigned i=0;i<cvs.size();i++)
			{
				cvs[i]->p[cvs[j]->selection].x = mseGraph.x;
			}
			int sz = p.size();
			key = cvs[j]->selection;
			
			if(cvs[j]->flat && sz>=3)
			{
			  // printf("p.size =%d flat is %d\n",p.size(),c->flat);
				cvs[j]->sortX();
			}
			cvs[j]->m->keytime(cvs[j]->selection,(float)x);
			
			//char* st;			
			//sprintf(st,"pt:%d cv:%d val:%d",cvs[j]->selection,j,x); 
			GsString str;
			str << "t: " << x ;
			message1 = str;

			GsString str2;
			str2 << "val: " << y - Origin ;
			message2 = str2;
			}break;
		}
	    cvs[j]->update();
	}
	
		if(key != -1)
		{
			 for(unsigned j=0;j<cvs.size();j++)
			 {
				//   cvs[j]->p[key].x = mseGraph.x;
			 }
			 App->timeslider->draw();
			 App->mainwin->ui_viewer->update_robot();
			//App->mainwin->ui_viewer->draw();
		}
   }
}
void AppGraphViewer::freeTangent()
{
	for(unsigned j=0;j<cvs.size();j++)
	{
		if(cvs[j]->selectionState == TANG_EDIT)
		{
			printf("free tang curve[%d]\n",j);
			cvs[j]->t[cvs[j]->selection]->free=true;
		}

	}
}
const char* getSelected()
{
	fltk::Browser* b = App->mainwin->ui_channelList;
	for(int it=0;it<b->children();it++)
	{
		if(b->child(it)->selected())
			return b->child(it)->label();
	}
	return 0;
}
std::vector<GsString> getAllSelected()
{
std::vector<GsString> sel;
fltk::Browser* b = App->mainwin->ui_channelList;
	for(int it=0;it<b->children();it++)
	{
		if(b->child(it)->selected())
		sel.push_back( b->child(it)->label() );
	}

return sel;
}

Curve* AppGraphViewer::makeCurve(Track* m,KnJoint* j,int dof) //dof rx,ry,rz,px,py,pz
{
	unsigned frames = m->frames();
	GsVec p;
	Curve* cv = new Curve();
	cv->dof = dof;
	cv->jntName = (const char*)j->name();
	cv->skel = j->skeleton();
	cv->m = m;
	cv->first = 1;
	for(unsigned it=0;it<frames;it++)
	{
		m->apply(m->keytime(it));
		p.z=0;
		p.x = m->keytime(it);
		if(dof<3){
			p.y=GS_TODEG(j->euler()->value(dof))+H/2;	
		}
		else
		{
			//printf("frame# %d jnt %s val[%d]: %g\n", it, (const char*)j->name() , dof-3, j->pos()->value(dof-3));
			p.y = 10*j->pos()->value(dof-3)+H/2;	
		}
		cv->p.push_back( win2sceneGraph((int)p.x,(int)p.y) );
	}
	return cv;
}
bool AppGraphViewer::jSelected(GsString j)
{
std::vector<GsString> sel = getAllSelected();
	for(unsigned i=0;i<sel.size();i++)
	{
		if(j==sel[i]) return true;
	}
	return false;
}
void AppGraphViewer::channelSel()
{
	GsString selected = getSelected();
	std::vector<GsString> sel = getAllSelected();
	cvs.clear();

	//decide which curves to display from ui dof toggles
	bool but[6];
	but[0] =  App->mainwin->ui_channelRX->value();
	but[1] =   App->mainwin->ui_channelRY->value();
	but[2] =   App->mainwin->ui_channelRZ->value();
	but[3] = App->mainwin->ui_channelPX->value();
	but[4] =  App->mainwin->ui_channelPY->value();
	but[5] =  App->mainwin->ui_channelPZ->value();
	

		Track* m;
		if(App->mainwin->ui_ikChannel->value())
		{
			m = App->timeslider->IKtimeline;
		}
		else
		{
			m = App->timeslider->timeline;
		}
		if(!m->empty)
		{
			int c =0;
			for(unsigned j=0; j< m->kg->jnts.size();j++)
			{
				Jnt* jnt = m->kg->jnts[j];
				for(unsigned d =0;d< jnt->dof.size();d++)
				{
					if(but[jnt->dof[d]]&& jSelected(jnt->name)) 
					{
						//gsout<<"jntName "<<jnt->name<<" Dof "<<jnt->dof[d]<<"  but[jnt->dof[d]] "<< but[jnt->dof[d]]<<"\n";
						m->cvs[c]->vis = true;
					}
					else
					{
					//	gsout<<"jntName "<<jnt->name<<" Dof "<<jnt->dof[d]<<"  but[jnt->dof[d]] "<< but[jnt->dof[d]]<<"\n";
						m->cvs[c]->vis = false;
					}
					c++;
				}
			}
			cvs = m->cvs;
		}

			/*for(unsigned k=0;k<sel.size();k++)
	{
	selected = sel[k];
	KnJoint* j = App->sk->joint(selected);
	KnJointPos* pch = j->pos();
	KnJointEuler* rch = j->rot()->euler();

	

	if(selected)
	{	*/

			/*int sz = m->cvs.size();
			printf("%d",sz);
			gsout << " curves in track "<< m->name()<< "  keygroup: " << m->kg->name <<"\n";

			for(unsigned i=0;i<m->cvs.size();i++)
			{
				cvs.push_back(m->cvs[i]);
			}*/
		
		/*KnChannels* ch;
		if(ch = m->channels())
		{
			
			if(App->mainwin->ui_ikChannel->value())
			{
				if(RX)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 0));
					cvs.push_back(m->cvs[]
				}
				if(RY)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 1));
				}
				if(RZ)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 2));
				if(PX)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 3));
				}
				if(PY)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 4));
				}
				if(PZ)
				{
					cvs.push_back(makeCurve( m , App->ikskel->joint(selected) , 5));
				}

				}
			}
			else
			{
				if(RX)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 0));
				}
				if(RY)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 1));
				}
				if(RZ)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 2));
				}
				if(PX)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 3));
					
				}
				if(PY)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 4));
				}
				if(PZ)
				{
					cvs.push_back(makeCurve( m , App->sk->joint(selected) , 5));
				}

			}
			*/
		//}
	//}
	draw();
//	printf("  butState: %d %d %d %d %d %d\n",PX,PY,PZ,RX,RY,RZ);
	
//	}
}
bool isFree(KnJoint* j)
{
		KnJointPos* pch = j->pos();
	KnJointEuler* rch = j->rot()->euler();

	return
		!pch->frozen(0)||!pch->frozen(1)||!pch->frozen(2)||
		!rch->frozen(0)||!rch->frozen(1)||!rch->frozen(2);
}
void AppGraphViewer::loadChannels()
{
	App->mainwin->ui_channelList->clear();
	if(App->mainwin->ui_ikChannel->value())
	{
		const GsArray<KnJoint*> jnts = App->ikskel->joints();
		for(int it=0;it<jnts.size();it++)
		{
			if(isFree(jnts[it]))
			App->mainwin->ui_channelList->add((const char*)jnts[it]->name());
		}
	}
	else
	{
		const GsArray<KnJoint*> jnts = App->sk->joints();
		for(int it=0;it<jnts.size();it++)
		{
			if(isFree(jnts[it]))
			App->mainwin->ui_channelList->add((const char*)jnts[it]->name());
		}
	}
	
}

void AppGraphViewer::drawGrid ()
{
	glColor3f(0.2f,0.2f,0.2f);
	for(int it = Origin; it < H; it+= gridH)
	{
		drawLine( win2sceneGraph(0,it),win2sceneGraph(W,it) );
	}
	for(int it = Origin; it > 0; it-= gridH)
	{
		drawLine( win2sceneGraph(0,it),win2sceneGraph(W,it) );
	}
	for(int it = 0; it < W; it+= gridW)
	{
		drawLine( win2sceneGraph(it,0),win2sceneGraph(it,H) );
	}
	glColor3f(1,1,0);
	drawLine( win2sceneGraph((int)App->timeslider->currentTime,0),win2sceneGraph((int)App->timeslider->currentTime,H) );
}
void AppGraphViewer::init (  )
 {
   App->graph = this;
   FlViewer::view_all();
   makeGroup();
   
 }
  
void AppGraphViewer::drawCircle(float rad, GsVec pos)
{
	int n=20;
	 float a=0.0, da = (float)(2*3.1415/n);
	glBegin ( GL_LINE_LOOP );
      while ( n-- )
         { 
			
			 GsVec r = win2sceneGraph((int)(rad*cos(a)),(int)(rad*sin(a)));
			 glVertex2f ( pos.x + r.x, pos.y + r.y);
           a+=da;
         }
      glEnd();
    
}
Jnt* getJnt(KnJoint* j)
{
		KnJointPos* pch = j->pos();
	KnJointEuler* rch = j->rot()->euler();

	Jnt* jn = new Jnt((GsString)j->name());
	if(!pch->frozen(0))jn->dof.push_back(PX);
	if(!pch->frozen(1))jn->dof.push_back(PY);
	if(!pch->frozen(2))jn->dof.push_back(PZ);
	if(!rch->frozen(0))jn->dof.push_back(RX);
	if(!rch->frozen(1))jn->dof.push_back(RY);
	if(!rch->frozen(2))jn->dof.push_back(RZ);
	return jn;
}
void AppGraphViewer::makeGroup()
{
	GsString name;
	KeyGroup* kg;
	bool makeIK = App->mainwin->ui_ikChannel->value();
	if(emptyTimeline)
	{
		//printf("default IK group made\n");
		name.set("IK");

		kg = new KeyGroup(name,App->ikskel);

		emptyTimeline=false;
		for(int i=0;i<App->ikskel->joints().size();i++)
		{	
				//KnJoint* j = App->sk->joint(App->mainwin->ui_channelList->child(i)->label()); 
			kg->jnts.push_back( getJnt( App->ikskel->joints()[i] ) );
		}

	}
	else
	{
			fl_string_input("Curve Group Name","please enter a name for the curve group",name);
		gsout << name << " was entered\n";
		
		if(makeIK)
			kg = new KeyGroup(name,App->ikskel);
		else
			kg = new KeyGroup(name,App->sk);
		
		for(int i=0;i<App->mainwin->ui_channelList->children();i++)
		{
			if(App->mainwin->ui_channelList->child(i)->selected()) 
			{
				KnJoint* j ;
				if(makeIK)
					j = App->ikskel->joint(App->mainwin->ui_channelList->child(i)->label());
				else
					j = App->sk->joint(App->mainwin->ui_channelList->child(i)->label()); 
				
					kg->jnts.push_back( getJnt(j) );
				
			}
		}
	}
	keyGroups.push_back(kg);
	App->mainwin->ui_keyGroups->clear();
	for(unsigned i=0;i<keyGroups.size();i++)
	{
		App->mainwin->ui_keyGroups->add_leaf((const char*)keyGroups[i]->name);
	}
	App->mainwin->ui_keyGroups->child(keyGroups.size()-1)->set_selected();
	App->mainwin->ui_keyGroups->set_changed();
	App->mainwin->ui_keyGroups->redraw();
}
void AppGraphViewer::makeTrack()
{
	if(getKeyGroupNum()==-1) {
		printf("no keyGroup selected\n");
		return;
	}
	GsString name;
	KeyGroup* k;

		if(emptyTrack) //runs through automatically at the begining
	{
		emptyTrack = false;
		name.set("default_track");
		k = keyGroups[0];
	}
	else
	{
		fl_string_input("Track Name","please enter a name for the track",name);
		gsout << name << " was entered\n";
		k = getKeyGroup();
	}
	GsString nm;
	nm<<name<<"_"<< k->name;

	App->mainwin->ui_TrackList->add_leaf(nm);
	App->mainwin->ui_TrackListNla->add_leaf(nm);
	Track* t = new Track(nm,k);
	if(emptyTrack)
		t->IKTrack = true;
	else
		t->IKTrack=App->mainwin->ui_ikChannel->value();

	for(unsigned i=0;i<k->jnts.size();i++)
	{
		for(unsigned j = 0; j< k->jnts[i]->dof.size();j++)
		{
			t->cvs.push_back( new Curve( t,k->sk->joint(k->jnts[i]->name),k->jnts[i]->dof[j] ) );
		}
	}

	App->nla->addTrack(t);
	App->timeslider->timeline = t;
	

}


int AppGraphViewer::getKeyGroupNum()
{
	int sel = -1;
	int children = App->mainwin->ui_keyGroups->children();
	for(int i=0;i<children;i++)
	{
		if( App->mainwin->ui_keyGroups->child(i)->selected() ){sel=i;}
	}
	return sel;
}
KeyGroup* AppGraphViewer::getKeyGroup()
{
return keyGroups[getKeyGroupNum()];
}

void AppGraphViewer::showGroup()
{

	int sel = getKeyGroupNum();


	if(sel != -1)
	{
		App->mainwin->ui_channelList->clear();
		for(unsigned i=0;i< keyGroups[sel]->jnts.size(); i++)
		{
			App->mainwin->ui_channelList->add_leaf(keyGroups[sel]->jnts[i]->name);
		}
	}
	
}
void AppGraphViewer::draw ()
 {
   std::vector<GsVec>& p = c->p;
   std::vector<GsVec>& cv = c->c;
   std::vector<Curve*> cvs; 
   bool drawC = false;
   Track* t = App->timeslider->timeline;
	   if(t) { drawC = true; cvs = t->cvs;}
//glFinish();
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
   glFlush();
   glPushMatrix();
   glLoadIdentity();
 //glScaled(1.5,1,1);
   
  
drawGrid();
   if(drawC)
   {
	   for(unsigned int it=0;it<cvs.size();it++)
	   {
		   if(cvs[it]->vis) drawCurve(cvs[it]);
	   }
   }
drawCurve(c);

	glDrawString ( message1, mseGraph.x,mseGraph.y + 0.04f , GsColor::blue );
	glDrawString ( message2, mseGraph.x,mseGraph.y+  0.005f, GsColor::blue );


 // Draw selection as a circle of radius pickprec:
	drawCircle(pickprec,mseGraph);
//drawCircle(0.2, mseGraph);
glPopMatrix();

   //----- Fltk will then automatically flush and swap buffers -----
   	fltk::GlWindow::redraw();
 
}




int AppGraphViewer::handle ( int ev )
{std::vector<GsVec>& p = c->p;
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
	if(drw) {
		draw();
		//App->mainwin->event(ev); //ui_glwindow->handle(ev);
	}

   return fltk::GlWindow::handle(ev);
 }

