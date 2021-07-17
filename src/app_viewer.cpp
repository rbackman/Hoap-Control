
# include <gsim/gs_event.h>
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>
# include <gsim/fl_skeleton_win.h>
# include <gsim/sn_model.h>
# include <gsim/kn_ik_manipulator.h>
# include <gsim/gs_mat.h>
# include "app_viewer.h"
# include "app_main_win.h"
# include <fltk/gl.h>
#include "utilities.h"
#include "draw_primitives.h"
#include <gsim/sa_render_mode.h>

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }



AppViewer::AppViewer ( int x, int y, int w, int h, const char *l ) : FlViewer ( x, y, w, h, l )
 {
   // we build here an example scene graph for polygon edition
	

		_root = new SnGroup;
		_knss = new HmScene;



		_kns = new HmScene;
		_knsc = new KnScene;
		_iknsc = new KnScene;

		drawCvs = false;

		SnGroup* balance = new SnGroup;
		balance->add(_coms = new SnGroup);
		balance->add(_com = new SnGroup);
		balance->add(_support = new SnGroup);
		_root->add(balance);

		_root->add(_task_lines = new SnGroup());
		_root->add(_traj_lines = new SnGroup());
		_root->add(_steps = new SnGroup);
		_root->add(_sense_lines = new SnGroup());
		_root->add(_lines = new SnGroup());
		_root->add(_iknsc);
		_root->add(_kns);
		_root->add(_knss);
		_root->add(_knsc);



   FlViewer::root ( _root );
// FlViewer::cmd ( FlViewer::CmdAxis );
 }

AppViewer::~AppViewer ()
 {

 }



int AppViewer::handle_keyboard(const GsEvent &e)
{
	switch(e.key)
{
case GsEvent::KeySpace:
   App->mainwin->message("viewer space hit\n");
	break;
case 'z':
	view_all();
	camera().eye.z = 1;
	break;
}
	return FlViewer::handle_keyboard(e);
}

static void IkManipCb ( KnIkManipulator* ikm, void* udata )
 {
   AppViewer* win = (AppViewer*)udata;

   // update selection text message:
   const char* desc[] = { "Left Arm", "Right Arm", "Left Leg", "Right Leg" };
   int i = ikm->ikbodyid();
   
   App->mainwin->updateSliders();
   
   GsVec p1,p2;
	if(i==2){
		App->sk->joint("LeftFootBase")->gmat().getrans(p1);
		App->ikskel->joint("lLegIKJoint")->pos()->value(p1) ;
	}
	if(i==3)
	{
		App->sk->joint("RightFootBase")->gmat().getrans(p2);
		App->ikskel->joint("rLegIKJoint")->pos()->value(p2);
	}
   App->viewer->updateIKSlide();
  

 }
static void RootManipCb ( SnManipulator* manip, const GsEvent& ev, void* udata )
 {
   AppViewer* win = (AppViewer*)udata;
  // App->sk->joint("Hips")->gmat() = App->mainwin->ui_viewer->_RootIK->mat();
   GsVec p = App->mainwin->ui_viewer->_RootIK->translation();
   App->sk->joint("Hips")->pos()->value(p);
   App->sk->joint("Hips")->quat()->value(App->mainwin->ui_viewer->_RootIK->rotation());
   float a =  App->sk->joint("Hips")->euler()->value(1);
int start = 2;
if(App->armIKActive)start = 0;


   for(int i = start;i<4;i++) //0&1 are hands 2&3 are legs
   { 
		KnIkManipulator* ikm = get_ik_manipulator( App->mainwin->ui_viewer->_ikmanips , i);
 		   if(App->moveRoot)
		   {

			 //  ikm->match();
		   }
			else ikm->update();
			
   }
 		   if(App->moveRoot)
		   {

			   GsVec lft = App->leftFootStart; //
			   GsVec rft = App->rightFootStart; //GsVec(p.x + App->rightFootStart.x,App->rightFootStart.y,p.z + App->rightFootStart.z );
			   lft.roty(a);
			   rft.roty(a);
			   setIK(gs_todeg(a),GsVec(p.x + lft.x, lft.y, p.z + lft.z ),LEFT_IK);
			   setIK(gs_todeg(a),GsVec(p.x + rft.x, rft.y, p.z + rft.z ),RIGHT_IK);


		   }

   if(App->walkMade)
   {
	   App->walk->goals[App->walk->curGoal-1]->Rotate(a);
	   App->walk->goals[App->walk->curGoal-1]->Translate(p);

	   if(App->walk->dcdtMade)
			App->walk->search();
		else
		{
			p.y=0;
			App->walk->cvs[App->walk->curCv]->p[0] = p;
			drawCurve(App->walk->cvs[App->walk->curCv]);
			App->walk->updateSteps();
			
			GsVec leng = App->walk->_Goal1Out->translation() - p;
			float l =  leng.len();
			GsVec dp = App->walk->goals[App->walk->curGoal]->pos - App->walk->goals[App->walk->curGoal-1]->pos;
			//if(l>dp.len()/2) 
				l=dp.len()/2;
			GsVec tg = GsVec( l*sin(a) ,0, l*cos(a));
			App->walk->_Goal1Out->translation(p+tg);
			App->walk->cvs[App->walk->curCv]->p[1] = p + tg;
	   }
   }
   App->mainwin->updateSliders();
  //  App->viewer->updateIKSkel();
	App->viewer->updateIKSlide();
		   

 }
void AppViewer::updateIKSkel()
{
	App->sk->update_global_matrices();
	GsVec p,p1,p2;

	_RootIK->mat().getrans(p);
	App->ikskel->joint("BodyIKJoint")->pos()->value(p);

	App->sk->joint("LeftFootBase")->gmat().getrans(p1);
	App->ikskel->joint("lLegIKJoint")->pos()->value(p1) ;
	App->sk->joint("RightFootBase")->gmat().getrans(p2);
	App->ikskel->joint("rLegIKJoint")->pos()->value(p2);

	//printf("left leg x:%g\n",App->ikskel->joint("lLegIKJoint")->pos()->valuex());

 App->ikskel->update_global_matrices();
 App->mainwin->ui_viewer->update_robot();



}
int AppViewer::makeIK()
{

   if (!App->sk) return 0;
   if(!App->mainwin->ikCreated)
   {
   App->mainwin->ikCreated = 1;

	_RootIK = new SnManipulator();
	_RootIK->callback(RootManipCb,this);
	_RootIK->child ( new SnModel(App->sk->joint("Hips")->visgeo()) );
	App->sk->update_global_matrices();


//	_root->add(_RootIK);

    // create new ik
    App->hikb = new KnIkBody;
    App->hikb->ref();

	App->hikb->init_hoap_ik ( App->sk ); 
    App->hikb->capture_init_posture();
    App->hikb->timer_activation ( true );

    if ( _root )
     { 

		  _root->add(_ikmanips = new SnGroup); 
		   add_ik_manipulators ( _ikmanips, App->hikb, App->mainwin->ui_viewer->_kns , IkManipCb, this, 0); //_coldet:0 );
		App->hikb->add_lines ( _ikmanips ); 
	  
		//((KnIkManipulator*)_ikmanips->get(0))->visible(false);
		//((KnIkManipulator*)_ikmanips->get(1))->visible(false);
	}
	
	// App->sk->update_global_matrices();

	App->hikb->lfoot().getrans(App->ikStart[0]); //App->sk->joint("LeftFootBase")->gcenter());
	App->hikb->rfoot().getrans(App->ikStart[1]); // App->ikStart[1].set(App->sk->joint("RightFootBase")->gcenter());  
	App->ikStart[2].set(App->sk->joint("Hips")->gcenter());

	_RootIK->translation(App->ikStart[2]);
	_iknsc->set_visibility(1,0,0,0);
   }
   else
	   App->mainwin->message("IK already exists");
//IKSlide();
    return 1;

}
void AppViewer::IKSlide()
{
	bool rot = App->mainwin->ui_ikrot->value();
	//if(rot)gsout<<"Rotate Radio\n"; 
	//else gsout<<"Translate Radio\n";
	//for(int i=0;i<9;i++){gsout<< App->mainwin->ui_IK[i]->value()<<"\n";}
	
	if(rot)
	{float scale = 9;
	App->ikskel->joint("lLegIKJoint")->euler()->value( 0,(float)App->mainwin->ui_IK[0]->value()*scale);
	App->ikskel->joint("lLegIKJoint")->euler()->value( 1,(float)App->mainwin->ui_IK[1]->value()*scale);
	App->ikskel->joint("lLegIKJoint")->euler()->value( 2,(float)App->mainwin->ui_IK[2]->value()*scale);

	App->ikskel->joint("rLegIKJoint")->euler()->value(0,(float)App->mainwin->ui_IK[3]->value()*scale);
	App->ikskel->joint("rLegIKJoint")->euler()->value(1,(float)App->mainwin->ui_IK[4]->value()*scale);
	App->ikskel->joint("rLegIKJoint")->euler()->value(2,(float)App->mainwin->ui_IK[5]->value()*scale);
	
	App->ikskel->joint("BodyIKJoint")->euler()->value(0,(float)App->mainwin->ui_IK[6]->value()*scale);
	App->ikskel->joint("BodyIKJoint")->euler()->value(1,(float)App->mainwin->ui_IK[7]->value()*scale);
	App->ikskel->joint("BodyIKJoint")->euler()->value(2,(float)App->mainwin->ui_IK[8]->value()*scale);

	App->ikskel->update_global_matrices();
	
	gsout<<"bodyIK joint euler "<<	App->ikskel->joint("BodyIKJoint")->euler()->value(0)<<"\n";
		}
		else
	{
/*	setIK(GsVec(
		(float)App->mainwin->ui_IK[0]->value(),
		(float)App->mainwin->ui_IK[1]->value(),
		(float)App->mainwin->ui_IK[2]->value()
		),LEFT_IK);

	setIK(GsVec(
		(float)App->mainwin->ui_IK[3]->value(),
		(float)App->mainwin->ui_IK[4]->value(),
		(float)App->mainwin->ui_IK[5]->value()
		),RIGHT_IK);
	setIK(GsVec(
		(float)App->mainwin->ui_IK[6]->value(),
		(float)App->mainwin->ui_IK[7]->value(),
		(float)App->mainwin->ui_IK[8]->value()
		),ROOT_IK);
	*/

		/*App->ikskel->joint("lLegIKJoint")->pos()->value( 0,(float)App->mainwin->ui_IK[0]->value());
	App->ikskel->joint("lLegIKJoint")->pos()->value( 1,(float)App->mainwin->ui_IK[1]->value());
	App->ikskel->joint("lLegIKJoint")->pos()->value( 2,(float)App->mainwin->ui_IK[2]->value());

	App->ikskel->joint("rLegIKJoint")->pos()->value(0,(float)App->mainwin->ui_IK[3]->value());
	App->ikskel->joint("rLegIKJoint")->pos()->value(1,(float)App->mainwin->ui_IK[4]->value());
	App->ikskel->joint("rLegIKJoint")->pos()->value(2,(float)App->mainwin->ui_IK[5]->value());
	
	App->ikskel->joint("BodyIKJoint")->pos()->value(0,(float)App->mainwin->ui_IK[6]->value());
	App->ikskel->joint("BodyIKJoint")->pos()->value(1,(float)App->mainwin->ui_IK[7]->value());
	App->ikskel->joint("BodyIKJoint")->pos()->value(2,(float)App->mainwin->ui_IK[8]->value());
	*/

	//App->ikskel->update_global_matrices();
	}

	//App->mainwin->orientIK();

}
void AppViewer::updateIKSlide()
{
	bool rot = App->mainwin->ui_ikrot->value();
/*	if(rot)gsout<<"Rotate Radio\n"; 
	else gsout<<"Translate Radio\n";
*/	
		App->ikskel->update_global_matrices();

	if(rot)
	{float scale = 9;
	App->ikskel->joint("lLegIKJoint")->euler()->value( 0,(float)App->mainwin->ui_IK[0]->value()*scale);
	App->ikskel->joint("lLegIKJoint")->euler()->value( 1,(float)App->mainwin->ui_IK[1]->value()*scale);
	App->ikskel->joint("lLegIKJoint")->euler()->value( 2,(float)App->mainwin->ui_IK[2]->value()*scale);

	App->ikskel->joint("rLegIKJoint")->euler()->value(0,(float)App->mainwin->ui_IK[3]->value()*scale);
	App->ikskel->joint("rLegIKJoint")->euler()->value(1,(float)App->mainwin->ui_IK[4]->value()*scale);
	App->ikskel->joint("rLegIKJoint")->euler()->value(2,(float)App->mainwin->ui_IK[5]->value()*scale);
	
	App->ikskel->joint("BodyIKJoint")->euler()->value(0,(float)App->mainwin->ui_IK[6]->value()*scale);
	App->ikskel->joint("BodyIKJoint")->euler()->value(1,(float)App->mainwin->ui_IK[7]->value()*scale);
	App->ikskel->joint("BodyIKJoint")->euler()->value(2,(float)App->mainwin->ui_IK[8]->value()*scale);

		}
		else
	{
	App->mainwin->ui_IK[0]->value(App->ikskel->joint("lLegIKJoint")->pos()->value( 0 ));
	App->mainwin->ui_IK[1]->value(App->ikskel->joint("lLegIKJoint")->pos()->value( 1 ));
	App->mainwin->ui_IK[2]->value(App->ikskel->joint("lLegIKJoint")->pos()->value( 2 ));

	App->mainwin->ui_IK[3]->value(App->ikskel->joint("rLegIKJoint")->pos()->value( 0 ));
	App->mainwin->ui_IK[4]->value(App->ikskel->joint("rLegIKJoint")->pos()->value( 1 ));
	App->mainwin->ui_IK[5]->value(App->ikskel->joint("rLegIKJoint")->pos()->value( 2 ));
	
	App->mainwin->ui_IK[6]->value(App->ikskel->joint("BodyIKJoint")->pos()->value( 0 ));
	App->mainwin->ui_IK[7]->value(App->ikskel->joint("BodyIKJoint")->pos()->value( 1 ));
	App->mainwin->ui_IK[8]->value(App->ikskel->joint("BodyIKJoint")->pos()->value( 2 ));

	}

}

void AppViewer::init (  )
 {
   App->viewer = this;
   if ( _kns->size()==0 )
    {
		_kns->connect(App->sk);  
    }
      if ( _knss->size()==0 )
    {
		_knss->connect(App->ssk);  
    }
	  	SaRenderMode rm(gsRenderModeLines);
		rm.apply(_knss);


	  	const GsArray<KnJoint*> jnts = App->ssk->joints();
		GsMaterial mt;
		mt.diffuse.set(255,0,0);
		for(int i=0;i<jnts.size();i++)
		{
			if(jnts[i]->visgeo()!=NULL)
			{
				jnts[i]->visgeo()->M.setall(mt);
				
			}
		}

      if ( _knsc->size()==0 )
    { 
		_knsc->connect(App->sc);
    }
	  if(_iknsc->size()==0)
	  {
	  _iknsc->connect(App->ikskel);
	  }

	  
   FlViewer::view_all();
 }

#include "draw_primitives.h"

void drawGsCircle(GsVec pos,float rad,SnGroup* gp)
{
	   SnLines* circle = new SnLines();
		   int n=20;
	 float a=0.0, da = (float)(2*3.1415/n);
	 circle->begin_polyline ();
      while ( n-- )
         { 
			 GsVec r(rad*cos(a),0,rad*sin(a));
			 circle->push(GsVec( pos.x + r.x, 0 , pos.z + r.z));
			 a+=da;
         }
		circle->push(GsVec( pos.x + rad, 0 , pos.z));
      circle->end_polyline();
	  gp->add(circle);
}
void AppViewer::draw ()
 { 
	 if(App->hoap_client->isConnected()&&App->mainwin->ui_motorReply->value())
	 {
		 App->viewer->_knss->set_visibility(0,1,0,0);
		 App->viewer->_knss->update();
	 }
	 else
	 {
		 App->viewer->_knss->set_visibility(0,0,0,0);
		 App->viewer->_knss->update();
	 }

	// if ( !valid() ){
			glViewport ( 0, 0, w(), h() );
			glEnable ( GL_DEPTH_TEST );
			glCullFace ( GL_BACK );
			glFrontFace ( GL_CCW );
			glPointSize ( 6 );
			glLineWidth ( 2 );
	// }
	 
	 	glClearColor ( 1, 1, 1, 1 );
	glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glMatrixMode(GL_MODELVIEW);
	
	glDisable(GL_LIGHTING);
   if(drawCvs){
	   _lines->remove_all();
	   for(unsigned i=0;i< App->walk->cvs.size(); i++)
	   {
		   Curve* cv = App->walk->cvs[i];
		   
		   SnLines* line = new SnLines();
		   
		   line->begin_polyline ();
		   for(unsigned j=0;j<cv->c.size();j++)
		   {	
			   // line->C.push(GsColor(1,0,0));
			   line->push(cv->c[j]);
			
		   }
		   line->end_polyline();
		   _lines->add(line);
	   }
	   glLineWidth ( 5 );
	   for(unsigned i=0;i<App->walk->goals.size();i++)
	   {
		   drawGsCircle(App->walk->goals[i]->pos,12,_lines);	
	   }
	   
	   glLineWidth ( 2 );
   }
  
if(App->walkMade)
{
   if(App->walk->dcdtMade)
   {
	   
	      // draw edges:
   static GsArray<GsPnt2> cedges;
   static GsArray<GsPnt2> ucedges;
//   App->walk->dcdtobj->get_mesh_edges ( &cedges, &ucedges );

   // draw non-constrained edges:
   int i;
			
   if(App->mainwin->ui_showTri->value())
   {
   SnLines* triLine = new SnLines();
		  triLine->color(GsColor(0,255,255));
		   //triLine->begin_polyline ();
		  
		   for(int j=0;j<cedges.size();j++)
		   {	
			   GsPnt2 pf = cedges.get(j);
			   triLine->push(pf.x,0,pf.y);
		   }
		  // triLine->end_polyline();
		   _lines->add(triLine);
   }
   if(App->mainwin->ui_showConst->value())
   {
		   SnLines* constLines = new SnLines();
constLines->color(GsColor(255,255,0));
			//constLines->begin_polyline ();
			
		   for(int j=0;j<ucedges.size();j++)
		   {	
			   // line->C.push(GsColor(1,0,0));
			    GsPnt2 pf = ucedges.get(j);
			   constLines->push(pf.x,0,pf.y);
		   }
		 // constLines->end_polyline();
		   _lines->add(constLines);
   } 
		  
		  

		   if(App->mainwin->ui_showPath->value())
		   {
			    GsPolygon path = App->walk->path;
			   SnLines* pathLines = new SnLines();
				pathLines->color(GsColor(255,0,0));
				
				pathLines->begin_polyline ();
			
			   for(int j=0;j<path.size();j++)
			   {	
				   
					GsPnt2 pf = path.get(j);
				   pathLines->push(pf.x,0,pf.y);
			   }
			  pathLines->end_polyline();
			   _lines->add(pathLines);
		   }
		   if(App->mainwin->ui_showChannel->value())
		   {
				GsPolygon channel = App->walk->channel;
				SnLines* chanLines = new SnLines();
				chanLines->color(GsColor(0,0,255));
				chanLines->begin_polyline ();

			   for(int j=0;j<channel.size();j++)
			   {	
					GsPnt2 pf = channel.get(j);
				   chanLines->push(pf.x,0,pf.y);
			   }
			  chanLines->end_polyline();
			   _lines->add(chanLines);
		   }
		   if(App->mainwin->ui_showFloorPoly->value())
		   {
			   for(i=0;i<(int)App->walk->floorPoly.size();i++)
			   {
				   SnLines* line = new SnLines();
				   line->color(GsColor(0,255,0));
			   GsPolygon p = App->walk->floorPoly[i];
				 line->begin_polyline ();
				for(int j=0;j<p.size();j++)
				{
					line->push(p.get(j).x,0,p.get(j).y);
				}
				line->push(p.get(0).x,0,p.get(0).y);
				 line->end_polyline ();
				  _lines->add(line);
			   }
		   }
   }
}
   glEnable(GL_LIGHTING);
   render();

	update_robot();

	App->mainwin->updateSliders();



   FlViewer::draw ();
 }

int AppViewer::handle_scene_event ( const GsEvent &e )
 {
   // window events can be custom-processed here:
   if ( e.button1 )
    {
    }

   // now let the viewer process the remaining events:
   return FlViewer::handle_scene_event ( e );
 }

