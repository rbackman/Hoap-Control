//#define SERVER_IP "192.168.1.105" 


# include <gsim/fl.h>
# include <gsim/fl_output_win.h>
# include <gsim/kn_joint_pos.h>
# include "app_main.h"
# include "app_main_win.h"

#include "vicon_client.h"

#include <gsim/gs_euler.h>

#include "draw_primitives.h"
#include "particle_ik.h"

//# define GS_USE_TRACE1  // keyword tracing
# include <gsim/gs_trace.h>


# define CHKVAR(s) if(vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }





#include <iostream>
#include <string>
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>


# ifdef _WIN32
# include <Windows.h>
# pragma warning(disable : 4996)
# endif
AppMain* App; // the only instance of M3dMain
AppMain::AppMain ()
 {

   mainwin = new AppMainWin();
   viewer =  mainwin->ui_viewer;
   vars = new GsVars;

	controlMade = false;
   armIKActive = 0;
   const char* cfgfile = "../src/config.txt";
   read_config_file ( cfgfile );
   CHKVAR("hoapfile");
   ssk = new HmHumanoid;

   sk = new HmHumanoid;
   sc = new KnSkeleton;
   ikskel = new KnSkeleton();
   KnJoint* rt = ikskel->add_joint(KnJoint::TypeQuat,0,"IKRoot");
stream = 0;
	walkMade = false;
	goalMade = false;
	realtime = false;
	moveRoot = false;
   //create a skeleton for ik animation.
    ikskel->add_joint(KnJoint::TypeQuat, rt,"BodyIKJoint");
	ikskel->add_joint(KnJoint::TypeQuat, rt,"LeftHandIKJoint");
	ikskel->add_joint(KnJoint::TypeQuat, rt,"RightHandIKJoint");
	ikskel->add_joint(KnJoint::TypeQuat, rt,"lLegIKJoint");
	ikskel->add_joint(KnJoint::TypeQuat, rt,"rLegIKJoint");
		

	ikskel->joint("lLegIKJoint")->pos()->thaw();
	ikskel->joint("lLegIKJoint")->quat()->activate(); //rot()->activate();
	ikskel->joint("lLegIKJoint")->rot()->euler()->thaw(); 
	ikskel->joint("lLegIKJoint")->rot()->activate(); 
	
	ikskel->joint("rLegIKJoint")->pos()->thaw();
	ikskel->joint("rLegIKJoint")->quat()->activate();
	ikskel->joint("rLegIKJoint")->rot()->activate();
	ikskel->joint("rLegIKJoint")->rot()->euler()->thaw();

	ikskel->joint("BodyIKJoint")->pos()->thaw();
	ikskel->joint("BodyIKJoint")->rot()->activate();
	ikskel->joint("BodyIKJoint")->rot()->euler()->thaw();
	ikskel->joint("BodyIKJoint")->quat()->activate();

	ikskel->make_channels();

//KnPosture* pst = new KnPosture(ikskel);
//pst->get();
//gsout<<"ikskel channels: ";
//pst->output(gsout,1,1);

	ikskel->update_global_matrices();



	intStep=5;

	 bool ok = ssk->load ( vars->gets("hoapfile") );
   if ( !ok ) gsout.fatal("Could not load hoap sense file!");
   ssk->land();

   ok = sk->load ( vars->gets("hoapfile") );
   if ( !ok ) gsout.fatal("Could not load hoap file!");
   sk->land();
  

   ok = sc->load(vars->gets(vars->gets("startScene")));
   if ( !ok ) gsout.fatal("Could not load scene file!");

  

   timeslider = mainwin->time_slider;
  
   leftFootStart = sk->joint("LeftFootBase")->gcenter();
   rightFootStart = sk->joint("RightFootBase")->gcenter();
   rootStart = sk->joint("Hips")->gcenter();
   rootStart.y=0;

	cont = new WalkControler();

	/*
	JIKmanip = new SnManipulator();
	JIKobstacle = new SnManipulator();
	JIKmanip->child(new SnModel(sk->joint("LeftHand")->visgeo()));
//	GsModel* bx = new GsModel;
//	bx->make_sphere(GsPnt(sk->joint("LeftHand")->pos()->value()),3,30,1);
//	JIKmanip->child(new SnModel(bx));
	//JIKmanip->child(new SnModel(sk->joint("LeftHand")->visgeo()));
	maxHipHeight = sk->joint("Hips")->gcenter().y;
	JIKobstacle->child(new SnModel(sk->joint("LeftHand")->visgeo()));
	JIKobstacle->visible(0);
	viewer->_root->add(JIKmanip);
	viewer->_task_lines->add(taskLines = new SnLines());
	//viewer->_root->add(JIKobstacle);
*/

	pIK = new ParticleIK();
	pIK->numParticles = 20;
	pIK->randomChain();
	//pIK->targetSkel(sk->joint("Hips"),sk->joint("LeftHand"));

	pIK->stiffness = (float)mainwin->ui_sikStiffness->value();
	pIK->damping = (float)mainwin->ui_sikDamp->value();
	pIK->grav = (float)mainwin->ui_sikGrav->value();
	pIK->hoapArm(sk);
	pIK->sikManip->child(new SnModel(sk->joint("LeftHand")->visgeo()));
	pIK->sikManip->translation(sk->joint("LeftHand")->gcenter());
	viewer->_root->add(pIK->sikManip);
	viewer->_root->add(pIK->spheres);
	viewer->_root->add(pIK->lines);
	pikRunning = false;
    runJacobian = false;
	playTask = false;

 }

	void AppMain::load_config()
	{
		const char* cfgfile = "../src/config.txt";
		read_config_file ( cfgfile );
		CHKVAR("hoapfile");


		App->mainwin->ui_stream->value(vars->getb("stream"));
		App->stream = vars->getb("stream");
		App->mainwin->ui_numStepInterp->value((double)vars->geti("interps"));
		App->mainwin->ui_StepFPS->value((double)vars->geti("fps"));
		App->mainwin->ui_rtCycleTime->value((double)vars->getf("rtCycle")); 
		App->mainwin->ui_turnAdjustAng->value((double)vars->getf("turnAdjustAngle"));
		App->mainwin->ui_turnAdjustDist->value((double)vars->getf("turnAdjustDist"));
		App->mainwin->ui_stepHeight->value((double)vars->getf("footHeight"));
		App->mainwin->ui_hipsHeight->value((double)vars->getf("hipsHeight"));
		App->mainwin->ui_stepDistance->value((double)vars->getf("maxStepDist"));
		App->mainwin->ui_channelRad->value((double)vars->getf("channelRad"));
		App->mainwin->ui_pathRad->value((double)vars->getf("pathRad"));
		App->mainwin->ui_pathdangle->value((double)vars->getf("dangle"));
		App->mainwin->ui_jointaxis->value(vars->getb("jointAxis"));
		App->mainwin->ui_visgeo->value(vars->getb("visgeo"));
		App->mainwin->ui_skeleton->value(vars->getb("skeleton"));
		App->mainwin->ui_part_com->value(vars->getb("partsCom"));
		App->mainwin->ui_com->value(vars->getb("partsCom"));
		App->mainwin->ui_support->value(vars->getb("support"));
		App->mainwin->ui_test_collisions->value(vars->getb("collisions"));
		App->mainwin->ui_arm_ik_active->value(vars->getb("armIK"));
		App->armIKActive = vars->getb("armIK");
		App->mainwin->ui_legBaseAngle->value((double)vars->getf("baseAngle"));
//		App->mainwin->ui_serverLocal->value(vars->getb("local"));
		App->mainwin->ui_motorReply->value(vars->getb("reply"));
		App->mainwin->ui_realtime->value(vars->getb("realtime"));
		App->realtime = vars->getb("realtime");
		App->mainwin->ui_intstep->value((double)vars->geti("stepsize"));

		if(!App->mainwin->ikCreated)
		{
			if(vars->getb("createIK")) App->viewer->makeIK();
			App->armIKActive = App->mainwin->ui_arm_ik_active->value();
		}
		fltk::check();
		App->viewer->redraw();

	
	}
	void AppMain::wipe()
	{

	}
	void AppMain::setTaskFrame(float taskFrme)
{

	float rad = (float)mainwin->ui_wipeRadius->value();
	GsVec r(wipeCenter.x + rad*cos(GS_TORAD(taskFrame)),wipeCenter.y ,wipeCenter.z + rad*sin(GS_TORAD(taskFrame)));
	App->JIKmanip->translation(r);
	}

	void AppMain::makeWipe()
{
	taskLines->color(GsColor(255,0,0));
	taskLines->init();
	float rad = (float)mainwin->ui_wipeRadius->value();
	int n=20;
		GsVec pos = App->JIKmanip->translation();
	wipeCenter = pos;

	 float a=0.0, da = (float)(2*3.1415/n);
	 taskLines->begin_polyline();
      while ( n-- )
         { 
			 GsVec r(rad*cos(a),0,rad*sin(a));
			 taskLines->push(GsVec( pos.x + r.x,pos.y , pos.z + r.z));
			 a+=da;
         }
	taskLines->push(GsVec( pos.x + rad, pos.y , pos.z));
	taskLines->end_polyline();
}
	void AppMain::save_config()
	{
		GsOutput f;
		f.open("../src/config.txt");
		AppMainWin* m = App->mainwin;
f<<"# config file\n\n";
f<<"parameters\n{\n\n";
f<<"#skeleton files for hoap and scene \n";

f<<"	hoapfile = \"../../models/hoap/hoap.hm\"; \n"; 
f<<"	viconscenefile = \"../../models/hoap/environment.s\";\n"; 
f<<"	scenefile = \"../../models/hoap/Scene.s\";\n";
f<<"	startScene = \"viconscenefile\";\n";	

f<<"\n#walk parameters"<<gsnl;
f<<"	turnAdjustAngle = "<< m->ui_turnAdjustAng->value()<<";\n";
	
f<<"	turnAdjustDist = "<< m->ui_turnAdjustDist->value()<<";\n";
f<<"	footHeight = "<<m->ui_stepHeight->value()<<";\n";
f<<"	hipsHeight = "<<m->ui_hipsHeight->value()<<";\n";
f<<"	maxStepDist = "<<m->ui_stepDistance->value()<<";\n";
f<<"	channelRad = "<<m->ui_channelRad->value()<<";\n";
f<<"	pathRad = "<<m->ui_pathRad->value()<<";\n";
f<<"	dangle = "<<m->ui_pathdangle->value()<<";\n";
f<<"	realtime = "<<m->ui_realtime->value()<<";\n";
f<<"	stepsize = "<<m->ui_intstep->value()<<";\n";
f<<"	baseAngle = "<<m->ui_legBaseAngle->value()<<";\n";
f<<"	interps = "<<m->ui_numStepInterp->value()<<";\n";
f<<"	fps = "<<m->ui_StepFPS->value()<<";\n";
f<<"	rtCycle = "<<m->ui_rtCycleTime->value()<<";\n";

f<<"\n#draw parameters\n";
f<<"	jointAxis = "<<m->ui_jointaxis->value()<<";\n";
f<<"	visgeo = "<<m->ui_visgeo->value()<<";\n";
f<<"	skeleton = "<<m->ui_skeleton->value()<<";\n";
f<<"	partsCom = "<<m->ui_part_com->value()<<";\n";
f<<"	COM = "<<m->ui_com->value()<<";\n";
f<<"	support = "<<m->ui_support->value()<<";\n";
f<<"	collisions = "<<m->ui_test_collisions->value()<<";\n";
f<<"	armIK = "<<m->ui_arm_ik_active->value()<<";\n";

f<<"\n#net parameters\n";
//f<<"	local = "<<m->ui_serverLocal->value()<<";\n";
f<<"	reply = "<<m->ui_motorReply->value()<<";\n";
f<<"	createIK = "<<1<<";\n";
f<<"	stream = "<<m->ui_stream->value()<<";\n";

f<<"\n}\n\nend";
f.close();
		
	}

int AppMain::getPosture(float* posture)
{
	 const GsArray<KnJoint*>& jnts = App->sk->joints();
	 posture = (float*)malloc(sizeof(float)*jnts.size());

	 int size = jnts.size();
	 for(int i=0;i<numDOF;i++)
	 {
		 posture[i] = jnts[i]->euler()->value(0);
		 printf("%g \n",posture[i]);
	 }
	 return size;
}
void AppMain::read_config_file ( const char* cfgfile )
 {
   GsInput in;
   if ( !cfgfile ) cfgfile = fltk::file_chooser ( "Enter Configuration File", "*.txt", 0 );
   if ( !cfgfile ) return;
   if ( !in.open(cfgfile) ) { fltk::alert("Could not open cfg file!"); return; }

   in.commentchar ( '#' );
   in.lowercase ( false );

   while ( true )
    { if ( in.get()==GsInput::End ) break;
    
      GS_TRACE1 ( "Token: "<<in.ltoken() );

      if ( in.ltoken()=="parameters" )
       { GsVars parms;
         in >> parms;
         vars->merge ( parms );
       }
      else if ( in.ltoken()=="end" )
       { break;
       }
    }

  GS_TRACE1("End Parsing");
 }





void set( int obj_num, GsVec obj_pos, GsQuat obj_rot )
 {
	 if ( obj_num >=1 && obj_num <= 9)
	 {
		 GsString _name;
		 _name <<"obj" << obj_num;
		 if ( obj_pos != GsVec(0,0,0) )
		 {
			 App->sc->joint(_name)->pos()->value(obj_pos);
			 App->sc->joint(_name)->quat()->value(obj_rot);
			 if(obj_num==9)
			 {	App->ssk->joint("Hips")->init_values(); 
				App->ssk->update_global_matrices();
				App->sc->update_global_matrices();

				GsMat mt =  App->ssk->joint("head")->gmat().inverse()*App->sc->joint(_name)->child(0)->gmat();
				//gsout<<mt<<gsnl;
				
				GsQuat qt;
				qt.set(mt,'C');
				//gsout<<qt<<gsnl;
				App->ssk->joint("hips")->rot()->value(qt);
				App->ssk->joint("hips")->pos()->value(&mt.e41);
				App->viewer->update_robot();
			 }
			// gsout << "obj" << obj_num <<" " << obj_pos << ", " << obj_rot << gsnl;
		 }
		// else App->sc->joint(_name)->pos()->value( GsVec(0,-20,0) );
	 }
	 if(App->walkMade)
	 {
		 if(App->walk->dcdtMade)
		 {
			 App->walk->flatten();
			 App->walk->search();
		 }
	 }
 }

void AppMain::checkVicon()
{
	netTime = RakNet::GetTime();
	if(vicon_client->isConnected)
	{
		  vicon_client->Update(App->netTime);

		  set( vicon_client->obj_num, vicon_client->pos_data, vicon_client->ori_data);
		  viewer->redraw();
		  RakSleep(5);
	}

}
void AppMain::checkHoap()
{
	netTime = RakNet::GetTime();
	if( hoap_client->isConnected()) 
	{
		//gsout<<"connected\n";
		hoap_client->update ( App->netTime );
      RakSleep(5);
	}
}
int main ( int argc, char** argv )
 {
	# ifdef _WIN32
   AllocConsole();
   freopen("conin$", "r", stdin);
   freopen("conout$", "w", stdout);
   freopen("conout$", "w", stderr);
   # endif
//      char ch;
   bool quit = false;




   // Init the application:
   App = new AppMain;

   App->vicon_client = new ViconClient;
   App->hoap_client = new HoapClient;

 
   // Setup parameters and show the window:
   App->mainwin->show();

    App->load_config();
   // Enter fltk loop:
	App->wipeFrameTime = get_time();
 //  App->jacobian = new Jacobian;
   while(1)
    {
		App->netTime = RakNet::GetTime();
		App->checkVicon();
		App->checkHoap();
		if(App->pikRunning)
		{
			App->pIK->evaluateHoapArm();
		}
		if(App->stream)
		{
			App->mainwin->sendPosture();
		}
		App->pIK->applyToHoapArm();
	/*	if(App->playTask)
		{
			if(get_time() - App->wipeFrameTime > 1.0/30.0)
			{
				App->wipeFrameTime = get_time();
				App->taskFrame++;
				if(App->taskFrame>360)
				{	App->taskFrame=0;
					App->playTask=false;
				}
				else
				{
				App->setTaskFrame(App->taskFrame);
				}
			}
		}
		if(App->runJacobian)
		{
			App->jacobian->evaluate();
			 for(int i = 2;i<4;i++) //0&1 are hands 2&3 are legs
			{ 
			   get_ik_manipulator( App->mainwin->ui_viewer->_ikmanips , i)->update();
			}
		}*/
		fltk::check();
    }


   return 0;
 }

