
# include <gsim/fl.h>
# include <gsim/fl_vars_win.h>
# include <gsim/gs_model.h>
#include <gsim/kn_ik_manipulator.h>
# include "app_main_win.h"
# include <sstream>
#include "draw_primitives.h"
#include "particle_ik.h"

# define CHKVAR(s) if(_vars->search(s)<0) { fltk::alert("Missing parameter [%s]!",s); exit(0); }
char* value = new char[12];
float Ang;
double otx,oty,otz,orx,ory,orz;
int enc;
#define ENC 209

#define fwd -1
#define rev 1
struct jntInfo
{
char* name;int dof;int dir;
};
jntInfo jntI[] = 
{
		{"RightUpLegBase",1,rev},
		{"RightUpLeg",2,rev},
		{"RightUpLeg",0,fwd},
		{"RightLeg",0,rev},
		{"RightFootBase",0,rev},
		{"RightFootBase",2,fwd},
		{"RightShoulder",0,rev},
		{"RightUpArm",1,rev},
		{"RightUpArmRoll",2,fwd},
		{"RightForeArm",0,fwd},
		{"LeftUpLegBase",1,rev},
		{"LeftUpLeg",2,rev},
		{"LeftUpLeg",0,rev},
		{"LeftLeg",0,fwd},
		{"LeftFootBase",0,fwd},
		{"LeftFootBase",2,fwd},
		{"LeftShoulder",0,fwd},
		{"LeftUpArm",1,rev},
		{"LeftUpArmRoll",2,fwd},
		{"LeftForeArm",0,rev},
		{"LegBase",0,fwd},
		{"Neck",1,fwd},
		{"Head",0,fwd},
		{"Head",2,fwd},
		{"RightHandThumb1",1,fwd},
		{"RightHand",2,fwd},
		{"LeftHandThumb1",1,rev},
		{"LeftHand",2,rev}
		
};

AppMainWin::AppMainWin ()
 {
   _vtwin = new FlVarsWin;
   otx=oty=otz=orx=ory=orz=0;
   ikCreated = 0;
 }

AppMainWin::~AppMainWin ()
 {
 }

hoap_posture AppMainWin::getPosture()
{
	hoap_posture p;
	// = (short int*)malloc(sizeof(short int)*numDOF);
	for(int i = 0;i<numDOF;i++)
	{
		p.mot[i] = (short int)ui_enc[i]->value();
	}
	//printf("leds: ");

///////right eyes
	if(ui_E[0]->value())
	   p.Eyes[0] = (unsigned short int)4097;
	else
	   p.Eyes[0] = (unsigned short int)4096;

	if(ui_E[1]->value())
	  p.Eyes[1] = (unsigned short int)8193;
	else
	  p.Eyes[1] = (unsigned short int)8192;

	if(ui_E[2]->value())
	  p.Eyes[2] = (unsigned short int)12289;
	else
	 p.Eyes[2] = (unsigned short int)12288;

	if(ui_E[3]->value())
	  p.Eyes[3] = (unsigned short int)16385;
	else
	  p.Eyes[3] = (unsigned short int)16384;

	/*if(ui_E[4]->value())
	  p.Eyes[4] = (unsigned short int)32768;
	else
	  p.Eyes[4] = (unsigned short int)32769;
	  */


	///////////left eyes

	if(ui_E[5]->value())
	  p.Eyes[5] = (unsigned short int)4097;
	else
	 p.Eyes[5] = (unsigned short int)4096;

	if(ui_E[6]->value())
	  p.Eyes[6] = (unsigned short int)8193;
	else
	  p.Eyes[6] = (unsigned short int)8192;

	if(ui_E[7]->value())
	  p.Eyes[7] = (unsigned short int)12289;
	else
	 p.Eyes[7] = (unsigned short int)12288;

	if(ui_E[8]->value())
	  p.Eyes[8] = (unsigned short int)16385;
	else
	  p.Eyes[8] = (unsigned short int)16384;

	if(ui_E[9]->value())
	  p.Eyes[9] = (unsigned short int)32768;
	else
	  p.Eyes[9] = (unsigned short int)32769;

	  p.hands[0]=(unsigned short int)ui_Ang[25]->value();
	  p.hands[1]=(unsigned short int)ui_Ang[27]->value();
	  p.hands[2]=(unsigned short int)ui_Ang[24]->value();
	  p.hands[3]=(unsigned short int)ui_Ang[26]->value();

	  p.head[0] = (unsigned short int)ui_Ang[21]->value();
	  p.head[1] = (unsigned short int)ui_Ang[22]->value();
	  p.head[2] = (unsigned short int)ui_Ang[23]->value();

	return p;
}
rt_msg lastMesg;
void AppMainWin::sendPosture()
{
	if(App->hoap_client->isConnected())
	{
	rt_msg msg;
	msg = getMesg();
	msg.command = ADD_TO_QUEUE_TASK;
	App->hoap_client->sendMessage(&msg);
	}
	else
	{
				rt_msg this_msg;
				this_msg = getMesg();
				int i;
				int maxEnc = 0;
				int numSteps = 0;
				short int deltaMotors[21];
				double addMotors[21];
				for (i=0; i < 21; i++)
				{   //determine the number of ticks between start and goal
					deltaMotors[i] = (short int)(this_msg.posture.mot[i] - lastMesg.posture.mot[i]);
					//determine the maximum difference
					if(abs(deltaMotors[i])>maxEnc) maxEnc = (short int)abs(deltaMotors[i]); 
				}

				//all motors have the same number of steps between start and goal
				numSteps = (int)(maxEnc / this_msg.stepSize);

				//determine the amount to change encoders for each iteration
				for(i=0;i<21;i++)
				{
					if(abs( deltaMotors[i] ) < this_msg.stepSize)
					{
						//if the deltamotors is smaller then stepsize then just set it on the last iteration
						addMotors[i]=0;
					}
					else
					{
						//otherwise the amount to increase the motors each step is the total change over the number of steps
						addMotors[i] = ((double)deltaMotors[i])/numSteps;
					}
				}
				printf("num steps: %d\n",numSteps);
				lastMesg = this_msg;
		//message("hoap not connected");
	}
}
int AppMainWin::getSelectedKey()
{
 
	for(int i = 0; i<ui_keylist->size();i++)
			{
			if(ui_keylist->selected(i))
				{
				return i;
				}
			}
	return -1;

}
void AppMainWin::updateSliders()
{
		for(int i=0;i<21;i++) //only update main motors
		{	
			double v = (double)App->sk->joint(jntI[i].name)->euler()->value(jntI[i].dof) ;
			ui_Ang[i]->value(gs_todeg(v));
			ui_enc[i]->value((int)(gs_todeg(v)*ENC*jntI[i].dir));
		}
}

void AppMainWin::matchIK()
{
	if(ikCreated)
	{
		match_ik_manipulators(ui_viewer->_ikmanips);
		
		App->viewer->_RootIK->mat().set(App->sk->joint("Hips")->gmat());
		App->viewer->updateIKSkel();
		
		
	}
}
rt_msg AppMainWin::getMesg()
{
	rt_msg mesg;
	mesg.posture = getPosture();
	mesg.autostart = ui_autoStart->value();
	mesg.clearQueue = 0;
	mesg.end = 0;
	mesg.reply = (int)ui_motorReply->value();
	mesg.stepSize = (int)ui_intstep->value();
	mesg.task = 0;
	mesg.period = (int)ui_rtCycleTime->value();
	return mesg;
}
void AppMainWin::orientIK()
{
	if( ui_viewer->_ikmanips)
	{
		SnGroup* g =  ui_viewer->_ikmanips;
		KnIkManipulator* leftFootIK;
		KnIkManipulator* rightFootIK;
		int  count=0;
		GsVec p1,p2;

		//larm, rarm, lleg, rleg
		GsVec p;
		App->ikskel->update_global_matrices();
		p.set(App->ikskel->joint("BodyIKJoint")->gcenter());// App->ikskel->joint("BodyIKJoint")->gmat().getrans(p);
		ui_viewer->_RootIK->translation(p); 

		App->sk->joint("Hips")->pos()->value( ui_viewer->_RootIK->translation());
		ui_viewer->_RootIK->update();

		p1 = App->ikskel->joint("lLegIKJoint")->gcenter(); //.getrans(p1);
		leftFootIK = (KnIkManipulator*)g->get(2);	
		leftFootIK->mat().setrans(p1); //->translation(p1);
		leftFootIK->update();

		p2 = App->ikskel->joint("rLegIKJoint")->gcenter(); //gmat().getrans(p2);
		rightFootIK = (KnIkManipulator*)g->get(3);	
		rightFootIK->mat().setrans(p2);
		rightFootIK->update();
		
		ui_viewer->update_robot();

	}
}
void AppMainWin::showLog(hoap_data_log log)
{

	App->viewer->_sense_lines->remove_all();
	SnLines* footLines = new SnLines();
	footLines->color(GsColor(0,255,255));

	SnLines* lineX = new SnLines();
	lineX->color(GsColor(255,0,0));
	SnLines* lineY = new SnLines();
	lineY->color(GsColor(0,255,0));
	SnLines* lineZ = new SnLines();
	lineZ->color(GsColor(0,0,255));

	printf("log num:%d \n",log.queueNum);
	float scale = 50;
	//gsout<<"skel enc: ";
	for(int i=0;i<21;i++)
	{
	//	gsout<<(float)(log.motor[i]/ENC*jntI[i].dir)<<"  ";
		App->ssk->joint(jntI[i].name)->euler()->value(jntI[i].dof,gs_torad((float)(log.motor[i]/ENC*jntI[i].dir)));
		ui_renc[i]->value(log.motor[i]);
	}

	for(int i=0;i<8;i++)
	{
		ui_sense[i]->value((double)log.fsr[i]);
	}
	for(int i=0;i<6;i++)
	{
		ui_sense[i+8]->value((double)log.acc_gyro[i]);
	}
	App->ssk->update_global_matrices();
//gsout<<gsnl;
//right foot front right
	footLines->push(App->ssk->joint("rfpfr")->gcenter()); 
	footLines->push(App->ssk->joint("rfpfr")->gcenter()+GsVec(0, log.fsr[0]/scale, 0) );
//right foot front left
	footLines->push(App->ssk->joint("rfpfl")->gcenter());
	footLines->push(App->ssk->joint("rfpfl")->gcenter()+GsVec(0, log.fsr[1]/scale, 0) );
//right foot back right				
	footLines->push(App->ssk->joint("rfpbr")->gcenter());
	footLines->push(App->ssk->joint("rfpbr")->gcenter()+GsVec(0, log.fsr[2]/scale, 0) );
//right foot back left
	footLines->push(App->ssk->joint("rfpbl")->gcenter());
	footLines->push(App->ssk->joint("rfpbl")->gcenter()+GsVec(0, log.fsr[3]/scale, 0) );
//left  foot front right				
	footLines->push(App->ssk->joint("lfpfr")->gcenter());
	footLines->push(App->ssk->joint("lfpfr")->gcenter()+GsVec(0, log.fsr[4]/scale, 0) );
//left  foot front left
	footLines->push(App->ssk->joint("lfpfl")->gcenter());
	footLines->push(App->ssk->joint("lfpfl")->gcenter()+GsVec(0, log.fsr[5]/scale, 0) );
//left  foot back right				
	footLines->push(App->ssk->joint("lfpbr")->gcenter());
	footLines->push(App->ssk->joint("lfpbr")->gcenter()+GsVec(0, log.fsr[6]/scale, 0) );
//left  foot back left
	footLines->push(App->ssk->joint("lfpbl")->gcenter());
	footLines->push(App->ssk->joint("lfpbl")->gcenter()+GsVec(0, log.fsr[7]/scale, 0) );

	//accel
	lineY->push(App->ssk->joint("Hips")->gcenter());
	lineY->push(App->ssk->joint("Hips")->gcenter()+GsVec(0, log.acc_gyro[1]/scale , 0) );

	lineY->push(App->ssk->joint("Hips")->gcenter());
	lineY->push(App->ssk->joint("Hips")->gcenter()+GsVec(0, log.acc_gyro[4]/scale , 0) );

	lineX->push(App->ssk->joint("Hips")->gcenter());
	lineX->push(App->ssk->joint("Hips")->gcenter()+GsVec(log.acc_gyro[0]/scale, 0 , 0) );

	lineX->push(App->ssk->joint("Hips")->gcenter());
	lineX->push(App->ssk->joint("Hips")->gcenter()+GsVec(log.acc_gyro[3]/scale, 0 , 0) );

		lineZ->push(App->ssk->joint("Hips")->gcenter());
	lineZ->push(App->ssk->joint("Hips")->gcenter()+GsVec(0, 0 , log.acc_gyro[2]/scale) );

	lineZ->push(App->ssk->joint("Hips")->gcenter());
	lineZ->push(App->ssk->joint("Hips")->gcenter()+GsVec(0, 0 , log.acc_gyro[5]/scale) );


App->viewer->_sense_lines->add(lineX);
App->viewer->_sense_lines->add(lineY);
App->viewer->_sense_lines->add(lineZ);
App->viewer->_sense_lines->add(footLines);
App->viewer->update_robot();
App->viewer->redraw();

}
void AppMainWin::show ()
 {
  
   ui_viewer->init (  );
   ui_graph_win->init( );
   ui_nla_win->init();

 ui_foot_win->init();

   _vtwin->vars ( App->vars );
   message ( "Ready" );
  
   App->viewer->_knsc->set_visibility(0,ui_showScene->value(),0,0);

	const GsArray<KnJoint*>& jnts = App->sc->joints();
	for(int i=0;i<jnts.size();i++)
	{
		// printf("joint %d is %s\n",i,(const char*)jnts[i]->name());
		ui_objects->add((const char*)jnts[i]->name());
		ui_objectList->add((const char*)jnts[i]->name());
	}
	ui_window->show();

	//ui_foot_window->show();
	KnPosture* p = new KnPosture(App->sk);  //FIX THIS !!!
	p->get();
	App->timeslider->timeline->connect(p);
	App->timeslider->timeline->IKTrack = false;

	KnPosture* p1 = new KnPosture(App->ikskel);
	p1->get();
	App->timeslider->IKtimeline->connect(p1);
	App->timeslider->IKtimeline->IKTrack = true;
	
 }


void AppMainWin::event ( AppEvent e )
 {
	char str[30] ;
	HmHumanoid* sk = App->sk;
	KnSkeleton* sc = App->sc;
	HmScene* hms = ui_viewer->_kns;
	float ob[3];
	std::vector<const char*> objects;
	int numObj;
	FILE* opf;
	float kt = 0;
	
   switch ( e )
    { 
   case evUpdateSprings:

	   App->pIK->stiffness = (float)ui_sikStiffness->value();
	   App->pIK->handStiffness = (float)ui_sikHandStiffness->value();
	   App->pIK->damping = (float)ui_sikDamp->value();
	   App->pIK->grav = (float)ui_sikGrav->value();
	   App->pIK->stepSize = (float)ui_sikStepSize->value();
	   break;
   case evPIKReset:
	   App->pIK->reset();
	   break;
   case evPIKRun:
	   App->pikRunning = !App->pikRunning;
	   break;
   case evMakeWipe:
		App->makeWipe();
	   break;
   case evWipe:
	   {
		   App->taskFrame = 0;
		   App->playTask = !App->playTask;

	   }break;

   case evResetJacobian:
	   {
		   App->jacobian->reset();
	   }
	   break;
   case evJacobianCreate:
	   App->jacobian->reset();
	   break;
		case evEval:
	   App->jacobian->evaluate();
	break;
   case evRun:
	   App->runJacobian = !App->runJacobian;
	   break;
   case evReset:
	   App->sk->init_values();
	   ui_viewer->_knsc->update();
	   ui_viewer->redraw();
	   break;

   case evControlStop:
	   App->cont->_stop = true;
	   break;
   case evStartControl:
	   App->cont->start();
	   break;
   case evControl:
	   App->cont->updateParams();
	   if(!App->cont->_running) App->cont->evaluate((float)ui_cont_time->value());
	   
	   break;
   case evFrameToggle:
	   App->timeslider->frameTog = ui_frameTog->value();
	   if(ui_frameTog->value()) ui_endFrame->value(200);
	   else ui_endFrame->value(10);

	   App->timeslider->draw();
	   break;
   case evStream:
	   App->stream = ui_stream->value();
	   break;
   case evTimeline:
	   ui_timeline_window->show();
	   break;
   case evLegBaseAngle:
	   {
	   float baseAngle = (float)ui_legBaseAngle->value();
	
	//	App->sk->joint("Hips")->pos()->value(2,baseAngle/5.0f);
		App->sk->joint("Hips")->euler()->value(0,gs_torad(baseAngle));
		App->sk->joint("LegBase")->euler()->value(0,-gs_torad(baseAngle));
updateIK();
	   }break;
		case evLoadConfig:
		App->load_config();
		break;
		case evSaveConfig:
		App->save_config();
		break;

   case evShowScene:
	   {
		   int shw = ui_showScene->value();
	   App->viewer->_knsc->set_visibility(0,shw,0,0);
	   }break;
   case	evResetWalk:
	   if(App->walkMade)App->walk->reset();else message("walk not made.. try addGoal");
	   break;
case	evDrawTraj:
	if(App->walkMade)
	{
		App->walk->drawTraj = 1;
		App->walk->playWalk();
		App->walk->drawTraj = 0;
	}
	else message("walk not made.. try addGoal");

		break;
   case evPauseWalk:
	   if(App->walkMade)
	   {
		   if(App->walk->pause){ App->walk->pause = 0; App->walk->resume = 1; App->walk->playWalk(); }
		   else{
			   App->walk->pause = 1;
		   }
		   message("paused");
	   }else message("walk not made.. try addGoal");
	   break;
   case evViconHello:
	   App->vicon_client->SendHello();
	   break;
   case evLoadMotion:
	   {
			GsString chc="solution.sm";
			fl_string_input("load motion","load",chc);
			GsString loadm = "../data/";
			loadm<<chc;
			gsout<<loadm<<gsnl;

			App->timeslider->timeline->init();
			if(App->timeslider->timeline->load(loadm))gsout<<"loaded"<<gsnl;
			else gsout<<"not loaded"<<gsnl;
			App->timeslider->timeline->connect(App->sk);
			gsout<<App->timeslider->timeline->frames()<<"frames loaded"<<gsnl;
	   }
	   break;
   case evGetData:
		App->hoap_client->getData();
	   break;
   case evStartQueue:
	  gsout<<"evStartQueue doesnt do anything\n";// App->hoap_client->sendStartQueue((int)ui_intstep->value());
	   break;
   case evSetStart:
	   {
		   if(App->walkMade)
		   {
			   GsVec p = App->ssk->joint("Hips")->pos()->value();
			   float a  = App->ssk->joint("Hips")->euler()->value(1);

				
				setIK(a,p,ROOT_IK);
				p.y=0;
				App->walk->goals[App->walk->curGoal-1]->Rotate(a);
				App->walk->goals[App->walk->curGoal-1]->Translate(p);
				if(App->walk->dcdtMade)
					App->walk->search();
				else
				{
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
				App->mainwin->matchIK();
		   }
	  
	   } break;
   case evSetEnd:
	   { 
		   if(App->walkMade)
		   {
		   GsVec p = App->sc->joint("wand")->offset();
			p.y =  App->sk->joint("Hips")->gcenter().y;
			float a = gs_todeg(App->sc->joint("wand")->euler()->value(1));

		   Curve* cv = App->walk->cvs[App->walk->curCv];
		   int gi = App->walk->curGoal;
		   App->walk->_GoalManip->translation(p);
		   p.y=0;
			App->walk->goals[gi]->Rotate(a);
			
			App->walk->goals[gi]->Translate(p);
			cv->p[3] = p;
			

			
			GsVec leng = App->walk->_Goal2In->translation() - p;
			float l =  leng.len();
			GsVec dp = App->walk->goals[gi]->pos - App->walk->goals[gi-1]->pos;
			//if(l>dp.len()/2) 
				l=dp.len()/2;
			
			GsVec tg = GsVec( l*sin(gs_torad(a)) ,0, l*cos(gs_torad(a)));
			App->walk->_Goal2In->translation(p-tg);
			cv->p[2] = p-tg;
		
		if(App->walk->dcdtMade)
			App->walk->search();
		else
		{
			drawCurve(cv);
			App->walk->updateSteps();
		}
		App->mainwin->matchIK();
	   }
	   }break;
   case evExportMesh:
	   if(App->walkMade){if(App->walk->dcdtMade) App->walk->exportMesh();}
	   break;
   case evMoveRoot:
	   App->moveRoot = App->mainwin->ui_moveRoot->value();
	   break;
   case evViconConnect:
	      App->vicon_client->Startup();
	 //  App->vicon_client->Connect();
	   break;
	case evQuit:          gs_exit(); break;
	case evFlatten:       if(App->walkMade)App->walk->flatten(); break;
	case evSearch:       if(App->walkMade)if(App->walk->dcdtMade) App->walk->search(); break;
	case evRealTime:      App->realtime = ui_realtime->value(); break;
//Walk Gen Events
	case evPlayWalk:   
		if(App->walkMade)
		{
			if(App->walk->pause)App->walk->resume = 1;  
			App->walk->pause=0; 
			App->walk->playWalk(); 
		}
		else message("cant play because a walk has not been made.. try addGoal");
		
		break;
	case evMakeWalk:	 if(App->walkMade) App->walk->playWalk(true);
				else message("cant bake because a walk has not been made.. try addGoal");
						 break;
	case evMakeSteps:

	   if(!App->walkMade) 
	   {  
		   App->walk = new WalkGenerator(App->viewer->_steps); 
	   }
	    if(App->goalMade) App->walk->updateSteps();
	   if(!ikCreated)ui_viewer->makeIK();

	   //App->walk->makeSteps();
   break;

	case evHipReset:    if(App->walkMade) App->walk->resetHip(); else message("a walk has not been made.. try addGoal");  break;
	case evHipFwd:		if(App->walkMade)App->walk->adjustHip(0); else message("a walk has not been made.. try addGoal");  break;
	case evHipRev:		if(App->walkMade)App->walk->adjustHip(1); else message("a walk has not been made.. try addGoal");  break;
	case evHipLeft:		if(App->walkMade)App->walk->adjustHip(2); else message("a walk has not been made.. try addGoal");  break;
	case evHipRight:	if(App->walkMade)App->walk->adjustHip(3); else message("a walk has not been made.. try addGoal");  break;
	case evHipUp:		if(App->walkMade)App->walk->adjustHip(4); else message("a walk has not been made.. try addGoal");  break;
	case evHipDown:		if(App->walkMade)App->walk->adjustHip(5); else message("a walk has not been made.. try addGoal");  break;
	case evClearGoals:  if(App->walkMade) App->walk->clearGoals(); else message("a walk has not been made.. try addGoal");  break;
	case evAddGoal:			   
		if(!App->walkMade) 
	   {  
		   App->walkMade = true;
		   App->walk = new WalkGenerator(App->viewer->_steps);
	   }
	   if(!ikCreated)ui_viewer->makeIK();
	   App->walk->addGoal(); break;


	case evLand:          App->sk->land(); matchIK(); break;
	case evIntStep:       App->intStep = (int)ui_intstep->value(); break;
//Foot window evets
	case evFoot: App->foot->clearCurves(); break;

//NLA EVENTS
	case evTrackList:     ui_nla_win->trackList(); break;
	case evInsertTrack:   ui_nla_win->insertTrack();break;
	case evLoadTracks:    ui_nla_win->loadTracks(); break;
	case evShowTrack:     ui_nla_window->show();  break;
//GRAPH EDITOR EVENTS
	case evMakeTrack:     App->graph->makeTrack();  break;
	case evFreeTangent:   ui_graph_win->freeTangent();  break;
	case evChannelSel:    ui_graph_win->channelSel();   break;
	case evLoadChannels:  ui_graph_win->loadChannels(); break;
	case evMakeGroup:     ui_graph_win->makeGroup();  //do next case to show
	case evKeyGroups:     ui_graph_win->showGroup(); break;
	case evGraphView:     ui_graph_window->show(); break;


//TIMESLIDER EVENTS
	case evClearKeys:   App->nla->getTrack()->init(); App->timeslider->draw(); break;
	case evPlay:          App->timeslider->play(); break;
	case evSetIK:         ui_viewer->makeIK();  break;
	
//CLIENT EVENTS
	case evEyes:          sendPosture(); break; //App->hoap_client->sendPostureQueue(getPosture(),(int)ui_autoStart->value(),(int)ui_motorReply->value()); break;
	case evHello:         App->hoap_client->sendHello(); break;
	case evSendPos:       sendPosture(); break;
	case evGetPos:      gsout<<"evGetPos does nothing\n";//  App->hoap_client->getPosture(); break;
	case evTimeSlider:   App->timeslider->draw(); break;
case evResetIK: 
	App->sk->init_values(); 
	App->sk->land();
	   for(unsigned i=0;i<9;i++)
	   {
		    ui_IK[i]->value(0);
	   }
	matchIK();
	   break;
  
      case evEditParameters:
       { 
         _vtwin->update ();
         _vtwin->show ();
       } break;


      case evZero: 
       { 
	   	if (!sk) break;
		for(int i=0;i<28;i++)
		{	
			ui_Ang[i]->value(0);
			sk->joint(jntI[i].name)->euler()->value(jntI[i].dof,0);
			ui_enc[i]->value(0);
		}
		ui_Ang[21]->value(60);
		ui_enc[21]->value(60);
		ui_Ang[22]->value(316);
		ui_enc[22]->value(316);
		ui_Ang[23]->value(572);
		ui_enc[23]->value(572);
		ui_Ang[24]->value(572);
		ui_enc[24]->value(572);
		ui_Ang[25]->value(60);
		ui_enc[25]->value(60);
		ui_Ang[26]->value(828);
		ui_enc[26]->value(828);
		ui_Ang[27]->value(316);
		ui_enc[27]->value(316);

		if(App->realtime)
			{
				sendPosture(); 
			}
		matchIK();

       } break;

	  case evViewPref:
		  hms->set_visibility(ui_skeleton->value(),ui_visgeo->value(),0,ui_jointaxis->value());
		  hms->view_com(ui_com->value());
		  hms->view_comparts(ui_part_com->value());
		  hms->view_support(ui_support->value());
		  //ui_test_collisions->value();
		  hms->update();
		 

		  ui_viewer->draw();
		  break;
	 
	  case evJntSlide:
			if (!sk) break;
				
			for(int i=0;i<21;i++)
			{	

				Ang = (float)ui_Ang[i]->value();
				//printf("name:%s dof: %d\n",jntI[i].name,jntI[i].dof);
				sk->joint(jntI[i].name)->euler()->value(jntI[i].dof,GS_TORAD(Ang));
				
				if(ui_actValues->value() && App->hoap_client->isConnected())
					gsout<<"ui_actValues does nothing\n"; //ui_enc[i]->value(App->ps->p.mot[i]);
				else
					ui_enc[i]->value(Ang*ENC*jntI[i].dir);
			}
			ui_enc[25]->value(ui_Ang[25]->value()); //*ENC*jntI[21].dir);
			ui_enc[27]->value(ui_Ang[27]->value()); //*ENC*jntI[21].dir);

			if(App->realtime)
			{
				sendPosture(); 
			}
			matchIK();
			

		break;
		case evConnect:
			
			if( ! App->hoap_client->isConnected() )
			{
				App->hoap_client->setIp ("192.168.1.105");
				App->hoap_client->start();
			}
			else
			{
				App->hoap_client->disconnect();
				message("disconected");
			}

			break;
		case evHead: //21,22,23 head joints

				//Ang = (float)ui_Ang[21]->value();
				//sk->joint(jntI[21].name)->euler()->value(jntI[21].dof,GS_TORAD(Ang));
				
			ui_enc[21]->value(ui_Ang[21]->value()); //*ENC*jntI[21].dir);
			ui_enc[22]->value(ui_Ang[22]->value()); //*ENC*jntI[21].dir);
			ui_enc[23]->value(ui_Ang[23]->value()); //*ENC*jntI[21].dir);
			break;
		case evLthumb:
			if (!sk) break;
			Ang = (float)ui_Ang[26]->value();
			ui_enc[26]->value(ui_Ang[26]->value());
			sk->joint("LeftHandThumb1")->euler()->value(1,GS_TORAD(Ang)/3);
			sk->joint("LeftHandThumb2")->euler()->value(1,GS_TORAD(Ang)/3);
			sk->joint("LeftHandThumb3")->euler()->value(1,GS_TORAD(Ang)/3);
		break;

		case evRthumb:
			if (!sk) break;
			Ang = (float)ui_Ang[24]->value();
			ui_enc[24]->value(ui_Ang[24]->value());
			sk->joint("RightHandThumb1")->euler()->value(1,GS_TORAD(Ang)/3);
			sk->joint("RightHandThumb2")->euler()->value(1,GS_TORAD(Ang)/3);
			sk->joint("RightHandThumb3")->euler()->value(1,GS_TORAD(Ang)/3);
		break;

		case evIKSlide: if(ikCreated)App->viewer->IKSlide(); break;
		case evAddObj: message("add object does nothing\n"); break;
		
		case evDelObj:
			message("delete object\n");
			numObj = ui_objectList->size();
			objects.clear();
			for(int i = 0; i<numObj;i++)
			if(ui_objectList->selected(i))
			{
				objects.push_back(ui_objectList->child(i)->label());
				//printf("%s selected\n",ui_objectList->child(i)->label());
			}
			sc->joint(objects[0])->visgeo()->unref();
			ui_viewer->_knsc->connect(NULL);

//delete(sc->joint(objects[0]));
		break;

		case objMove:
			numObj = ui_objectList->size();
			objects.clear();
			for(int i = 0; i<numObj;i++)
			if(ui_objectList->selected(i))
			{
				objects.push_back(ui_objectList->child(i)->label());
				//printf("%s selected\n",ui_objectList->child(i)->label());
			}

			otx = ui_objX->value();
			oty = ui_objY->value();
			otz = ui_objZ->value();
			orx = gs_torad(ui_objrX->value());
			ory = gs_torad(ui_objrY->value());
			orz = gs_torad(ui_objrZ->value());

			sc->joint(objects[0])->offset( GsVec ( (float)otx,(float)oty,(float)otz) );
			sc->joint(objects[0])->euler()->value(0,(float)orx);
			sc->joint(objects[0])->euler()->value(1,(float)ory);
			sc->joint(objects[0])->euler()->value(2,(float)orz);


			//printf("%g %g %g %g %g %g\n",otx,oty,otz,orx,ory,orz);
			if(App->walkMade)
			{
				if(App->walk->dcdtMade)
				{
					App->walk->flatten();
					App->walk->search();
				}
			}

		break;
		case evObjSelected:
			objects.clear();
			numObj = ui_objectList->size();

			for(int i = 0; i<numObj;i++)
			if(ui_objectList->selected(i))
			{
				objects.push_back(ui_objectList->child(i)->label());
				//printf("%s selected\n",ui_objectList->child(i)->label());
			}
		
			
			sc->joint(objects[0])->offset().get(ob);
			ui_objX->value(ob[0]);
			ui_objY->value(ob[1]);
			ui_objZ->value(ob[2]);

		break;


		case evIKKey:
			App->timeslider->ikKey();
			//add keyframe name to channel list
			sprintf(str,"ikKey %g", time_slider->currentTime); 
			 printf("str is %s\n",str);
			ui_keylist->add((const char*)str);

	   break;
		case evSet:
			 time_slider->jointKey();
			//add keyframe name to channel list
			sprintf(str,"jointKey %g", time_slider->currentTime); 
			// printf("str is %s\n",str);
			ui_keylist->add((const char*)str);
		break;
		case evInsert:
		if(fl_value_input("enter","enter key time",kt))
		{
			 time_slider->jointKey(kt);
		} 
		  sprintf(str,"keyframe %g",kt); 
		  printf("str is %s\n",str);
		  ui_keylist->add((const char*)str);
			break;
	
		case evMoveKey:
			 time_slider->move=! time_slider->move;
			 time_slider->moveKey = getSelectedKey();

			time_slider->timeline->keytime(0,kt);
		break;
		case evKeyMenu:
			//int num =  time_slider->timeline->frames();
		numObj = ui_keylist->size();

			for(int i = 0; i<numObj;i++)
			{
			if(ui_keylist->selected(i))
				{
				printf( "key %d selected %s\n",i,ui_objectList->child(i)->label() );
				time_slider->appMotionFunc((int)time_slider->timeline->keytime(i),0);
				}
			}
	 
			break;

		case evSavePos:
			 opf = fopen("posture.csv","w");
			fprintf(opf,"2,2,");
			for(int i=0;i<21;i++)
			{
				fprintf(opf,"%d,",(short int)ui_enc[i]->value());
				printf("%d,",(short int)ui_enc[i]->value());
			}
			fprintf(opf,"60,60,R,R,R,R");
			fclose(opf);
		break;
		case evArmIKActive:
			App->armIKActive = ui_arm_ik_active->value();
			break;
		
		

    }
			ui_viewer->update_robot();
			ui_viewer->redraw();
			time_slider->redraw();
 }

