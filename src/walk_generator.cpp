#include <gsim/sn_transform.h>
#include "walk_generator.h"
#include "app_main_win.h"
#include "utilities.h"
#include <gsim/gs_euler.h>
#include <gsim/kn_ik_manipulator.h>
#include "draw_primitives.h"

#include <gsim/gs.h>
#include <gsim/gs_array.h>

#include <gsim/gs.h>


static inline float sBump ( float t ) // f([0,1])->[0,1]
 {
  return 1.0f - pow ( 2.0f*t - 1, 6.0f ); //exponent has to be pair: 2 (low extremity speed), 4, 6, 8 (quicker), ...
 }


void WalkGenerator::init()
{
	App->walkMade = true;
	dcdtMade = false;
	playing = false;
	clearSteps();
	leftfootMod = new GsModel();
	leftfootMod->load("../../models/hoap/LeftFootstep.m");
	rightfootMod = new GsModel();
	rightfootMod->load("../../models/hoap/RightFootstep.m");
	rootMod = App->sk->joint("Hips")->visgeo();
	pause = 0;
	resume = 0;
	lastH = 0;
	drawTraj = false;
	footOffsetDist = App->sk->joint("LeftFootBase")->gcenter().y;
	gsout<< footOffsetDist<<gsnl;
	leftFootStart = App->sk->joint("LeftFootBase")->gcenter();
	rightFootStart = App->sk->joint("RightFootBase")->gcenter();
}
void WalkGenerator::clearSteps()
{
	leftSteps.clear();
	rightSteps.clear();
	rootSteps.clear();
	adjust.clear();
	stepGroup->remove_all();
}
void WalkGenerator::getPoints ( GsModel* m, GsPolygon& pol ,GsMat mat)
{
	 if(! m->empty()){
		 int i;
		 GsPnt p;
		 GsPolygon polz;
	
		
		 for ( i=0; i<m->V.size(); i++ )
		  { p = m->V[i]*mat ;
		    //offst.roty(angle);
		   // p.roty(ang);
				//GsVec p2 = offst + p;
				//p2.roty(angle);
		 //if ( GS_NEXTZ(p.y,200.0f) )  // if vertex is close to floor plane (z==0)
			// { 
				 polz.push ( GsVec2(p.x, p.z) );  // project to floor
			// }
		  }
		
		 if(App->mainwin->ui_floorBoundingBox->value())
		 {
			 GsVec2 max,min;

			 polz.get_bounding_box(min,max);
			
			 pol.push(GsVec2(min.x,min.y));
			 pol.push(GsVec2(min.x,max.y));
			 pol.push(GsVec2(max.x,max.y));
			 pol.push(GsVec2(max.x,min.y));
		 }
		 else
		 {
			 polz.convex_hull ( pol );  // get convex hull
		 }
	
		floorPoly.push_back(pol);

	 }
}
void WalkGenerator::exportMesh()
{
GsOutput p;
p.open("floorPoly.txt");
for(int i=0;i<(int)floorPoly.size();i++)
{
	p<<"Poly : " << i <<"  "<<floorPoly[i]<<gsnl;
}
p.close();
}
void WalkGenerator::flatten()
{
	if(!dcdtMade)
	{
	dcdtMade = true;
//	dcdtobj = new SeDcdt;
	GsModel* m = App->sc->joint("SceneRoot")->visgeo();
	getPoints(m,domain,App->sc->joint("SceneRoot")->gmat());
	}
	App->viewer->_lines->init();
	App->walk->floorPoly.clear();
	dcdtobj->init(domain,0.00001);
	
	App->sc->update_global_matrices();

	for(int i = 0;i<App->sc->joints().size();i++)
	{
					if(App->sc->joints()[i]->visgeo()==NULL)
			{
			//no geo
			}
			else if(App->sc->joints()[i]->name()=="SceneRoot"||App->sc->joints()[i]->name()=="wand")
			{
				//skip rot and wand gsout<<App->sc->joints()[i]->name()<<" not added"<<gsnl;
			}
			else
			{
			//	gsout<<App->sc->joints()[i]->name()<<gsnl;
				GsPolygon p;
				
			//	getPoints ( App->sc->joints()[i]->visgeo(), p , App->sc->joints()[i]->offset(),App->sc->joints()[i]->euler()->value(1));


				//floorPoly.push_back(p);
				dcdtobj->insert_polygon (p);
			}

		for(int j=0;j<App->sc->joints()[i]->children();j++)
		{
			if(App->sc->joints()[i]->child(j)->visgeo()==NULL)
			{
				gsout<<"jnt "<<i<<" is NULL"<<gsnl;
			}
			else
			{
				//gsout<<App->sc->joints()[i]->name()<<gsnl;
				GsPolygon p;
				gsout<<"child joint added visgeo\n";
		
	

				//GsVec ps = App->sc->joints()[i]->pos()->value();
				//ps.roty(angle);
				//gs_angles(gsEulerOrder::gsXYZ,App->sc->joints()[i]->quat()->value(),a,angle,c);

			//	gsout << angle << "is angle for joint"<< App->sc->joints()[i]->name() << gsnl ;
				getPoints (App->sc->joints()[i]->child(j)->visgeo(), p ,App->sc->joints()[i]->child(j)->gmat());
				gsout<<"poly: "<<p;

				//floorPoly.push_back(p);
				dcdtobj->insert_polygon (p);
			}
		}
	}

}



void WalkGenerator::search()
{
	
	gsout<<"check WalkGenerator::search been disabled"<<gsnl;

	/*
	GsVec bg = goals[0]->pos;
	GsVec end = goals[1]->pos;
	//gsout<<"beg: "<<bg<<gsnl;
	////gsout<<"end: "<<end<<gsnl;
	float chnRad = (float)App->mainwin->ui_channelRad->value();
	float pathRad = (float)App->mainwin->ui_pathRad->value();
	float dang = gs_torad((float)App->mainwin->ui_pathdangle->value());
   bool found =  dcdtobj->_search_path ( bg.x, bg.z, end.x, end.z, chnRad );

   if ( !found )
    { 
		App->mainwin->message ( "Endpoints cannot be connected with a free path!\n" );
     
    }
   else
    { 
		dcdtobj->_get_channel_boundary ( channel );
      dcdtobj->_get_funnel_path ( path ,pathRad,dang);
	//  gsout<<"Path:" <<path<<gsnl;
	//	gsout<<"channel:" <<channel<<gsnl;

		 Curve* c = cvs[0];
   c->c.clear();
   path.resample((float)App->mainwin->ui_stepDistance->value()/3.0f);
   for(int i=0;i<path.size();i++)
   {
	   c->c.push_back(GsVec(path.get(i).x,0,path.get(i).y));
   }
   updateSteps();

    }

*/

}

static void goalManipCB ( SnManipulator* mnp,const GsEvent& ev, void* udata )
 {
	WalkGenerator* walk = ( WalkGenerator*)udata;
	GsVec p = mnp->translation();
	int gi = walk->curGoal;
	float x,y,z;
	gs_angles_xyz(mnp->mat(),x,y,z,'c');
	Curve* cv = walk->cvs[walk->curCv];
	p.y = 0;
	if(!walk->playing){
		if(walk->dcdtMade)
		{
			walk->search();
		}
		if(strcmp("GoalIn",mnp->label())==0)
		{


			cv->p[2] = p;
		}

		else if(strcmp("GoalOut",mnp->label())==0)
		{
			/*GsVec g1p =  walk->_Goal1Out->translation() ;
			GsVec stp = walk->goals[walk->curGoal]->pos;
			GsVec leng = g1p - stp ;
			float a = walk->goals[walk->curGoal]->angle;
			float l =  leng.len();
			GsVec tg = GsVec( l*sin(gs_torad(a)) ,0, l*cos(gs_torad(a)));
			walk->_Goal1Out->translation(walk->goals[0]->pos + tg);
			cv->p[2] = p-tg;
*/
			cv->p[1] = p;
			
		}
		else if(strcmp("GoalRoot",mnp->label())==0)
		{
			
			walk->goals[gi]->Rotate(gs_todeg(y));
			walk->goals[gi]->Translate(p);
			cv->p[3] = p;
			

			
			GsVec leng = walk->_Goal2In->translation() - p;
			float l =  leng.len();
			GsVec dp = walk->goals[walk->curGoal]->pos - walk->goals[walk->curGoal-1]->pos;
			//if(l>dp.len()/2) 
				l=dp.len()/2;
			
			GsVec tg = GsVec( l*sin(y) ,0, l*cos(y));
			walk->_Goal2In->translation(p-tg);
			cv->p[2] = p-tg;

			if(App->mainwin->ui_drawTraj->value())
			{
			App->walk->drawTraj = 1;
			App->walk->playWalk();
			App->walk->drawTraj = 0;
			}

		}
		if(walk->dcdtMade)
			walk->search();
		else
		{
			drawCurve(cv);
			App->walk->updateSteps();
		}

		
		
	}
 }



void WalkGenerator::addGoal()
{
	App->mainwin->message("new goal added");
	GsVec p = App->sk->joint("Hips")->pos()->value();
	rootStartAngle = App->sk->joint("Hips")->euler()->value(1);
	curCv = goals.size()-1;
	
	if(goals.size()==0) 
	{
		
		App->goalMade = true;
		//create the starting point as a goal
		curGoal = 0;
		GsVec rootfloor(p.x,0,p.z);
		goalRightOffset =	rightFootStart - App->rootStart;
		goalLeftOffset  =	leftFootStart - App->rootStart;
		FootGoal* g = new FootGoal(rootfloor,rootStartAngle);
		g->leftFoot->model->model(leftfootMod);
		g->rightFoot->model->model(rightfootMod);
		g->leftFoot->offset.set(goalLeftOffset);
		g->rightFoot->offset.set(goalRightOffset);
		g->leftFoot->Translate(rootfloor);
		g->rightFoot->Translate(rootfloor);

		goals.push_back(g);

		//create the goal manipulator
		_Goal1Out = new SnManipulator();
		_Goal2In = new SnManipulator();
		_GoalManip = new SnManipulator();
		
		_GoalManip->child(new SnModel(App->sk->joint("Hips")->visgeo()));
		p.y = (float)App->mainwin->ui_hipsHeight->value();
		App->viewer->_RootIK->translation(p);
		_GoalManip->translation(p);
		
		_GoalManip->callback(goalManipCB,this);
		_Goal1Out->callback(goalManipCB,this);
		_Goal2In->callback(goalManipCB,this);

		_Goal2In->label("GoalIn");
		_Goal1Out->label("GoalOut");
		_GoalManip->label("GoalRoot");


		GsModel* m = new GsModel();
		m->make_sphere(GsVec(0,0,0),3,30,1);
		_Goal1Out->child(new SnModel(m));
		_Goal2In->child(new SnModel(m));

		App->viewer->_root->add(_Goal1Out);
		App->viewer->_root->add(_Goal2In);
		App->viewer->_root->add(_GoalManip);
		//App->viewer->_root->add(g->g);
		curCv=0;
		App->viewer->drawCvs = true;
	}

		Curve* cv = new Curve();
		cv->divPerPair = 10;
		cv->curveMode = BEZIER;
		
		GsVec G_Start = _GoalManip->translation();
		GsVec gdr; gdr.set(0,0,1); gdr.roty(rootStartAngle);
		GsVec G_Start_Dir = G_Start + gdr*10;
		GsVec G_End_Dir = G_Start + gdr*30;
		GsVec G_end = G_Start + gdr*40;
		GsVec F_End = G_end;
		F_End.y=0;

		_Goal1Out->translation(GsVec(G_Start_Dir.x,0,G_Start_Dir.z));
		_Goal2In->translation(GsVec(G_End_Dir.x,0,G_End_Dir.z));

		cv->p.push_back(GsVec(G_Start.x,0,G_Start.z));
		cv->p.push_back(GsVec(G_Start_Dir.x,0,G_Start_Dir.z));
		cv->p.push_back(GsVec(G_End_Dir.x,0,G_End_Dir.z));
		cv->p.push_back(GsVec(G_end.x,0,G_end.z));

		cvs.push_back(cv);
		drawCurve(cv);

		curGoal++;
		FootGoal* g2 = new FootGoal(F_End,rootStartAngle);

		g2->leftFoot->Translate(goalLeftOffset);
		g2->rightFoot->Translate(goalRightOffset);
		
		g2->Translate(F_End); // leftFoot->Translate(F_End);
		//g2->rightFoot->Translate(F_End);

		goals.push_back(g2);
		//App->viewer->_root->add(g2->g);
		//App->viewer->_root->add(g2->rightFoot->g);

		_GoalManip->translation(G_end);
		
		updateSteps();
		App->viewer->update_robot ();
		App->viewer->redraw();

}
void WalkGenerator::updateSteps()
{
	if(!playing)
	{
	clearSteps();
	//get walk parameters from UI
	float stepDis = (float)App->mainwin->ui_stepDistance->value();
	float turnAdjustAngle = (float)App->mainwin->ui_turnAdjustAng->value();
	float turnAdjustDist =  (float)App->mainwin->ui_turnAdjustDist->value();
	float hipsH = (float)App->mainwin->ui_hipsHeight->value();
	float legBaseAngle = (float)App->mainwin->ui_legBaseAngle->value();
	rootStartAngle = App->sk->joint("Hips")->euler()->value(1);

	float prevAngle;

//set hips to walk parameter value
	App->sk->joint("Hips")->pos()->value(1,hipsH);
	GsVec rootstart = App->sk->joint("Hips")->pos()->value();
	updateIK();
	App->sk->update_body_gmats();

//	leftFootStart = App->sk->joint("LeftFootBase")->gcenter();
//	rightFootStart = App->sk->joint("RightFootBase")->gcenter();

	prevAngle = gs_todeg(rootStartAngle);
	GsVec rootStartFloor = rootstart;
	rootStartFloor.y=0;

	
	//add steps for starting point
	addStep(LEFT_IK,leftFootStart,rootStartAngle);
	addStep(RIGHT_IK,rightFootStart,rootStartAngle);
	addStep(ROOT_IK,GsVec(leftFootStart.x,hipsH,leftFootStart.z),rootStartAngle);

	//set color to step models
	for(int i=0;i<leftfootMod->M.size();i++)
	{
		leftfootMod->M[i].diffuse.set(255,0,0);
	}
		for(int i=0;i<rightfootMod->M.size();i++)
	{
		rightfootMod->M[i].diffuse.set(0,255,0);
	}

	int numSteps = 1;
	for(unsigned i=0;i<cvs.size();i++)
	{
	int lastCvPt = 0;
	Curve* c = cvs[i];
	GsVec lastStep = c->c[0];
	for(unsigned j=1;j<c->c.size()-1;j++)
	{
		float dst = dist(lastStep,c->c[j]);
		float dstEnd = dist( c->c[j] , c->c[ c->c.size()-1] );
		float dstStart = dist( c->c[j],c->c[0] );

		if(j == c->c.size()-2)
		{
			goalLeftOffset =  App->leftFootStart - App->rootStart;//leftFootStart - rootStartFloor;
			goalRightOffset = App->rightFootStart - App->rootStart;//rightFootStart - rootStartFloor;
			goalLeftOffset.y = footOffsetDist;
			goalRightOffset.y = footOffsetDist;

			GsVec dpc = c->c[c->c.size()-2] - c->c[c->c.size()-3];
			dpc.y=0;
			dpc.normalize();
			float angle = gs_todeg(acos(dot(dpc,GsVec(0,0,1))));
			float xdot = dot(dpc,GsVec(1,0,0));
			if (xdot < 0)
					angle = 360 - angle;
			
			goalLeftOffset.roty(gs_torad(angle));
			goalRightOffset.roty(gs_torad(angle));
			GsVec lf = c->c[c->c.size()- 2] + goalLeftOffset;
			GsVec rf = c->c[c->c.size()- 2] + goalRightOffset;

			addStep(LEFT_IK, lf ,angle);
			addStep(RIGHT_IK,rf ,angle);
			addStep(ROOT_IK,GsVec(rf.x, hipsH,rf.z),angle );
			addStep(ROOT_IK,GsVec(lf.x, hipsH,lf.z),angle);

		}
		else if( ( dst >= stepDis && dstEnd > stepDis/2) || ( dst > stepDis/2 && lastStep == c->c[0] ) )// || j == c->c.size()-2 )
		{
			goalLeftOffset =  App->leftFootStart - App->rootStart;//leftFootStart - rootStartFloor;
			goalRightOffset = App->rightFootStart - App->rootStart;//rightFootStart - rootStartFloor;
			goalLeftOffset.y = footOffsetDist;
			goalRightOffset.y = footOffsetDist;

			numSteps++;

			lastStep = c->c[j];
			GsVec dpc = c->c[j+1]-c->c[j-1];
			dpc.y=0;
			dpc.normalize();
			float angle = gs_todeg(acos(dot(dpc,GsVec(0,0,1))));
			bool adjust = 0;
			
			float xdot = dot(dpc,GsVec(1,0,0));
			if (xdot < 0)
					angle = 360 - angle;

			if(abs(angle-prevAngle)>turnAdjustAngle)
			{
				adjust = 1;
				GsVec lad = goalLeftOffset;
				lad.len(turnAdjustDist); // * abs(angle-prevAngle)/turnAdjustAngle);
				goalLeftOffset += lad;

				GsVec rad = goalRightOffset;
				rad.len(turnAdjustDist); // * abs(angle-prevAngle)/turnAdjustAngle);
				goalRightOffset += rad;

				goalLeftOffset.y = footOffsetDist;
				goalRightOffset.y = footOffsetDist;
			}
prevAngle = angle;
			
			GsVec dst = GsVec(0,0,stepDis/4);
			GsVec lof = goalLeftOffset+dst;
			GsVec rof = goalRightOffset-dst;
			GsVec lofr = lof;
			GsVec rofr = rof;
			lofr.roty(gs_torad(angle));
			rofr.roty(gs_torad(angle));
			GsVec lf = c->c[j] + lofr;
			GsVec rf = c->c[j] + rofr;
			
		//	gsout<<leftSteps.size()<<" left foot: "<<lf<<gsnl;
			addStep(LEFT_IK, lf ,angle);
			addStep(RIGHT_IK,rf ,angle);
			addStep(ROOT_IK,GsVec(rf.x, hipsH, rf.z),angle );
			addStep(ROOT_IK,GsVec(lf.x, hipsH,lf.z),angle);
			
			if(adjust && leftSteps.size() > 2)
			{
				leftSteps[leftSteps.size()-1]->adjusted = true;
				rightSteps[rightSteps.size()-1]->adjusted = true;

				if(!leftSteps[leftSteps.size()-2]->adjusted)
				{
					leftSteps[leftSteps.size()-2]->adjusted = true;
					lof.roty(gs_torad(leftSteps[leftSteps.size()-2]->angle));
					leftSteps[leftSteps.size()-2]->Translate(c->c[lastCvPt]+lof);
				}
				if(!rightSteps[rightSteps.size()-2]->adjusted)
				{
					rightSteps[rightSteps.size()-2]->adjusted = true;
					rof.roty(gs_torad(rightSteps[rightSteps.size()-2]->angle));
					rightSteps[rightSteps.size()-2]->Translate(c->c[lastCvPt]+rof);
				}
			}
			lastCvPt = j;
		}

	}
			if(App->mainwin->ui_drawTraj->value())
			{
			App->walk->drawTraj = 1;
			App->walk->playWalk();
			App->walk->drawTraj = 0;
			}
}


App->mainwin->ui_numSteps->value(leftSteps.size()-1);


}
}
void WalkGenerator::makeGoal(GsVec pos, float angle)
{
FootGoal* goal = new FootGoal(pos,angle);

}
void WalkGenerator::addStep(int manip, GsVec pos, float angle)
{
	Foot* foot = new Foot(pos);
	foot->c = pos;
switch(manip)
{
case LEFT_IK:
	{
	foot->model->model(leftfootMod);
	leftSteps.push_back(foot);
	}
	break;
case ROOT_IK:

	foot->model->model(rootMod);
	
		if(!App->mainwin->ui_showRoot->value())foot->model->visible(false);
		rootSteps.push_back(foot);
	break;
case RIGHT_IK:
	{
	foot->model->model(rightfootMod);
	rightSteps.push_back(foot);
	}
break;
}


foot->Rotate(angle); 
foot->Translate(pos); //->get().setrans(pos);
stepGroup->add(foot->g);
}

int WalkGenerator::getFixI(int time)
{	//find the closest root position to fix
	int minDist = 100;
	int idx = 0;
	for(unsigned i=0;i<adjust.size();i++)
	{
		if(abs(adjust[i]->t - time)<minDist) idx = i;
	}
	return idx;
}
void WalkGenerator::adjustHip(int dir)
{
	float amt = (float)App->mainwin->ui_hipAmount->value();
	App->sk->joint("Hips")->update_gmat();
	GsVec rootTemp = App->sk->joint("Hips")->gcenter();
	float angle = App->sk->joint("Hips")->euler()->value(1);

	int sel = App->mainwin->ui_adjustState->focus_index();
	//printf("selected %d\n",label);

	if(sel == 0) //left
	{
		switch(dir)
		{
		case 0://fwd
			leftRootAdjust.z+=amt;
			rootTemp.z+=amt;
			break;
		case 1://rev
			leftRootAdjust.z-=amt;
			rootTemp.z-=amt;
			break;
		case 2://left
			leftRootAdjust.x+=amt;
			rootTemp.x+=amt;
			break;
		case 3://right
			leftRootAdjust.x-=amt;
			rootTemp.x-=amt;
			break;
		case 4://up
			leftRootAdjust.y+=amt;
			rootTemp.y+=amt;
			break;
		case 5://down
			leftRootAdjust.y-=amt;
			rootTemp.y-=amt;
			break;
		}
		gsout<<"leftRoot adjust: "<<leftRootAdjust<<"\n";
	}
	else if(sel == 1)
	{
		switch(dir)
		{
		case 0://fwd
			rightRootAdjust.z+=amt;
			rootTemp.z+=amt;
			break;
		case 1://rev
			rightRootAdjust.z-=amt;
			rootTemp.z-=amt;
			break;
		case 2://left
			rightRootAdjust.x+=amt;
			rootTemp.x+=amt;
			break;
		case 3://right
			rightRootAdjust.x-=amt;
			rootTemp.x-=amt;
			break;
		case 4://up
			rightRootAdjust.y+=amt;
			rootTemp.y+=amt;
			break;
		case 5://down
			rightRootAdjust.y-=amt;
			rootTemp.y-=amt;
			break;
		}
		//GsOutput st;
		//st<<"rightRoot adjust: "<<rightRootAdjust<<"\n";
		App->mainwin->message("rightRoot adjust: ");
	}
		else if(sel == 2) //rest
	{
		switch(dir)
		{
		case 0://fwd
			restRootAdjust.z+=amt;
			rootTemp.z+=amt;
			break;
		case 1://rev
			restRootAdjust.z-=amt;
			rootTemp.z-=amt;
			break;
		case 2://left
			restRootAdjust.x+=amt;
			rootTemp.x+=amt;
			break;
		case 3://right
			restRootAdjust.x-=amt;
			rootTemp.x-=amt;
			break;
		case 4://up
			restRootAdjust.y+=amt;
			rootTemp.y+=amt;
			break;
		case 5://down
			restRootAdjust.y-=amt;
			rootTemp.y-=amt;
			break;
		}
		//GsOutput st;
		//st<<"rightRoot adjust: "<<rightRootAdjust<<"\n";
		App->mainwin->message("rest root adjust: ");
	}
		else if(sel == 3) //all adjust
	{
		switch(dir)
		{
		case 0://fwd
			allRootAdjust.z+=amt;
			rootTemp.z+=amt;
			break;
		case 1://rev
			allRootAdjust.z-=amt;
			rootTemp.z-=amt;
			break;
		case 2://left
			allRootAdjust.x+=amt;
			rootTemp.x+=amt;
			break;
		case 3://right
			allRootAdjust.x-=amt;
			rootTemp.x-=amt;
			break;
		case 4://up
			allRootAdjust.y+=amt;
			rootTemp.y+=amt;
			break;
		case 5://down
			allRootAdjust.y-=amt;
			rootTemp.y-=amt;
			break;
		}
		//GsOutput st;
		//st<<"rightRoot adjust: "<<rightRootAdjust<<"\n";
		App->mainwin->message("all root adjust: ");
	}
	setIK(angle,rootTemp,ROOT_IK);
	App->mainwin->ui_viewer->_kns->update();
		App->mainwin->ui_viewer->redraw();

	/*	int fixI = getFixI(time); //find closest root pos to fix
	switch(dir)
	{
	case 0://fwd
		adjust[fixI]->amt.z+=amt;
		break;
	case 1://rev
		adjust[fixI]->amt.z-=amt;
		break;
	case 2://left
		adjust[fixI]->amt.x+=amt;
		break;
	case 3://right
		adjust[fixI]->amt.x-=amt;
		break;
	case 4://up
		adjust[fixI]->amt.y+=amt;
		break;
	case 5://down
		adjust[fixI]->amt.y-=amt;
		break;
	}*/
}
WalkGenerator::WalkGenerator(SnGroup* root)
{
	stepGroup = root;
	state = STOP;
	run = true;
	time=0;
	t=0;
	resetHip();
	init();

}
void WalkGenerator::reset()
{
	App->mainwin->ui_moveRoot->value(1);
setIK(0,rootStart,ROOT_IK);
	App->mainwin->ui_moveRoot->value(0);
setIK(0,rightFootStart,RIGHT_IK);
setIK(0,leftFootStart,LEFT_IK);

}
void WalkGenerator::resetHip()
{
	leftRootAdjust.set(0,0,0);
	rightRootAdjust.set(0,0,0);
	restRootAdjust.set(0,0,0);
	allRootAdjust.set(0,0,0);
}


class TimeSlider;
void WalkGenerator::playWalk(bool setKeys)
{
	App->mainwin->message("walk playing");
	char str[30];
	inc		   =  App->mainwin->ui_playIncr->value();
	fps        = (float)App->mainwin->ui_StepFPS->value();
	stepHeight = (float)App->mainwin->ui_stepHeight->value();
	steps      = (int)App->mainwin->ui_numSteps->value();
	frames     = (int)App->mainwin->ui_numStepInterp->value();
	TimeSlider* time_slider = App->timeslider;
	fltk::Browser* ui_keylist = App->mainwin->ui_keylist;
	length = frames*steps*3; 
	dh = stepHeight/(2*frames);
	period = (double)(1.0/fps);
	
	float h = 0;
bool doneDraw = 0;
	SnLines* rootFloorLines = new SnLines();
	SnLines* leftFootLines = new SnLines();
	SnLines* rightFootLines = new SnLines();
	SnLines* rootLines = new SnLines();
	SnLines* leftHandLines = new SnLines();
	SnLines* rightHandLines = new SnLines();


	App->viewer->_traj_lines->remove_all();
	
	rootFloorLines->color(GsColor(255,0,0));
	leftFootLines->color(GsColor(255,0,0));
	rightFootLines->color(GsColor(255,0,0));
	rootLines->color(GsColor(255,0,05));
	leftHandLines->color(GsColor(255,0,0));
	rightHandLines->color(GsColor(255,0,0));
	
	rootFloorLines->begin_polyline();
	leftFootLines->begin_polyline();
	rightFootLines->begin_polyline();
	rootLines->begin_polyline();
	leftHandLines->begin_polyline();
	rightHandLines->begin_polyline();




	//incremental mode stops after each step
	//then returns to the previous state set
		stateInit = false;

	if(inc){ 
		if(time==0)
		{
			
			reset();
			state = START;
		}
		else
			state = returnState;
	}
	else if(resume)
	{
		h = lastH;
		resume = 0;
		state = returnState;
		t = lastIt;
		stateInit = true;
	}
	else 
	{
		
		state = START;
	}

	float intp=0; //to hold interpolation value
	float intpCub = 0;
	while(1)
	{
		App->checkHoap();
		App->checkVicon();

		//state=0:Root->LeftFoot 1: RightFoot->move 2: Root->RightFoot 3:LeftFoot->move move Right 2: 
	startTime = get_time();
	intp = (float)t/(float)frames;
	
	GsVec avrg ; //used to calculate 
	float angle,a1,a2;
	switch(state)
	{
	case START : 
	
	step=0;

	//set starting point to skeleton position
	if(!stateInit)
	{	
		time = 0;
		playing = true;
		rootStart = App->sk->joint("Hips")->gcenter();
		a1 = App->sk->joint("Hips")->rot()->euler()->value(1);
		a2 = rootSteps[0]->angle;
		rootStartH = rootStart.y;

		//get first goal for root
		rootSteps[0]->tfm->get().getrans(nextRoot);
		nextRoot += allRootAdjust;
		stateInit=true;
		t=0;
		
	}

	angle = gs_anglerp(a1,a2,intp);
	intpCub = interp_cubic(intp,0,1);
	root = interp(rootStart, nextRoot, intpCub);
	root.y = rootStartH;
	

	
	//transform the root ik 
	setIK(angle,root,ROOT_IK);

	
	t++;
	if(pause)
	{
		returnState = state;
		lastIt = t;
		return;
	}
	if(t>frames)
	{
		t=0;
		prevState=START;
		state = RIGHT_MOVE;
		returnState = state;


		stateInit=false;
		if(inc)return;
	}

	break;

case STOP : 
	

	if(!stateInit)
	{
		avrg = (nextLeft+nextRight)*.5;
		rootEnd = GsVec(avrg.x,rootStart.y,avrg.z);
		rootEnd += restRootAdjust + allRootAdjust;
		rootSteps[steps*2-1]->tfm->get().getrans(prevRoot);
		stateInit=true;
		a1 = a2;
		a2 = rootSteps[rootSteps.size()-1]->angle;
		t=0;
	}
	angle = gs_anglerp(a1,a2,intp);
	intpCub = interp_cubic(intp,0,1);
	root = interp( prevRoot, rootEnd, intpCub ) ;
	root.y = rootStartH;
	setIK(angle,root,ROOT_IK);

		t++;
	if(pause)
	{
		App->mainwin->message("paused");
		returnState = state;
		lastIt = t;
		return;
	}

	if(t>frames)
	{
		if(drawTraj)
		{

	rootFloorLines->end_polyline();
	leftFootLines->end_polyline();
	rightFootLines->end_polyline();
	rootLines->end_polyline();
	leftHandLines->end_polyline();
	rightHandLines->end_polyline();

	App->viewer->_traj_lines->add(rootFloorLines);
		App->viewer->_traj_lines->add(rootLines);
	App->viewer->_traj_lines->add(leftFootLines);
	App->viewer->_traj_lines->add(rightFootLines);
	App->viewer->_traj_lines->add(leftHandLines);
	App->viewer->_traj_lines->add(rightHandLines);
reset();
		}
		playing = false;
		t=0;
		App->mainwin->message("done with playback\n");
		prevState=STOP;
		returnState = STOP;
		stateInit=false;
		return;
	}

	break;

case ROOT_TO_LEFT: 

	if(!stateInit){
		rootSteps[2*step]->tfm->get().getrans(nextRoot);
		a2 = rootSteps[2*step]->angle;
		rootSteps[2*step-1]->tfm->get().getrans(prevRoot);
		a1 = rootSteps[2*step-1]->angle;
		stateInit=true;
		t=0;
	}
	intpCub = interp_cubic(intp,0,1);
	root = interp(prevRoot,nextRoot,intpCub );
	root += interp(rightRootAdjust,leftRootAdjust,intpCub);
	root += allRootAdjust;
	angle = interp(a1,a2,intp);

	root.y = rootStartH;
	setIK(angle,root,ROOT_IK);

		t++;
	if(pause)
	{
		returnState = state;
		lastIt = t;
		return;
	}

	if(t>frames){
		t=0;
		prevState = ROOT_TO_LEFT;
		state = RIGHT_MOVE;
		returnState = state;
		stateInit=false;
		if(inc)return;
	}
	break;
case RIGHT_MOVE :
		if(!stateInit){
		rightSteps[step+1]->tfm->get().getrans(nextRight);
		a2 = rightSteps[step+1]->angle;
		rightSteps[step]->tfm->get().getrans(prevRight);
		a1 = rightSteps[step]->angle;
		stateInit=true;
		t=0;
		h=0;
	}
	h =  stepHeight * sBump (intp);
	//t < frames/2 ? h+=dh : h-=dh;
	//if(t==frames)h=0;
	right = interp(prevRight,nextRight,intp) + GsVec(0,h,0);

	angle = gs_todeg(gs_anglerp(gs_torad(a1),gs_torad(a2),intp));
	setIK(angle,right,RIGHT_IK);

		t++;
	if(pause)
	{
		lastH = h;
		returnState = state;
		lastIt = t;
		return;
	}

	if(t>frames){
		t=0;
		prevState = RIGHT_MOVE;
		state = ROOT_TO_RIGHT;
		returnState = state;
		stateInit=false;
		if(inc)return;
	}
	break;
case  ROOT_TO_RIGHT :

	if(!stateInit){
		rootSteps[2*step+1]->tfm->get().getrans(nextRoot);
		a2 = rootSteps[2*step+1]->angle;
		rootSteps[2*step]->tfm->get().getrans(prevRoot);
		a1 = rootSteps[2*step]->angle;
		stateInit=true;
		t=0;
	}
	intpCub = interp_cubic(intp,0,1);
	root = interp(prevRoot,nextRoot,intpCub);
	root += interp(leftRootAdjust,rightRootAdjust,intpCub);
	root += allRootAdjust;
	root.y = rootStartH;
	angle = gs_todeg(gs_anglerp(gs_torad(a1),gs_torad(a2),intp));
	setIK(angle,root,ROOT_IK);

		t++;
	if(pause)
	{
		returnState = state;
		lastIt = t;
		return;
	}
	if(t>frames){
		t=0;
		prevState = ROOT_TO_RIGHT;
		state = LEFT_MOVE;
		returnState = state;
		stateInit=false;
		if(inc)return;
	}
	break;
case LEFT_MOVE : 
	if(!stateInit)
	{
		leftSteps[step+1]->tfm->get().getrans(nextLeft);
		a2 = leftSteps[step+1]->angle;
		leftSteps[step]->tfm->get().getrans(prevLeft);
		a1 = leftSteps[step]->angle;
		stateInit=true;
		t=0;
	}
	h =  stepHeight * sBump (intp);
//	t < frames/2 ? h+=dh : h-=dh;
//	if(t==frames)h=0;

	left = interp(prevLeft,nextLeft,intp) + GsVec(0,h,0);
	angle = gs_todeg(gs_anglerp(gs_torad(a1),gs_torad(a2),intp));
	setIK(angle,left,LEFT_IK);

		t++;
	if(pause)
	{
		lastH = h;
		returnState = state;
		lastIt = t;
		return;
	}
	if(t>frames)
	{
		t=0;
		prevState = LEFT_MOVE;
		if(++step>=steps) {
			state = STOP;
			stateInit = false;
		}
		else
		{
		state = ROOT_TO_LEFT;
		stateInit = false;
		}
		returnState = state;
		if(inc)return;
	}
	break;

case  WAIT : 
	
	break;

}




time++;

//send posture to robot
if(App->realtime)
{
	App->mainwin->sendPosture();
}

if(setKeys)
{	//set a key
	if(t==0){
		sprintf(str,"ikKey %g", time); 
		printf("str is %s\n",str);
		ui_keylist->add((const char*)str);
		App->timeslider->currentTime = time;
		App->timeslider->ikKey();
		App->nla->getTrack()->empty = false;
		App->graph->showGroup();
	}
}
else if(drawTraj)
{
	GsVec rootD = App->sk->joint("Hips")->pos()->value();
	rootLines->push(rootD);
	rootD.y = 0;
	rootFloorLines->push(rootD);
	leftFootLines->push(App->sk->joint("LeftFootBase")->gcenter());
	rightFootLines->push(App->sk->joint("RightFootBase")->gcenter());
	leftHandLines->push(App->sk->joint("LeftHand")->gcenter());
	rightHandLines->push(App->sk->joint("RightHand")->gcenter());

}
else
{

		fltk::check();
		endTime=get_time();
		dt = endTime-startTime;
		
		if(dt<period)
			wait(period - dt);
		//else
			//printf("ran out of time  ");

	//	printf("endTime %g startTime %g dt is %g wait %g\n", endTime, startTime, dt, period - dt);
		App->timeslider->draw();
		App->mainwin->ui_viewer->_kns->update();
			
		App->mainwin->ui_viewer->redraw();
		
	//	App->nla->updateCurves();

}

		
	} 
}


void setKey(float deg,int t,int Manip,GsVec p)
{
		setIK(deg,p,Manip);
		App->timeslider->currentTime = t;
		App->timeslider->ikKey();
}
void WalkGenerator::makeWalk()
{

	float height = (float)App->mainwin->ui_stepHeight->value();
	int num = (int)App->mainwin->ui_numSteps->value();
	float stepD = (float)App->mainwin->ui_stepDistance->value();
	TimeSlider* time_slider = App->timeslider;
	fltk::Browser* ui_keylist = App->mainwin->ui_keylist;
	
	
	GsVec nextLeft;
	GsVec prevLeft;
	GsVec nextRight;
	GsVec prevRight;
	GsVec rpstart,rp;
	GsVec leftp,rightp;
	float rootH;

	App->nla->getTrack()->init();

	//get first and second step for each foot
//	leftSteps[1]->mat().getrans(nextLeft);
//	leftSteps[0]->mat().getrans(prevLeft);
//	rightSteps[1]->mat().getrans(nextRight);
//	rightSteps[0]->mat().getrans(prevRight);
	//start point for root
	rootH = App->mainwin->ui_viewer->_RootIK->translation().y; 

	//first check if it is first loop in cycle and transition from rest

	//weight on left foot
	rp = GsVec(prevLeft.x + adjust[0]->amt.x ,rootH + adjust[0]->amt.y, prevLeft.z + adjust[0]->amt.z) ; //add correction here
	setKey(0,0,ROOT_IK,rp);
	//move right foot
	setKey(0,30,RIGHT_IK,nextRight);
	//weight on right foot
	rp = GsVec(nextRight.x+adjust[1]->amt.x,rootH+adjust[1]->amt.y,nextRight.z+adjust[1]->amt.z) ; //add correction here
	setKey(0,60,ROOT_IK,rp);
	//move left foot
	setKey(0,90,LEFT_IK,nextLeft);

}
