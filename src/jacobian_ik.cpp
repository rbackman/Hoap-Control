#include "jacobian_ik.h"
# include "app_main_win.h"

GsMatn tr2diff(GsMat hand,GsMat goal)
{
	GsVec handP,goalP;
	hand.getrans(handP);
	goal.getrans(goalP);
	GsVec dp = goalP - handP;
	GsMatn diff(1,6);
	diff(0,0)=dp.x;
	diff(0,1)=dp.y;
	diff(0,2)=dp.z;

	GsVec rx = cross((GsVec)&hand.e11,(GsVec)&goal.e11);
	GsVec ry = cross((GsVec)&hand.e21,(GsVec)&goal.e21);
	GsVec rz = cross((GsVec)&hand.e31,(GsVec)&goal.e31);
	GsVec dr = rx+ry+rz;
	dr = dr*0.5f;

	diff(0,3)=dr.x;
	diff(0,4)=dr.y;
	diff(0,5)=dr.z;
	diff.transpose();
return diff;
}

Jacobian::Jacobian()
{
reset();

}



void Jacobian::reset()
{

		 for(int i=0;i<n;i++)
		 {
			 jnts[i].j->init_values();
		 }
		 App->sk->land();
		 
		 App->sk->joint("Hips")->pos()->value(1,35.2f);
		//gsout<<"0 low limit "<< App->sk->joint("Hips")->rot()->euler()->lolim(0)<<gsnl;

		 	App->JIKmanip->translation(App->sk->joint("LeftHand")->gcenter());

			jnts.remove(0,jnts.size());



KnJoint* jtmp =App->sk->joint("Hips") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i) && App->mainwin->ui_JacHips[i]->value())
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}


jtmp =App->sk->joint("LeftShoulder") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i))
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}

	jtmp =App->sk->joint("LeftUpArm") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i))
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}
	jtmp =App->sk->joint("LeftUpArmRoll") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i))
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}
	jtmp =App->sk->joint("LeftForeArm") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i))
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}
		jtmp =App->sk->joint("LeftHand") ;
	for(int i=0;i<3;i++)
	{
		if(!jtmp->euler()->frozen(i))
		{
			Jdof jd;
			jd.j = jtmp;
			jd.dof = i;
			jnts.push(jd);
		}
	}


		m = 6;
		n = jnts.size(); //one extra for hips translation
		printf("num dof: %d\n",n);

		J.resize(m,n);
		Jcol.resize(m,n);
		theta.resize(1,n);
		eff.resize(m,1);
		JJt_inv.resize(n,n);

		 App->viewer->redraw();
}
void Jacobian::evaluate()
{
	bool debug = App->mainwin->ui_debug->value();
	falloff = (float)App->mainwin->ui_falloff->value();
	int method = App->mainwin->ui_solveMethod->focus_index();
	avoidCol = App->mainwin->ui_col->value();
    closestJ = 0;
	closestD = 1000;
	maxV = (float)App->mainwin->ui_maxV->value();
	//gsout<<method<<gsnl;

	s = App->JIKmanip->translation(); 
	if(s.len()>LENGTH)
	{
	s.len(LENGTH);
	}

	endJ = App->sk->joint("LeftHand")->gcenter(); 

	 dEnd = s - endJ;
	 if(App->mainwin->ui_stopSolve->value())
	 {
		 if(dEnd.len()<App->mainwin->ui_tol->value())
		 {
		//gsout<<"got to target"<<gsnl;
		return;
		 }
	 }
	 dEnd.len((float)App->mainwin->ui_dEnd->value());
	
		 for(int i=0;i<n;i++)
		 {
			GsVec p = jnts[i].j->gcenter();

			GsVec axs;
			if(jnts[i].dof==0) axs = &jnts[i].j->gmat().e11;
			else if(jnts[i].dof==1) axs = &jnts[i].j->gmat().e21;
			else if(jnts[i].dof==2) axs = &jnts[i].j->gmat().e31;
			//p.z = 0;
			GsVec crsX = cross(axs,s-p);
			

			 J.set(0,i,crsX.x);
			 J.set(1,i,crsX.y);
			 J.set(2,i,crsX.z);
			 J.set(3,i,axs.x);
			 J.set(4,i,axs.y);
			 J.set(5,i,axs.z);
			 
	

		 }
	 
		 if(avoidCol)
		{
			/*	 for(int i=0;i<n;i+=3)
			 {
				GsVec p = jnts[i/3]->gcenter();
				GsVec c = App->JIKobstacle->translation();
				GsVec ds = p-c;
				if(ds.len() < closestD)
				{
					closestD = ds.len();
					closestJ = (int)(i/3);
				}

				//p.z = 0;
				GsVec crsX = cross(GsVec(1,0,0),ds);
				GsVec crsY = cross(GsVec(0,1,0),ds);
				GsVec crsZ = cross(GsVec(0,0,1),ds);

*/
			/*	float len = ds.len();
				float wt;
				gsout<<"len "<<len;
				if(len>=falloff) wt=0;
				else if(len<=0.001) wt=1;
				else
				{
					wt = 	(falloff - len)/ falloff;
					gsout<<"  wt "<<wt<<gsnl;
				}

				crsX*=wt;
				crsY*=wt;
				crsZ*=wt;*/
/*
				 Jcol.set(0,i,crsX.x);
				 Jcol.set(1,i,crsX.y);
				// Jcol.set(2,i,crsX.z);
				 
				 Jcol.set(0,i+1,crsY.x);
				 Jcol.set(1,i+1,crsY.y);
				// Jcol.set(2,i+1,crsY.z);
				 
				 Jcol.set(0,i+2,crsZ.x);
				 Jcol.set(1,i+2,crsZ.y);
				// Jcol.set(2,i+2,crsZ.z);

			 }*/
		 }


	 eff = tr2diff(App->sk->joint("LeftHand")->gmat(),App->JIKmanip->mat());
	 eff*= (float)App->mainwin->ui_dEnd->value();
	
	
	 Jt = J;
	 Jt.transpose();

	 switch(method)

	 {
	 case PSEUDO:
		{
		 JJt.mult(J,Jt);

		
		if(!inverse(JJt,JJt_inv)){gsout<<"couldnt invert\n";return;}				 
		j_plus.mult(Jt,JJt_inv);
		theta.mult(j_plus,eff);
		if(debug)
		 {
				gsout<<"Jt= \n"<<Jt;
				gsout<<"J= \n"<<J;
				gsout<<"J rows:"<<J.lin()<<" col: "<<J.col()<<gsnl;
				gsout<<"JJt rows:"<<JJt.lin()<<" col: "<<JJt.col()<<gsnl;
				gsout<<"JJt:\n \n"<<JJt<<gsnl<<gsnl;;
				gsout<<"JJt_inv: \n"<<JJt_inv<<gsnl<<gsnl;		
		 }
		 }	break;	
	 case TRANSPOSE:
		 {
			 avoidCol=false;
		 theta.mult(Jt,eff);

		 }break;
	 case DAMPED:
		 {
			 float dampC = (float)App->mainwin->ui_dampC->value();
			 dampC = dampC*dampC;

			 JJt.mult(J,Jt);
			 Idt.resize(n,n);
			 Idt.identity();
			 Idt*=dampC;
			
			 JJTPG.add(JJt,Idt);

		 
		if(!inverse(JJTPG,JJt_inv)){gsout<<"couldnt invert\n";return;}				 
		j_plus.mult(Jt,JJt_inv);
		//gsout<<"\n\nJPseudoI:\n"<<j_plus<<gsnl<<gsnl;
		theta.mult(j_plus,eff);
		 }break;


	 }
	 
	 if(avoidCol)
	 {
		 /*
		 GsVec obst = App->JIKobstacle->translation();
			 GsVec d1 ,d2;
		if(closestJ < jnts.size()-1) 
		{
			d1 = GetClosetPoint(jnts[closestJ]->gcenter(),jnts[closestJ+1]->gcenter(),obst,true);
				
		}
		if(closestJ >= 1) 
		{
			d2 = GetClosetPoint(jnts[closestJ-1]->gcenter(),jnts[closestJ]->gcenter(),obst,true);
			
		}
		GsVec ds1 = d1-obst;
		GsVec ds2 = d2 - obst;
			if(ds1.len()<ds2.len())
			{
				closestPoint = d1;
				gsout<<"d1"<<gsnl;
			}
		else
		{
			closestPoint = d2;
			gsout<<"d2"<<gsnl;
		}

		GsVec xo = closestPoint - obst ;
		
		xo.len(App->mainwin->ui_obsRep->value());
		wts.resize(n,1);
				
						 for(int i=0;i<n;i++)
			 {
				GsVec p = jnts[i]->gcenter();
				GsVec c = App->JIKobstacle->translation();
				GsVec ds = p-c;
				float len = ds.len();
				float wt;
			//	gsout<<"len "<<len;
				if(len>=falloff) wt=0;
				else if(len<=0.001) wt=1;
				else
				{
					wt = 	(falloff - len)/ falloff;
				//	gsout<<"  wt "<<wt<<gsnl;
					
				}
				wts.set(i,0,wt);
				wts.set(i+1,0,wt);
				wts.set(i+2,0,wt);
			//	 gsout<<"wts:\n"<<wts<<gsnl;
			}
				//		 gsout<<"wts:\n"<<wts<<gsnl;


		GsMatn xO(3,1);

		xO.set(0,0,xo.x);
		xO.set(1,0,xo.y);
	//	xO.set(2,0,xo.z);

		Idt.resize(n,n);
		Idt.identity();

		JpiJ.mult(j_plus,J);         //Je+Je
		ImJpiJ.sub(Idt,JpiJ);        //I-Je+Je
		Ucol.mult(Jcol,ImJpiJ);     //J0(I-Je+Je)
		UcolT = Ucol;
		UcolT.transpose();

		//gsout<<"jPlus:\n\n"<<j_plus<<gsnl<<gsnl;
		PIO.mult(Ucol,UcolT);
		

	//	gsout<<"PIO:\n\n"<<PIO<<gsnl<<gsnl;
		if(!inverse(PIO,PIO_inv)){gsout<<"couldnt invert\n";return;}		
	//	gsout<<"PIO invers:\n\n"<<PIO_inv<<gsnl<<gsnl;
		col_pseudoI.mult(UcolT,PIO_inv);
//gsout<<"colPseudo\n\n"<<col_pseudoI<<gsnl<<gsnl;
		JoJpi.mult(Jcol,j_plus);

		 JoJpiXe.mult(JoJpi,eff);
		  Xo_JoJpiXe.sub(xO,JoJpiXe);
GsMatn newC;
newC.mult(col_pseudoI,Xo_JoJpiXe);
//gsout<<"new theta: \n"<<newC<<gsnl<<gsnl;
		//Ucol
		//Ucol*=App->mainwin->ui_obsRep->value();
				if(debug){
					gsout<<ImJpiJ<<gsnl<<gsnl<<Idt<<gsnl;
					gsout<<"UCOL"<<Ucol<<gsnl;

					gsout<<Xo_JoJpiXe<<gsnl<<gsnl;
					gsout<<col_pseudoI<<gsnl<<gsnl;
					gsout<<"theta before:\n\n"<<theta<<gsnl<<gsnl;
				}


newC.mult(wts,newC);



				
		theta.add(theta,newC);
				for(int i=0;i<n;i++)
				{
				if(theta.get(i,0)>maxV)theta.set(i,0,maxV);
				else if(theta.get(i,0)<-maxV)theta.set(i,0,-maxV);
				}

*/
	 }


		 if(debug)
		 {
			 gsout<<"theta= \n";
			 gsout<<theta<<gsnl<<gsnl<<gsnl<<gsnl;
		 }


		 GsVec og = App->sk->joint("Hips")->gcenter();
		 GsVec dg = App->JIKmanip->translation() - App->sk->joint("LeftHand")->gcenter();
		 dg.len((float)App->mainwin->ui_JtransVel->value());
		 GsVec newR = og+dg;
		 if(newR.x>5)newR.x=5;
		 if(newR.x<-5)newR.x=-5;
		 if(newR.y>App->maxHipHeight)newR.y=App->maxHipHeight;
		 if(newR.y<30)newR.y=30;
		 if(newR.z>5)newR.z=5;
		 if(newR.z<-5)newR.z=-5;

		 if(App->mainwin->ui_hipsJacobian->value())App->sk->joint("Hips")->pos()->value(newR);
		 //App->sk->joint("Hips")->pos()->value(og.x+theta.get(0,0),og.y+theta.get(0,1),og.z+theta.get(0,2));

		for(int i=0;i<n;i++)
		{
			
			float prevAng = jnts[i].j->euler()->value(jnts[i].dof);
			float newAng;
			/*if(App->sk->joint("Hips")==jnts[i].j)
				newAng= prevAng + 0.01f*theta.get(0,i);
			else*/
				newAng= prevAng + (float)theta.get(0,i);

				if(jnts[i].j->euler()->inlimits(jnts[i].dof,newAng))
					jnts[i].j->euler()->value(jnts[i].dof,newAng);

			/*if( i<=2) 
				
				gsout<<"low limit x"<<jnts[i].j->euler()->lolim(0)<<
				" new Angle "<<newAng<<gsnl;*/
		}
		

	//	App->sk->update_global_matrices();
	//	App->mainwin->ui_viewer->_knsc->update();
		App->mainwin->ui_viewer->redraw();
	
	 
}
GsVec GetClosetPoint(GsVec A, GsVec B, GsVec P, bool segmentClamp)
{
    GsVec AP = P - A;
    GsVec AB = B - A;
    float ab2 = AB.x*AB.x + AB.y*AB.y;
    float ap_ab = AP.x*AB.x + AP.y*AB.y;
    float t = ap_ab / ab2;
    if (segmentClamp)
    {
         if (t < 0.0f) t = 0.0f;
         else if (t > 1.0f) t = 1.0f;
    }
    GsVec Closest = A + AB * t;
    return Closest;
}