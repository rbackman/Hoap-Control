#include "particle_ik.h"

///Particle IK System
ParticleIK::ParticleIK()
{
	sikManip = new SnManipulator;
	numParticles = 20;
	   stiffness = 0.2f;
	   handStiffness = 0.2f;
	   damping = 0.1f;
	   grav = 0.0f;
	   stepSize = 0.1f;

	spheres = new SnGroup;
	skelTargeted = false;
	spheres->separator(true);
	lines = new SnGroup;
	lines->separator(true);
	init();
	
}
void ParticleIK::reset()
{
	for(int i=0;i<particles.size();i++)
	{
		particles.get(i)->reset();
	}
}
void ParticleIK::addParticle(Particle* particle)
{
	GsMaterial newMat;
	newMat.diffuse.set(255,0,0);
	particle->pik = this;
	GsModel* m = new GsModel;
	m->make_sphere(GsPnt(0,0,0),1,10,true);
	m->set_one_material(newMat);
	SnModel* sm = new SnModel(m);
	SnGroup* partGroup = new SnGroup;
	partGroup->separator(true);
	partGroup->add(particle->spherePos);
	partGroup->add(sm);
	spheres->add(partGroup);
	particles.push(particle);
	numParticles++;
}

void ParticleIK::makeSpring(Particle* p1, Particle* p2)
	{
		Spring* s = new Spring(p1,p2);
		p1->springs.push(s);
		p2->springs.push(s);
		lines->add(s->line);
		springs.push(s);
	}

void ParticleIK::targetSkel(KnJoint* rootJ, KnJoint* effJ)
	{	
		init();
		skelTargeted = true;
		skel = rootJ->skeleton();
		effector = effJ;
		root = rootJ;
		KnJoint* nextParent;
		nextParent = effJ;

		do
		{
			gsout<<nextParent->name()<<gsnl;
			Particle* p = new Particle(nextParent->gcenter());
			p->pik = this;
			p->id = nextParent->index();
			GsModel* m = new GsModel;
			m->make_sphere(GsPnt(0,0,0),3,10,true);
			GsMaterial newMat;
			newMat.diffuse.set(255,0,0);
			m->set_one_material(newMat);

			SnModel* sm = new SnModel(m);
			SnGroup* partGroup = new SnGroup;
			partGroup->separator(true);
			partGroup->add(p->spherePos);
			partGroup->add(sm);
			spheres->add(partGroup);
			particles.push(p);
			numParticles++;
			nextParent = nextParent->parent();
		}while( nextParent->index()!= rootJ->index() );


	for(int i=0;i<numParticles-1;i++)
	{
		Particle* p1 = particles.get(i);
		Particle* p2 = particles.get(i+1);
		Spring* s = new Spring(p1,p2);
		p1->springs.push(s);
		p2->springs.push(s);
		lines->add(s->line);
		springs.push(s);
	}
	for(int i=0;i<springs.size();i++)
	{
		springs.get(i)->update();
	}


	}
void ParticleIK::randomChain()
	{


	for(int i=0;i<numParticles;i++)
	{
		Particle* p = new Particle(GsVec(gs_random(-15.0f,15.0f),gs_random(-15.0f,15.0f),gs_random(-15.0f,15.0f)));
		p->pik = this;
		p->id = i;
		GsModel* m = new GsModel;
		m->make_sphere(GsPnt(0,0,0),3,10,true);
		GsMaterial newMat;
			newMat.diffuse.set(255,0,0);
			m->set_one_material(newMat);
		SnModel* sm = new SnModel(m);
		SnGroup* partGroup = new SnGroup;
		partGroup->separator(true);
		partGroup->add(p->spherePos);
		partGroup->add(sm);
		spheres->add(partGroup);
		particles.push(p);
	}
	for(int i=0;i<particles.size()-1;i++)
	{
		Particle* p1 = particles.get(i);
		Particle* p2 = particles.get(i+1);
		Spring* s = new Spring(p1,p2);
		p1->springs.push(s);
		p2->springs.push(s);
		lines->add(s->line);
		springs.push(s);
	}
	for(int i=0;i<springs.size();i++)
	{
		springs.get(i)->update();
	}
	}

void ParticleIK::init()
{
	spheres->remove_all();
	lines->remove_all();
	particles.size(0);
	springs.size(0);
	numParticles = 0;
}

void ParticleIK::applyToSkel()
{
	for(int i=particles.size()-2;i>=0;i--)
	{
		Particle* nextP = particles.get(i);
		Particle* thisP = particles.get(i+1);
		KnJoint* joint = skel->joints().get(thisP->id);
		GsVec direction = nextP->pos - thisP->pos;
		
	}
}

void ParticleIK::snapToSkel()
{

}

void ParticleIK::evaluate()
{
	
	for(int i=0;i<particles.size();i++)
	{
		particles.get(i)->update();
	}
	particles.get(0)->setPos(sikManip->translation());
	particles.get(numParticles-1)->reset();
	for(int i=0;i<springs.size();i++)
	{
		springs.get(i)->update();
	}

}

void Particle::update()
{
	if(dynamic)
	{
		GsVec force;
		for(int i=0;i<springs.size();i++)
		{
			Spring* s = springs.get(i);
			force += s->getForce(this);
		}
		force -= vel*pik->damping;
		force += GsVec(0,-pik->grav,0);
		accel = force/mass;
		vel += accel*pik->stepSize;
		pos += vel*pik->stepSize;
		setPos(pos);
	}
}

//hoap specific functions
void ParticleIK::evaluateHoapArm()
{
	for(int i=0;i<particles.size();i++)
	{
		particles.get(i)->update();
	}
	hipsP->reset();  //shoulderP->reset();
	setEffector(sikManip->translation(),sikManip->rotation());

	//handP->setPos(sikManip->translation());
	for(int i=0;i<springs.size();i++)
	{
		springs.get(i)->update();
	}

}
void ParticleIK::applyToHoapArm()
{
	GsVec upArm = elbowP->pos-shoulderP->pos;
	GsVec foreArm = handP->pos-elbowP->pos;
	GsVec hips = shoulderP->pos-hipsP->pos;
	GsQuat upArmQ;
	GsQuat hipsQ;
	hipsQ.set(shoulderAxis,hips);
	upArmQ.set(GsVec(0,0,1),upArm);
	skel->joint("Hips")->euler()->set(hipsQ);

	skel->joint("LeftShoulder")->euler()->set(upArmQ);
	skel->joint("LeftUpArm")->euler()->set(upArmQ);
	skel->joint("LeftUpArmRoll")->euler()->set(upArmQ);
	GsQuat elbowQ;
	elbowQ.set(upArm,foreArm);
	skel->joint("LeftForeArm")->euler()->set(elbowQ);

updateIK();

/*	GsVec handPos = (handPFL->pos + handPFR->pos + handPF->pos) / 3.0f;
	GsVec effPos = 	(effPF->pos + effPFR->pos + effPFL->pos) / 3.0f ;
	GsVec handRotVec = handPF->pos - handPos;
	GsVec effRotVec = effPF->pos - effPos;
	float handAngle = angle(handRotVec,effRotVec);

	skel->joint("LeftHand")->euler()->value(0,handAngle);
	skel->joint("LeftHand")->euler()->value(1,handAngle);
	skel->joint("LeftHand")->euler()->value(2,handAngle);
*/
}
void ParticleIK::setEffector(GsVec pos, GsQuat rot)
{
	effPF->setPos(pos + rot.apply(effPF->startPos));
	effPFR->setPos(pos + rot.apply(effPFR->startPos));
	effPFL->setPos(pos + rot.apply(effPFL->startPos));
	effPFD->setPos(pos + rot.apply(effPFD->startPos));
	effP->setPos(pos + rot.apply(effP->startPos));
}
void ParticleIK::hoapArm(KnSkeleton* sk)
{
	init();

//for hoap arm keep track of specific joints 
	skelTargeted = true;
	skel = sk;
	skel->init_values();
	skel->update_global_matrices();
	shoulder = skel->joint("LeftUpArm");
	elbow = skel->joint("LeftForeArm");
	hand = skel->joint("LeftHand");
	hips = skel->joint("Hips");
	effector = hand;
	root = shoulder;
	shoulderAxis = shoulder->gcenter() - hips->gcenter();
//spacing of hand effector particles
	float ht = 8;
	float wd = 12;

//dynamic joint particles
	handP   = new Particle(hand->gcenter(), hand->index());
	handPFL = new Particle(hand->gcenter() + GsVec(ht,0,wd), hand->index());
	handPFR = new Particle(hand->gcenter() + GsVec(-ht,0,wd), hand->index());
	handPF  = new Particle(hand->gcenter() + GsVec(0,ht,wd), hand->index());
	handPFD = new Particle(hand->gcenter() + GsVec(0,-ht,wd), hand->index());
	hipsP   = new Particle(hips->gcenter(),hips->index());
	elbowP = new Particle(elbow->gcenter(),elbow->index());
	shoulderP = new Particle(shoulder->gcenter(),shoulder->index());

	addParticle(shoulderP);
	addParticle(handPFD);
	addParticle(handPF);
	addParticle(handPFL);
	addParticle(handPFR);
	addParticle(handP);
	addParticle(hipsP);
	addParticle(elbowP);

//non dynamic effector particles
	effPF = new Particle(GsVec(0,ht,wd), hand->index());
	effPFD = new Particle(GsVec(0,-ht,wd), hand->index());
	effPFR = new Particle(GsVec(-ht,0,wd), hand->index());
	effPFL = new Particle(GsVec(ht,0,wd), hand->index());
	effP = new Particle(GsVec(0,0,0), hand->index());

	effPFD->dynamic = false;
	effPF->dynamic = false;
	effPFR->dynamic = false;
	effPFL->dynamic = false;
	effP->dynamic = false;
	addParticle(effPFD);
	addParticle(effPF);
	addParticle(effPFR);
	addParticle(effPFL);
	addParticle(effP);

//create springs for effector connections with zero restlength
	makeSpring(hipsP,shoulderP);
	makeSpring(effPFD,handPFD); springs.get(springs.size()-1)->restLength=0; 
	makeSpring(effPFL,handPFL); springs.get(springs.size()-1)->restLength=0; 
	makeSpring(effPFR,handPFR); springs.get(springs.size()-1)->restLength=0; 
	makeSpring(effPF,handPF);	springs.get(springs.size()-1)->restLength=0; 
	makeSpring(effP,handP);		springs.get(springs.size()-1)->restLength=0; 

	makeSpring(elbowP,handPFD);
	makeSpring(elbowP,handPFL); 
	makeSpring(elbowP,handPFR); 
	makeSpring(elbowP,handPF); 
	makeSpring(elbowP,handP);
	makeSpring(elbowP,shoulderP);


	makeSpring(handP,handPFL);		springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handP,handPFR);		springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handP,handPF);		springs.get(springs.size()-1)->handSpring = true;

	makeSpring(handP,handPFD);		springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handPFL,handPFD);	springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handPFR,handPFD);	springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handPF,handPFD);		springs.get(springs.size()-1)->handSpring = true;

	makeSpring(handPF,handPFL);		springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handPF,handPFR);     springs.get(springs.size()-1)->handSpring = true;
	makeSpring(handPFL,handPFR);    springs.get(springs.size()-1)->handSpring = true;

	gsout<<springs.size()<<" springs made"<<gsnl;

	for(int i=0;i<springs.size();i++)
	{
		springs.get(i)->update();
	}


}

///Spring class
Spring::Spring(Particle* p1n, Particle* p2n)
{	
	p1 = p1n;
	p2 = p2n;
	handSpring = false;
	line = new SnLines;
	line->push(p1->pos,p2->pos);
	GsVec springVec =  p2->pos - p1->pos;
	restLength = springVec.len();
	springK = 0.1f;
}
void Spring::update()
{
	springK = p1->pik->stiffness;
	if(handSpring) springK = p1->pik->handStiffness;
	line->init();
	line->push(p1->pos,p2->pos);
}
GsVec Spring::getForce(Particle* p)
{
	center = (p1->pos + p2->pos)/2;
	GsVec dist = p2->pos - p1->pos;
	float stretch = restLength - dist.len();
	if(abs(stretch)<0.001)stretch = 0;
	GsVec dir = p->pos - center;
	dir.normalize();
	return dir*stretch*springK;
}
///Particle class
Particle::Particle(GsVec spos)
{
	mass = 1;
	spherePos = new SnTransform;
	startPos = spos;
	dynamic = true;
	setPos(startPos);
}
Particle::Particle(GsVec spos,int ind)
{
	mass = 1;
	spherePos = new SnTransform;
	startPos = spos;
	dynamic = true;
	setPos(startPos);
	id = ind;
}
void Particle::reset()
{
	setPos(startPos); 
	vel.set(0,0,0);
	accel.set(0,0,0);
}
void Particle::setPos(GsVec p)
{
	spherePos->get().translation(p);
	pos = p;
}
