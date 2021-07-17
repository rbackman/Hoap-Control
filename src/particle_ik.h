#pragma once

#include "app_main.h"

class Spring;

class Particle
{
public:
	ParticleIK* pik;
	int id;
	bool dynamic;
	SnTransform* spherePos;
	float mass;
	GsVec startPos;
	GsVec pos;
	GsVec vel;
	GsVec accel;
	GsArray<Spring*> springs;
	void update();
	Particle(GsVec spos);
	Particle(GsVec spos,int ind);
	void setPos(GsVec p);
	void reset();
};
class Spring
{
public:
	SnLines* line;
	Particle* p1;
	Particle* p2;
	float springK;
	float restLength;
	bool handSpring;
	GsVec center;
	Spring(Particle* p1n, Particle* p2n);
	GsVec getForce(Particle* p);
	void update();
};

	class ParticleIK
{
public:
	SnManipulator* sikManip;
	SnGroup* spheres;
	SnGroup* lines;
	float stiffness;
	float handStiffness;

	float damping;
	GsArray<Particle*> particles;
	GsArray<Spring*> springs;
	
	int numParticles;
	float stepSize;
	float grav;
	GsVec shoulderAxis;
	KnSkeleton* skel;
	KnJoint* effector;
	KnJoint* root;
	//for hoap
	KnJoint* elbow;
	KnJoint* hand;
	KnJoint* shoulder;
	KnJoint* hips;

	Particle* elbowP;
	Particle* handP;
	Particle* handPFL;
	Particle* handPFD;
	Particle* handPFR;
	Particle* handPF;
	Particle* shoulderP;
	Particle* hipsP;

	Particle* effPF;
	Particle* effPFR;
	Particle* effPFL;
	Particle* effPFD;
	Particle* effP;





	void setEffector(GsVec pos, GsQuat rot);
	bool skelTargeted;
	void targetSkel(KnJoint* rootJ, KnJoint* effJ);
	void hoapArm(KnSkeleton* sk);
	void randomChain();
	void applyToHoapArm();
	ParticleIK();
	void addParticle(Particle* particle);
	void evaluate();
	void evaluateHoapArm();
	void reset();
	void init();
	void run();
	void applyToSkel();
	void snapToSkel();
	void makeSpring(Particle* p1, Particle* p2);

};

