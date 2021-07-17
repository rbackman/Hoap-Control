

#include "app_nla_viewer.h"
#include "track.h"
#include <gsim/gs_model.h>
#include <gsim/sn_model.h>
Timeline::Timeline()
{
printf("new timeline\n");
}
void Timeline::init()
{
	for(unsigned i=0;i<tracks.size();i++)
	{
		tracks[i]->init();
	}
}

void Track::move(GsVec newLoc)
{
	float x = mouseHit.x;
	float nx = newLoc.x;
	float dx = nx - x;
	start = startHit+dx;
	end = endHit;

	float y = mouseHit.y;
	float ny = newLoc.y;
	float dy = ny-y;
	int trackH = App->nla->trackH;
	if(abs(dy)> trackH)
	{
		timeChannel = (int)(startLevel+dy/trackH);
	}
	
}
bool Track::hit(GsVec mouseP)
{
	if(mouseP.x>start&&mouseP.x<end)
	{
		printf("hit track");
		startHit = start;
		endHit = end;
		mouseHit.set(mouseP);
		washit = true;
		startLevel = timeChannel;
	}
	else 
	{
		washit = false;
	}
	return washit;
}
Track::Track()
{
empty=true;
IKTrack = false;
}

Track::Track(GsString n,KeyGroup* keyg):KnMotion()
{
	KnMotion::KnMotion();
	Track();
	IKTrack = false;
	start = 0.0;
	end = 50.0;
	timeChannel=0;
	visCol.set(1,0,0);
	kg = keyg;
	KnMotion::name(n);
	KnPosture* p =  new KnPosture(kg->sk);
    p->get();
    KnMotion::connect(p);
	init();
}
void Track::init()
{
	KnMotion::init();
	cvs.clear();
	for(unsigned i=0;i<kg->jnts.size();i++)
	{
		KnJoint* j = kg->sk->joint(kg->jnts[i]->name);
		if(j->rot()->euler()->frozen(0)) cvs.push_back( new Curve(this,j,0 ) );
		if(j->rot()->euler()->frozen(1)) cvs.push_back( new Curve(this,j,1 ) );
		if(j->rot()->euler()->frozen(2)) cvs.push_back( new Curve(this,j,2 ) );
		if(j->pos()->frozen(0)) cvs.push_back( new Curve(this,j,3 ) );
		if(j->pos()->frozen(1)) cvs.push_back( new Curve(this,j,4 ) );
		if(j->pos()->frozen(2)) cvs.push_back( new Curve(this,j,5 ) );

	}
	
}