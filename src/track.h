# ifndef TRACK_H
# define TRACK_H

#include "app_main.h"
#include "curve.h"
#include <gsim/kn_motion.h>

class Track;
class Jnt;
class Curve;

class Timeline
{
public:
	std::vector<Track*> tracks;
	void init();
	Timeline();
};
class Jnt{
public:
GsString name;
std::vector<int> dof;
Jnt(GsString n){name =n;}
};
class KeyGroup
{
public:
	GsString name;
	KnSkeleton* sk;
	std::vector<Jnt*> jnts; 
	KeyGroup(GsString n,KnSkeleton* k){ name=n; sk = k;}
};
class Track : public KnMotion
{
  public:
	  bool IKTrack;
  bool empty; /*!whether or not keys have been placed yet*/
  int timeChannel;
  float start,end;
  GsColor visCol;
  KeyGroup* kg;
  GsVec mouseHit; //the position of the mouse click
  float startHit,endHit;
  int startLevel;
  bool washit;
  bool selected; //whether a track is selected or not
  std::vector<Curve*> cvs;
  Track();
  Track(GsString n,KeyGroup* keyg);
  bool hit(GsVec mousP);
  void move(GsVec newLoc);
  void setTime(float kt);
  void init();
};

#endif
