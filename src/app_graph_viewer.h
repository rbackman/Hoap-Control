
# ifndef APP_GLPA_WINDOW_H
# define APP_GLPA_WINDOW_H

# include <gsim/fl_viewer.h>
# include <fltk/GlWindow.h>
#include "curve.h"
#include "app_main.h"
#include "track.h"

#define TANG_EDIT 2
#define POINT_EDIT 1
class CurveGroup
{
	public:
	GsString name;
	std::vector<GsString> jnts;
	std::vector<Curve*> cvs;
	CurveGroup(GsString n){name = n;}
};
class AppGraphViewer : public FlViewer
 { private :
	std::vector<KeyGroup*> keyGroups;
	std::vector<CurveGroup*> cvGroups;
	/*!current curves being displayed*/
	std::vector<Curve*> cvs; 

	int emptyTimeline;
	int emptyTrack;
	/*!The current keygroup for the graphviewer */
	int curKG;
	/*!Extra curve for graphviewer*/
	Curve* c;
	/*!radius of influence for mouse events*/
	float pickprec;   
	/*!width and height of window*/
	int W,H;	
	/*!reference in screen coords to determine origin of graph*/
	int Origin;
	/*!pixel size of grids*/
	int gridW;
	int gridH;

	/*!most recent mouse event location*/
	GsVec mseGraph;
	/*! message for mouse*/
	GsString  message1;
	GsString  message2;
public:
	AppGraphViewer ( int x, int y, int w, int h, const char *l );
	void init ( );

	/*!screen space to scene  pixels to pos*/
	GsVec win2sceneGraph ( int x, int y );
		/*!position to screen space*/
	GsVec scene2winGraph ( int x, int y );
	/*!draw circle at given location*/
	void drawCircle(float rad, GsVec pos);
	/*!called on mouse press*/
	void appMouseFunc (  int x, int y );
	/*!called on mouse drag*/
	void appMotionFunc ( int x, int y ) ;
	/*!*/
	void drawGrid();
	/*!free selected tangent*/
	void freeTangent();
	/*!make a keygroup*/
	void makeGroup();
	/*!show keygroup selected*/
	void showGroup();
	/*!make track with selected keygroup*/
	void makeTrack();
	/*!get index of group selected*/
	int getKeyGroupNum();
	/*!get pointer to selected keygroup*/
	KeyGroup* getKeyGroup();
	/*!show list with all available channels*/
	void loadChannels();
	/*!called when channel list is selected*/
	void channelSel();
	/*!create a curve with the given info*/
	Curve* makeCurve(Track* m,KnJoint* j,int dof);
	/*!check to see if joint is selected in graph viewer*/
	bool jSelected(GsString j);

	public :
	virtual void draw ();
	virtual int handle ( int ev );
	 
};

#endif // APP_GL_WINDOW_H
