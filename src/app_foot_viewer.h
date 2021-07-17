
# ifndef APP_FOOT_WINDOW_H
# define APP_FOOT_WINDOW_H

# include <gsim/fl_viewer.h>
# include <fltk/GlWindow.h>

#include "curve.h"
#include "app_main.h"
#include "foot.h"


class AppFootViewer : public FlViewer
 { private :

std::vector<Curve*> cvs;
std::vector<Foot*> feet;
std::vector<Foot*> keyfeet;

	float pickprec;      //radius of influence for mouse events
	int W,H;			 //width and height of window
	int gridW,gridH;
	int setMode;
	GsString message1,message2;
	int selTrack;
	bool drawCurves;
	GsVec stepStart,stepEnd;
	GsVec mse,mseStart,mseGrid;
	GsVec stepStartDir,stepEndDir;
 public:
	 	
    AppFootViewer ( int x, int y, int w, int h, const char *l );
	GsVec win2scene( int x, int y );
	void appMouseFunc (  int x, int y );
	void appMotionFunc ( int x, int y ) ;
	void drawGrid();
	void drawCircle(float rad, GsVec pos);
	void init ();
	void clearCurves();
	void laySteps();
   public :
    virtual void draw ();
	virtual int handle ( int ev );
	 
};

#endif // APP_GL_WINDOW_H
