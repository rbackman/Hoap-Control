
# ifndef APP_NLA_WINDOW_H
# define APP_NLA_WINDOW_H

# include <gsim/fl_viewer.h>
# include <fltk/GlWindow.h>
#include "curve.h"
#include "app_main.h"
#include "track.h"


class AppNLAEditor : public FlViewer
 { private :
	
    std::vector<Track*> tracks;

	float pickprec;      //radius of influence for mouse events
	int W,H;			 //width and height of window
	int gridW;

	GsString message1,message2;
	int selTrack;
	GsVec mseTrack;
 public:
	    int curTrack;
	 	int trackH;
    AppNLAEditor ( int x, int y, int w, int h, const char *l );
	GsVec win2scene( int x, int y );
	void appMouseFunc (  int x, int y );
	void appMotionFunc ( int x, int y ) ;
	void drawGrid();
	/*!put a selected track at current time and channel*/
	void insertTrack();
	/*!get current track in focus*/
	Track* getTrack();
	/*!draw a track in the nle*/
	void drawTrack(Track* t);
	/*!creates curves from motion*/
	void updateCurves();
	/*!track list selected*/
	void trackList();
	/*!index of current track selected*/ 
	int trackSelected();
	/*!add a new track to the nla*/
	void addTrack(Track* t);
/*!get list of all selected tracks*/
	std::vector<GsString> getSelected();
	void init ();
	void loadTracks();
   public :
    virtual void draw ();
	virtual int handle ( int ev );
	 
};

#endif // APP_GL_WINDOW_H
