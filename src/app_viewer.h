# pragma once

# include <gsim/fl_viewer.h>
# include <gsim/kn_scene.h>
# include <gsim/sn_manipulator.h>
# include <gsim/kn_coldet.h>
# include <gsim/hm_scene.h>
# include "track.h"

class AppMainWin;

class AppViewer : public FlViewer
 { public :
float W,H;
    SnGroup*  _root;
    HmScene*  _kns;
	HmScene*  _knss;
	KnScene*  _knsc;
	KnScene*  _iknsc;
	SnGroup*  _iksc;
	
SnGroup* _steps;
	SnGroup* _com;
	SnGroup* _coms;
	SnGroup* _support;
    SnGroup*  _ikmanips;
	SnGroup* _traj_lines;
	SnGroup* _task_lines;
	SnGroup* _lines;
	SnGroup* _sense_lines;
	SnManipulator* _RootIK;
	KnColdet* _coldet;

	int drawCvs;

   public:
    AppViewer ( int x, int y, int w, int h, const char *l=0 );
   ~AppViewer ();
    void init ( );
	void IKSlide();
	void updateIKSlide();
	void updateIKSkel();

	void update_robot () { _kns->update();_knsc->update();  _iknsc->update(); _knss->update();}
	int makeIK();

	virtual int handle_keyboard(const GsEvent &e);

    virtual void draw ();
    // several event handlers can be re-writen, below is on example; see the
    // base class header file for all possible event methods to be re-written
    virtual int handle_scene_event ( const GsEvent &e );
};

