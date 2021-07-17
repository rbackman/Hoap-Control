# pragma once
class AppMainWin;
class AppNLAEditor;
class AppGraphViewer;
class AppViewer;
class TimeSlider;
class WalkGenerator;
class AppFootViewer;
class WalkControler;

# include <vector>
# include <gsim/gs_vars.h>
# include <gsim/hm_humanoid.h>
# include <gsim/kn_ik_body.h>
#include <gsim/gs_vec.h>


# include "hoap_client.h"
# include "time_slider.h"
# include "walk_generator.h"
# include "vicon_client.h"
# include "walk_controler.h"
# include "jacobian_ik.h"

class ParticleIK;
class Jacobian;
// this class contains pointers to the main
// classes in the application, so that they
// can be accessed from anywhere in the code
class AppMain
 { public :
bool pikRunning;
	ParticleIK* pIK;
	Jacobian* jacobian;
	bool runJacobian;
	float maxHipHeight;
   ViconClient* vicon_client;
   RakNetTime netTime;
   SnManipulator* JIKmanip;
   SnManipulator* JIKobstacle;
	bool vicon_started;
	bool moveRoot;
	WalkControler* cont;
	bool controlMade;
	AppFootViewer* foot;
    AppMainWin* mainwin;
	TimeSlider* timeslider;
	AppNLAEditor* nla;
	AppGraphViewer* graph;
	AppViewer* viewer;
	WalkGenerator* walk;
    GsVars* vars;
	HmHumanoid* sk;  //hoap skeleton
	HmHumanoid* ssk; //skeleton according to sensors
  bool stream;
	bool goalMade;
	bool walkMade;
	bool armIKActive;
	GsVec ikStart[3];
	GsVec leftFootStart,rightFootStart,rootStart;
	KnSkeleton* sc;   //scene skeleton
    KnIkBody* hikb;   //hoap ik group
	KnSkeleton* ikskel;
	HoapClient* hoap_client;   //for communication with robot

	bool axis;
	bool realtime;

	int intStep;
    AppMain ();
	int getPosture(float*);
    void read_config_file ( const char* cfgfile );
	void load_config();
	void save_config();
	void checkVicon();
	void checkHoap();

	void setTaskFrame(float taskFrme);
	GsVec wipeCenter;
	double wipeFrameTime;
	float taskFrame;
	bool playTask;
	void wipe();
	void makeWipe();
	SnLines* taskLines;
 };

// there is only one instance of AppMain, which
// is allocated in app_main.cpp and which can be
// accessed with the pointer below:
extern AppMain* App; 

