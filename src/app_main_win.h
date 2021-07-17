# pragma once
# include <vector>
# include <gsim/gs_vars.h>
# include <gsim/fl_vars_win.h>

# include "app_fluid.h"
# include "app_main.h"
#include "../../hoapnet/server/robotapi.h"
#include <gsim/fl.h>



class AppMainWin : public AppFluid
 { private :
    FlVarsWin* _vtwin;
    GsString _buf;
	
int getSelectedKey();
   public :
int ikCreated;
    AppMainWin ();
   ~AppMainWin ();
    void show ();
	void updateSliders();
	void orientIK();
	void message ( const char* s ) { ui_message->label(s); ui_message->redraw(); fltk::check();   }
    void message ( const char* s, int i ) { _buf.setf(s,i); ui_message->label(_buf); ui_message->redraw(); fltk::check(); }
    void message ( const char* s, float f ) { _buf.setf(s,f); ui_message->label(_buf); ui_message->redraw(); fltk::check(); }
	void matchIK();
	void sendPosture();
 
	void showLog(hoap_data_log log);

	hoap_posture getPosture();
	rt_msg getMesg();
   public :
    virtual void event ( AppEvent e );
 };

