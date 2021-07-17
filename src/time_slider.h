
# ifndef TIME_SLIDER_H
# define TIME_SLIDER_H

# include <gsim/kn_motion.h>
# include <fltk/GlWindow.h>
# include "app_viewer.h"

class AppViewer;

class Track;
class TimeSlider : public fltk::GlWindow
 { private :
   public:
	
   AppViewer* viewer;
   Track* timeline;
   Track* IKtimeline;
   double startTime,endTime;
   int fps;
   GsVec mseTime;
int moveKey,move;
	float kt;
	double currentTime;
	int W,H;			 //width and height of window
	int size;
	int brushSize;
	void ikKey();
	void ikKey(double kt);
	void jointKey();
	void jointKey(double kt);
	float* pixels;
	bool frameTog;
	int timeToWin(float tm);
float winToTime(int x);
	//void resize(int w,int h);
void play();
void setPixel(int x,int y, float r,float g,float b);
float getPixel(int x,int y);
void flood(float r,float g,float b);

void drawSquare(int x,int y, int w, int h,float r, float g, float b);

void circleMidpoint(int xCenter, int yCenter, int radius, float r,float g, float b);
void circlePoints(int cx, int cy, int x, int y,float r,float g, float b);
    TimeSlider ( int x, int y, int w, int h, const char *l );
	
	GsVec win2scene ( int x, int y );
	void appMouseFunc (  int x, int y );
	void appMotionFunc ( int x, int y ) ;
   public :
    virtual void draw ();
    virtual int handle ( int ev );
};

#endif // TIME_SLIDER_H