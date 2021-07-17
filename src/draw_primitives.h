# pragma once




class Track;
class Curve;
class GsVec;

void drawLine(GsVec a, GsVec b);
int drawCurve(Curve* curve);
void drawTrack(Track* t);
void drawCircle(float rad,GsVec pos);