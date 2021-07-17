


# include "draw_primitives.h"
#include <math.h>
#include "curve.h"
# include <fltk/gl.h>


void drawLine(GsVec a, GsVec b)
{
	glBegin ( GL_LINES);	
	glVertex3f (a.x, a.y, a.z);
	glVertex3f (b.x, b.y, b.z);
	glEnd();
}



void drawCircle(float rad,GsVec pos)
{
glBegin ( GL_LINE_STRIP );
	   for	   ( unsigned i=0; i<36; i+=10 ) 
	   {
		   glVertex3f ( pos.x+rad*cos(GS_TORAD(i)),pos.y,pos.x+rad*sin(GS_TORAD(i)));
	   }
glEnd();
}
void drawPolyline(std::vector<GsVec>& poly)
{	
glBegin ( GL_LINE_STRIP );
	   for	   ( unsigned i=0; i<poly.size(); i++ ) {
		   glVertex2f ( poly[i].x, poly[i].y );
	   }
glEnd();
}

void drawCurvePieces(Curve* c)
{
	if(c->first) {c->initTang(); c->first = 0;}
	c->update();
	c->makeSub();
	for(unsigned i=0;i<c->subCvs.size();i++)
	{
		drawCurve(c->subCvs[i]);
	}
}
int drawCurve(Curve* c)
{
	std::vector<GsVec>& poly = c->p;
	std::vector<GsVec>& curve = c->c;
	unsigned i;
	if(c->curveMode==BEZIERPIECES)
	{
		drawCurvePieces(c);
		glPointSize ( 10 );
		glColor3f ( 1,0,0);
		glBegin ( GL_POINTS );
		for ( i=0; i<poly.size(); i++ )
			glVertex2f ( poly[i].x, poly[i].y );
		glEnd();
		
		for ( i=0; i<poly.size(); i++ ) 
		{
			if( c->selectionState && i ==c->selection)
			{
				glPointSize ( 6 );
				glBegin ( GL_POINTS );
				glColor3f ( 0,0,1);
				glVertex3f ( c->t[i]->pi.x, c->t[i]->pi.y ,c->t[i]->pi.z );
				glVertex3f ( c->t[i]->po.x, c->t[i]->po.y ,c->t[i]->po.z );
				glEnd();
			}
		}
		for ( i=0; i<poly.size(); i++ ) 
		{
			if( c->selectionState && i ==c->selection)
			{
				glBegin(GL_LINES);
				glVertex3f ( c->t[i]->pi.x, c->t[i]->pi.y,c->t[i]->pi.z );
				glVertex3f ( c->p[i].x, c->p[i].y ,c->p[i].z );

				glVertex3f ( c->t[i]->po.x, c->t[i]->po.y, c->t[i]->po.z );
				glVertex3f (  c->p[i].x, c->p[i].y ,c->p[i].z );
				glEnd();
			}
		}
		return BEZIERPIECES;
	}else if(c->controlPoly)
	{
		glColor3f ( c->cpCol.x , c->cpCol.y, c->cpCol.z );
		drawPolyline(poly);
		// draw control points
		glColor3f ( c->cCol.x , c->cCol.y, c->cCol.z );
		glBegin ( GL_POINTS );
		for ( i=0; i<poly.size(); i++ ) glVertex3f ( poly[i].x, poly[i].y, poly[i].z  );
		glEnd();

	}
//draw curve
    if(poly.size()>2)
   {
		float dt =(float)(1.0/(c->divPerPair*(poly.size()-1)));
		curve.clear();
		GsVec pt;
			switch(c->curveMode)
			{
				
				case LEGRANGE:
					  for ( float t =0.0; t<=c->T + dt/2; t+=dt )
						{	
							curve.push_back(c->eval_lagrange(t));
						}
				break;
				case BEZIER:
					  for ( float t =0.0; t<=c->T  + dt/2; t+=dt )
						{
							GsVec p = c->eval_bezier(t);
							curve.push_back(p);
						}
				break;	
				case BSPLINEQUAD:
					for ( float t =2; t<=c->T*poly.size(); t+=dt )
						{
							curve.push_back(c->eval_bspline(t,2));
					    }

				break;
				case BSPLINECUBIC:
					for ( float t =3; t<=c->T*poly.size(); t+=dt )
						{
							curve.push_back(c->eval_bspline(t,3));
					  }
				break;
			}
}
glColor3f ( 0, 0, 1 );
drawPolyline(curve);
		
return 1;		
}