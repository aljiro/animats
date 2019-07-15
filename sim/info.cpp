#include "../include/info.h"

using namespace arma;
using namespace std;

//AnimatInfoWindow

AnimatInfoWindow::AnimatInfoWindow( morph::Gdisplay *disp, int rwidth, int rheight, bool docked ){
	this->gdisp = disp;
	this->width = 300;
	this->height = 200;
	this->screen = DefaultScreen( gdisp->disp );
	XVisualInfo vinfo;
	XMatchVisualInfo( gdisp->disp, screen, 32, TrueColor, &vinfo );
	XSetWindowAttributes attr;
	attr.colormap = XCreateColormap(gdisp->disp, docked?disp->win:disp->root, vinfo.visual, AllocNone);
	attr.border_pixel = 0;
	attr.background_pixel = 0x80808080;
	infoPanel = XCreateWindow( gdisp->disp, docked?disp->win:disp->root, 
							   rwidth - this->width, rheight - this->height, this->width, this->height, 0, vinfo.depth, 
							   InputOutput, vinfo.visual, CWColormap | CWBorderPixel | CWBackPixel, &attr);
	// infoPanel = XCreateSimpleWindow(gdisp->disp, gdisp->win, 
				// 			   		rwidth - this->width, rheight - this->height, this->width, this->height, 
				// 			   		5, WhitePixel(gdisp->disp, screen), BlackPixel(gdisp->disp, screen));


	XSelectInput( gdisp->disp, infoPanel, ExposureMask | KeyPressMask );
	XMapWindow( gdisp->disp, infoPanel );
}

void AnimatInfoWindow::draw( Animat &a ){
	XClearWindow(this->gdisp->disp, this->infoPanel);
	GC gc = XCreateGC(this->gdisp->disp, this->infoPanel, 0, 0);
	XSetFillStyle(this->gdisp->disp, gc, FillSolid);
	// Draw line		
	XDrawLine(this->gdisp->disp, this->infoPanel, gc, 10, 20, this->width-10, 20 );
	// Draw skin map
	vector<PointMass *> points = a.body->points;
	int d = 6;	
	XColor xcolour1, xcolour2;
	xcolour1.red = 32000; xcolour1.green = 65000; xcolour1.blue = 32000;
	xcolour1.flags = DoRed | DoGreen | DoBlue;
	xcolour2.red = 65000; xcolour2.green = 00000; xcolour2.blue = 00000;
	xcolour2.flags = DoRed | DoGreen | DoBlue;
	XAllocColor(this->gdisp->disp, DefaultColormap(this->gdisp->disp,this->screen), &xcolour1);
	XAllocColor(this->gdisp->disp, DefaultColormap(this->gdisp->disp,this->screen), &xcolour2);
	int area = 0;

	for( PointMass *p : points ){
		vec x = map(a, p->originalPosition);
		area += p->iscontact;
		XSetForeground(this->gdisp->disp, gc, p->iscontact?xcolour2.pixel:xcolour1.pixel);
		XDrawArc(this->gdisp->disp, this->infoPanel, gc, x(0)-(d/2), x(1)-(d/2), d, d, 0, 360*64);
	}

	area *= 100/points.size();
	// Draw title info
	string title = string("Contact detail animat ") + to_string(a.id) + " - area: " + to_string(area) + "%";
	XSetForeground(this->gdisp->disp, gc, BlackPixel(this->gdisp->disp, this->screen));
	XDrawString(this->gdisp->disp, this->infoPanel, gc, 10, 15, title.c_str(), title.length() ); 
	XFlush(this->gdisp->disp);
	//exit(0);
	
}	

vec AnimatInfoWindow::map( Animat& a, vec p ){
	vec e = {1, 0, 0};
	mat Pr = eye(3,3) - e*e.t();
	mat T = {{0, 1, 0},
			 {0, 0, 1}};

	vec t, np;
	double w = this->width;
	double h = this->height;
	double k = 28;
	double r = 1;
	double nr = 0.95*w/4;
	vec cm = a.body->getOriginalCm();
	np = (p - cm);

	if( dot(e, np) > 0 )
		t = {w/4, k+w/4};
	else
		t = {3*w/4, k+w/4};

	np = Pr*np;
	np = T*np;
	np = nr*np/r + t;
	return np;
}