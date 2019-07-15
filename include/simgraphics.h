#ifndef SIM_GRAPHICS_H
#define SIM_GRAPHICS_H

#include "geometry.h"
#include "primitives.h"

#include <vector>
#include <iostream>
#include <armadillo>

#include <GL/glut.h>
#include <GL/glx.h>
#include <GL/glu.h>

#include "morph/display.h"
#include <X11/Xlib.h>
#include <X11/X.h>

using namespace std;
using namespace arma;
using namespace morph;

//class GeometricObject; // forward declaration
//#include "info.h"

typedef struct{
	std::array<float, 3> normal;   
    std::array<float, 3> highlighted;  
} Color;

class SimGraphics{
private:

	vector<double> fix;
    vector<double> eye;
	vector<double> rot;
	vector<morph::Gdisplay> displays;
	//AnimatInfoWindow *animatInfoWindow;

public:
	int width, height;
	SimGraphics();
	void init( string winTitle, int width, int height );
	void takeSnapshot( int step );
	void reset();
	void redraw();
	void drawObject( GeometricObject* go, Color color, bool drawPoints );
};

#endif