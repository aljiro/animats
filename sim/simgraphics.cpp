#include "../include/simgraphics.h"

using namespace std;
using namespace arma;

SimGraphics::SimGraphics(): fix(3, 0.0), eye(3, 0.0), rot(3, 0.0){

}

void SimGraphics::init( string winTitle, int width, int height ){
	double rhoInit = 7.0;

	displays.push_back(morph::Gdisplay(
		width, height, 100, 0, 
		winTitle.c_str(), rhoInit, M_PI/4,  M_PI/2
	));

    displays.back().resetDisplay (fix, eye, rot);
    displays.back().redrawDisplay();

    //this->animatInfoWindow = new AnimatInfoWindow( &displays[0], this->width, this->height, true );
}

void SimGraphics::takeSnapshot( int step ){
	Debug::log("Taking snapshot");
	morph::Gdisplay display = this->displays[0];
	string fname = "captures/frame" + to_string(step) + ".jpeg";
	display.saveImage( fname );
}

void SimGraphics::reset(){
	
	this->displays[0].resetDisplay (fix, eye, rot);
}

void SimGraphics::redraw(){
	this->displays[0].redrawDisplay();
}

void SimGraphics::drawObject( GeometricObject* go, Color color, bool drawPoints ){
	morph::Gdisplay display = this->displays[0];
    vector<Face *> faces = go->faces;
    vector<PointMass *> points = go->points;

    for( int i = 0; i < faces.size(); i++ ){
        
        Face *f = faces[i];
        vec u1 = f->points[0]->position;
        vec u2 = f->points[1]->position;
        vec u3 = f->points[2]->position;
        std::array<float,3> p1 = {u1[0], u1[1], u1[2]};
        std::array<float,3> p2 = {u2[0], u2[1], u2[2]};
        std::array<float,3> p3 = {u3[0], u3[1], u3[2]};        

        display.drawTriFill( p1, p2, p3, 
        					 f->iscontact?color.highlighted:color.normal );
    }

    if( drawPoints ){
    	vector<double> c = {0.3, 0.3, 0.3};
	    // Drawing points
	    for( int i = 0; i < points.size(); i++ ){

	        display.drawSphere( points[i]->position(0),
	                      		points[i]->position(1),
	                      		points[i]->position(2),
	                      		points[i]->radius,
	                      		points[i]->iscontact?c:c, 9);
	    }
	}
}