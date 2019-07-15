#include "../include/animats.h"

using namespace Util;
using namespace arma;
using namespace std;

//Animat
Animat::Animat( int id, Body *body, Brain *brain ){
    this->id = id;
    this->body = body;    
    this->brain = brain;
}

/*
    "Voluntary" actions of the agent
*/
void Animat::act(){
    return;
}

void Animat::update( int step ){
    this->body->update( step );
}


void Animat::draw( SimGraphics& sgraphics ){

    body->draw(sgraphics);
}

Animat* Animat::clone(){

    Body *body = new Body( new CloneMeshProvider( this->body ), this->body->dk->alpha, 1 );
    Brain *brain = new DefaultBrain();
    Animat *a = new Animat( id, body, brain );

    return a;
}

// Plane
Plane::Plane( MeshProvider *mp, bool draw ){

    this->isdraw = draw;
    mp->populate(this);
}

void Plane::draw( SimGraphics& sgraphics ){
    if( isdraw ){
        std::array<float, 3> c = {0.5, 0.5, 0.5};     

        Color color;
        color.normal = c;
        color.highlighted = c;

        sgraphics.drawObject( (GeometricObject*)this, color, false );
    }
}

string Plane::toString(){
    return "plane";
}

double Plane::getMass(){
    return 10000000;
}