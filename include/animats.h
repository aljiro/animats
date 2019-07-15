#ifndef ANIMATS_H
#define ANIMATS_H

#include "primitives.h"
#include "simgraphics.h"
#include "geometry.h"
#include "brain.h"
#include "util.h"
#include "body.h"

#include <vector>
#include <iostream>
#include <armadillo>

using namespace arma;
using namespace std;
using namespace Util;
using namespace morph;

class Animat{
private:

public:
    Body* body;
    Brain* brain;
    int id;

    Animat( int id, Body* body, Brain* brain );

    /*
        "Voluntary" actions of the agent
    */
    void act();
    /* 
        Update the dynamics:
        - Get collition spring forces
        - Integrate deformation compoentns
        - External forces
        - And integrate the equations of motion
    */ 
    void update( int step );

    /* Draws himself using the simGraphics interface */
    void draw( SimGraphics& sg );

    Animat* clone();
};

/*
    A plane
*/
class Plane : public GeometricObject{
private:

public:
    bool isdraw;

    Plane( MeshProvider *mp, bool draw );
    void draw( SimGraphics& sgraphics );
    double getMass();
    string toString();

};
#endif