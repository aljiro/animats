#ifndef BODY_H
#define BODY_H

#include <vector>
#include <iostream>
#include <armadillo>

#include "util.h"
#include "primitives.h"
#include "cluster.h"
#include "deformation.h"
#include "meshes.h"
#include "geometry.h"
#include "collisions.h"
#include "simgraphics.h"

using namespace arma;
using namespace std;
using namespace morph;

/* The body contains the physical properties of the object */
class Body : public GeometricObject{
protected:
    double h;
    vec fext; // All the external forces
    vector<double> *color;
    int N;

public:
    DeformationKernel* dk;
    Body( MeshProvider* mp, double alpha, double density );

    void init( double density);
    void updateVelocities( int step );
    void updatePositions( int step );
    void addExternalForce( double fx,double fy,double fz );
    vector<double> bounds();
    vec getCm();
    vec getOriginalCm();
    int numPoints();
    void setSoftness( double alpha );
    double getMass();
    void fixNormals();
    void compensateMomentum( );
    void draw( SimGraphics& sgraphics );
    string toString();
};
#endif
