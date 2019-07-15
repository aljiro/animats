#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "primitives.h"
#include "util.h"

#include <vector>
#include <iostream>
#include <armadillo>

using namespace arma;
using namespace std;
using namespace Util;
using namespace morph;

class CollisionManager;
class SimGraphics;

namespace morph{

class GeometricTransform{
protected:
	mat M;
public:
	GeometricTransform();
	void map( vector<PointMass *> *points );
	/* 
		Computes the composition:
	   	gt0.compose( gt1 ) => gt0 = gt1*gt0
	*/
	void compose( GeometricTransform& GT );
};

class ScaleTransform : public GeometricTransform{
public:
	ScaleTransform( double sx, double sy, double sz);
};

class RotateTransform : public GeometricTransform{
public: 
	RotateTransform( double alpha, double beta, double gamma );
};

class TranslateTransform : public GeometricTransform{
public: 
	TranslateTransform( double tx, double ty, double tz );
};

class GeometricObject{
public:
    CollisionManager* cm; // to be injected
    
	vector<Face*> faces;
	vector<Edge> edges;
	vector<PointMass*> points;

    void mapTransform( GeometricTransform& gt );
    void addPoint(vec position);
    void addFace( int i1, int i2, int i3);
    void addFace( int i1, int i2, int i3, vec normal);
    virtual double getMass() = 0;
    virtual void draw( SimGraphics& sgraphics ) = 0;
    virtual string toString() = 0;
};

}

#endif