#ifndef COLLISIONS_H
#define COLLISIONS_H 

#include <vector>
#include <iostream>
#include <armadillo>
#include <map>
#include <list>
#include <time.h>

#include "primitives.h"
#include "geometry.h"
#include "util.h"

using namespace std;
using namespace arma;
using namespace Util;
using namespace morph;

class Box{
public:
    vec min;
    vec max;

    Box();
    Box( vec min, vec max );
    bool collide( Box& b );

    // static
    static void compute( vector<PointMass*>& points, Box *b ){

    	vec pmin = points[0]->position;
    	vec pmax = points[1]->position;

    	for( PointMass* pm: points ){
    		vec p = pm->position;

    		if( p(0) < pmin(0) ) pmin(0) = p(0);
    		if( p(1) < pmin(1) ) pmin(1) = p(1);
    		if( p(2) < pmin(2) ) pmin(2) = p(2);

			if( p(0) > pmax(0) ) pmax(0) = p(0);
    		if( p(1) > pmax(1) ) pmax(1) = p(1);
    		if( p(2) > pmax(2) ) pmax(2) = p(2);
    	}

    	b->min = pmin;
    	b->max = pmax;
    }
};

typedef struct {
	list<int> items;
	int timestamp;
} CHashItem;

typedef struct {
	PointMass* point;
	GeometricObject* go;
	int originalIdx;
} CPoint;

typedef struct {
	morph::Face* face;
	GeometricObject* go;
	Box *aabb;
	int originalIdx;
} CFace;

typedef struct {
	double depth;
	vec faceProjection;
	CPoint cpoint;
	CFace cface;
	int pindex;
} CollisionInformation;

class CHashTable{

	const unsigned long p1 = 73856093;
	const unsigned long p2 = 19349663;
	const unsigned long p3 = 83492791;
public:
	int n;
	double l; // Cell size
	CHashTable( int n, double l );
	vector<CHashItem> hashes;

	int discretize( double a );
	void discretizeBox( Box *b );
	unsigned int getHash( vec point );
	unsigned int getHashDiscrete( vec p );
	void hashIn( vec point, int index, int step ); 
	CHashItem getItem( unsigned int h );

};

/* Scanning based on Teschner et al. 2003 */
class CollisionManager{
private:
	vector<CPoint> points;
	vector<CFace> faces;
	map<GeometricObject*, vector<int>> indexes;
	map<GeometricObject*, int> objects;
	ContactList contacts;
	// Hash tables
	CHashTable ht;
	// Coll cision spring constant
	const double Kc = 100000;

	void freePoints();
	void compensateVelocities( GeometricObject *go, vec vc ); 
	void handleCollisions( CFace cf, CHashItem chi, int step );
	void evaluateContacts( CFace cf, int step );
	void firstPass( int step ); // Hashes points and computes aabb
	void secondPass( int step ); // Check the faces and handles collisions
public:
// Scans the space
	CollisionManager();
	void registerObject( GeometricObject *go );
	void findCollisions( int step );
	/* Computes the physically based collision response */
	vec getRigidBodyResponse( GeometricObject *o1, GeometricObject *o2, vec penetration );
	vec addCollisionResponse( CFace& cf, CPoint& cp );
	void correctVelocities();

};


#endif