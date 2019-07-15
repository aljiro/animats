#include "../include/geometry.h"

using namespace arma;
using namespace std;
using namespace morph;

// GeometricTransform fields
morph::GeometricTransform::GeometricTransform(){
	this->M = { {1, 0, 0, 0}, 
				{0, 1, 0, 0}, 
				{0, 0, 1, 0},
				{0, 0, 0, 1} };
}

void morph::GeometricTransform::map( vector<PointMass *> *points ){
	Debug::log(string("Applying transform "));

	for( PointMass *p : *points ){
		vec x = {p->position(0), p->position(1), p->position(2), 1};
		vec xp = this->M*x;
		p->position = {xp(0), xp(1), xp(2)};
		p->originalPosition = {xp(0), xp(1), xp(2)};
	}
}

void morph::GeometricTransform::compose( GeometricTransform& GT ){
	this->M = GT.M*this->M;
}

// ScaleTransform
morph::ScaleTransform::ScaleTransform( double sx, double sy, double sz){
	this->M = { {sx, 0, 0, 0}, 
				{0, sy, 0, 0}, 
				{0, 0, sz, 0},
				{0, 0, 0, 1} };
} 

// RotateTransform
morph::RotateTransform::RotateTransform( double alpha, double beta, double gamma ){
	mat Rx = { {1, 0, 0, 0}, 
				 {0, cos(alpha), -sin(alpha), 0}, 
				 {0, sin(alpha), cos(alpha), 0},
				 {0, 0, 0, 1} };
	mat Ry = { {cos(beta), 0, -sin(beta), 0}, 
				{0, 1, 0, 0}, 
				{sin(beta), 0, cos(beta), 0},
				{0, 0, 0, 1} };
	mat Rz = { {cos(gamma), -sin(gamma), 0, 0}, 
				{sin(gamma), cos(gamma), 0, 0}, 
				{0, 0, 1, 0},
				{0, 0, 0, 1} };
	this->M = Rx * Ry * Rz;

}


// TranslateTransform
morph::TranslateTransform::TranslateTransform( double tx, double ty, double tz ){
	this->M = { {1, 0, 0, tx}, 
				{0, 1, 0, ty}, 
				{0, 0, 1, tz},
				{0, 0, 0, 1} };
}

void morph::GeometricObject::mapTransform( GeometricTransform& GT ){
	GT.map( &this->points );
}

void morph::GeometricObject::addPoint(vec position){
	PointMass *p1 = new PointMass();
	p1->position = position;
	this->points.push_back(p1);
}

void morph::GeometricObject::addFace( int i1, int i2, int i3){
	
	morph::Face *f = new Face( this->points[i1], this->points[i2], this->points[i3] );		
	f->setIndexes( i1, i2, i3 );
	this->faces.push_back( f );
}

void morph::GeometricObject::addFace( int i1, int i2, int i3, vec normal){
	
	morph::Face *f = new Face( this->points[i1], this->points[i2], this->points[i3] );		
	f->setIndexes( i1, i2, i3 );
	this->faces.push_back( f );
	f->normal = normal;
}