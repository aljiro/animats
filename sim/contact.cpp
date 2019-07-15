#include "../include/contact.h"

// ContactRegion
void ContactRegion::add( CPoint& p ){
	for( int i = 1; i < points.size(); i++ )
		if( points[i].point == p.point )
			return;
	
	points.push_back( p );
}

void ContactRegion::prune(){
	// If the point is not in the plane anymore

	// If its is in the plane but not in the convex hull

}

void ContactRegion::compute(){

	

}

void ContactRegion::tighten(){
// Computes a plane - the contact region
	if( points.size() < 2 ) return;

	if( !fixed )
		compute();

	for( int i = 1; i < points.size(); i++ ){
		PointMass *p = points[i].point;
		p->position = this->P*p->position + this->t;
	}
}

// ContactList
ContactList::ContactList(){

}

ContactRegion& ContactList::get( int i, int j ){

}

void ContactList::tightenRegions(){

}

void ContactList::correctVelocities(){

}