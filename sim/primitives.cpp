#include "../include/primitives.h"

using namespace arma;
using namespace std;
using namespace Util;
using namespace morph;
//PointMass
PointMass::PointMass(){
    this->originalPosition = zeros<vec>(3);
    this->position = zeros<vec>(3);
    this->nposition = zeros<vec>(3);
    this->goalPosition = zeros<vec>(3);
    this->velocity = zeros<vec>(3);
    this->vc = zeros<vec>(3);
    this->vi = zeros<vec>(3);
    this->ve = zeros<vec>(3);
    this->pre = NULL;

    mass = 1.0;
    moved = false;
    radius = 0.01;
    iscontact = false;
}


void PointMass::updateExternal( vec ve){
	if( !this->iscontact ){
		this->ve += ve;
	}else
		this->ve = 0*this->ve;
}

void PointMass::updateVelocity(  vec ve, vec vi, double h ){
	double c = 0.1;// damping 
	this->velocity += (1-c)*this->vi + this->ve + this->vc;
	this->vc = zeros<vec>();
}

void PointMass::updatePosition( double h ){
	this->position += h*this->velocity;	
}


//Edge
Edge::Edge(  PointMass* v0, PointMass* v1 ){
	this->v0 = v0;
	this->v1 = v1;
}

PointMass* Edge::getHead(){
	vec u = this->v1->position - this->v0->position;

	if( dot(u, this->v1->velocity) > 0 ) 
		return this->v1;
	else
		return this->v0;
}

//Face

Face::Face(PointMass *p1, PointMass *p2, PointMass *p3){
	this->recompute = true;
	this->iscontact = false;
	
	this->points.push_back( p1 );
	this->points.push_back( p2 );
	this->points.push_back( p3 );

	addEdge( 0, 1 );
	addEdge( 1, 2 );
	addEdge( 2, 0 );
}

void Face::setIndexes( int i1, int i2, int i3 ){
	this->indexes.push_back(i1);
	this->indexes.push_back(i2);
	this->indexes.push_back(i3);
}	

void Face::addEdge( int i, int j ){
	Edge e( this->points[i], this->points[j] );
	this->edges.push_back(e);
}

/*
	Computes the normal vector to the face for a particular 
*/
void Face::computeNormal( ){
	//for( int i = 0; i < this->edges.size(); i++ )
	//	cout << "Edge " << i << ": " << this->edges[i].v0 << "->" << this->edges[i].v1 << endl; 

	PointMass e1 = *(this->points[0]);
	PointMass e2 = *(this->points[1]);
	PointMass e3 = *(this->points[2]);

	vec u1 = e1.position - e2.position;
	vec u2 = e2.position - e3.position;

	this->normal = cross( u1, u2 );	
	double n = norm(this->normal);

	this->normal /= n==0?1:norm(this->normal);
	
}

/* 
	Predicate type A. Requires:
	1. A point p in the face
	2. An extreme v of the edge in the colliding shape

	The test indicates if one of the extremes of the edge is on the positive 
	half space of the plane that contains the face
*/
bool Face::A( PointMass face_point, vec edge_vector ){

	// cout << "Penetration test:\n-------" << endl;
	// cout << "Face point: " << printvec(face_point.position) << endl;
	// cout << "Edge point: " << printvec(edge_vector) << endl;
	// cout << "Normal: " << printvec(this->normal) << endl;

	vec u = face_point.position;
	return dot( this->normal, edge_vector - u ) > 0;
}

/*
	Predicate type B. Requires:
	1. An edge in the face with extremes vn, vm
	2. An edge of the incident object with extremes vl, vk

	
*/
bool Face::B( PointMass vn, PointMass vm, PointMass vl, PointMass vk ){
	vec e_face = vm.position - vn.position;
	vec e_object = vk.position - vl.position;
	vec v = vm.position - vk.position;

	vec u = cross( e_face, e_object );
	return dot( u, v ) > 0;
}

bool Face::penetrates( PointMass *p ){

	if( this->recompute ){
		//Debug::log("Computing normal to the face.");
		this->computeNormal();
	}

	if( !(this->points[0]->pre != NULL && 
		this->points[1]->pre != NULL &&
		this->points[2]->pre != NULL && 
		p->pre != NULL) )
		return false;

	vec p1 = this->points[0]->pre->position;
	vec p2 = this->points[1]->pre->position;
	vec p3 = this->points[2]->pre->position;
	vec normal_past = cross( p1 - p2, p2 - p3 );

	if( dot(this->normal, normal_past) <  0 )
		normal_past = -normal_past;

	vec u_past = this->points[0]->pre->position;
	vec u = this->points[0]->position;
	
	double a = dot( this->normal, p->position - u );
	double b = dot( normal_past, p->pre->position - u_past );

	if( ((a > 0  && b < 0) || (a < 0 && b > 0 )) && 
		this->isInside(p->position) ){		
		return true;
	}
	else
		return false;
}

/* 
	Checks the predicates for face piercing.
*/
PenetrationInfo Face::isPenetrated( Edge&  e ){

	bool apred_out, apred_in, bpred_out, bpred_in, b, O_out, O_in;
	PenetrationInfo contact;
	
	if( this->recompute ){
		//Debug::log("Computing normal to the face.");
		this->computeNormal();
	}

	apred_out = (not this->A( *(this->points[0]), e.v1->position ) ) && 
		       		 this->A( *(this->points[0]), e.v0->position );
    //Debug::log("Computing predicate A_in.");
	apred_in = 	     this->A( *(this->points[0]), e.v1->position ) &&
		     	(not this->A( *(this->points[0]), e.v0->position ) );
	bpred_out = true;
	bpred_in = true;
	
	//Debug::log("Computing predicates type B");
	for( int i = 0; i < this->edges.size(); i++ ){
		b = this->B( *(edges[i].v0), *(edges[i].v1),
					 *(e.v0), *(e.v1) );
		bpred_out = bpred_out && b;
		bpred_in = bpred_in && !b;   
	}

	O_out = apred_out && bpred_out;
	O_in = apred_in && bpred_in;

	contact.result = O_out || O_in;
	contact.faceEdge = apred_in || apred_out;
	contact.edgeEdge = bpred_in || bpred_out;

	return contact;
}

vec Face::faceObjectCentroid( vector<PointMass *>& face_points ){
	vec vv = zeros<vec>(3);

	for( int i = 0; i <face_points.size(); i++ )
		vv += face_points[i]->position;

	vv /= face_points.size();
	return vv;
}

double Face::getPenetrationDepth( Edge& e ){	

	vec v = e.v1->position - e.v0->position;
	// if( dot(v, this->normal) >= 0){

	// 	this->normal = -this->normal;
	// }

	vec n = this->normal;

	vec f_cm = (this->points[0]->position +
			   this->points[1]->position + 
			   this->points[2]->position)/3.0;
	vec a = e.v0->position - f_cm;
	vec b = e.v1->position - f_cm;

	double tp = -(dot(n, a))/(dot(n, b-a));
	vec g = b*tp + a*(1-tp);
	g += f_cm;
	double d = norm( e.v1->position - g );

	return d ; 
}

vec Face::getFaceProjection( Edge& e ){
	vec v = e.v1->position - e.v0->position;
	// if( dot(v, this->normal) >= 0){

	// 	this->normal = -this->normal;
	// }

	vec n = this->normal;
	vec f_cm = (this->points[0]->position +
			   this->points[1]->position + 
			   this->points[2]->position)/3.0;
	vec fp_cm;
	if( this->points[0]->pre != NULL ){
		fp_cm = (this->points[0]->pre->position +
			   this->points[1]->pre->position + 
			   this->points[2]->pre->position)/3.0;
	}else
		fp_cm = f_cm;

	vec a = e.v0->position - f_cm;
	vec b = e.v1->position - f_cm;
	vec g;

	if( norm(e.v1->position - e.v0->position) < 0.001 && norm(f_cm - fp_cm) > 0.01){
		g = b - f_cm;
	}else{
		double tp = -(dot(n, a))/(dot(n, b-a));
		//tp = 0;
		g = b*tp + a*(1-tp);
		g += f_cm;
	}

	

	return g; 
}

vec Face::getFaceProjection2( Edge& e ){
	vec x0 = this->points[0]->position; // A point in the plane	
	// We calculate which of the points is in the interior
	vec u0 = e.v1->position - x0;


	//if( dot( u0, this->normal ) < 0 )
		return  e.v1->position + abs(dot( u0, this->normal ))*this->normal;
	//else
	//	return 0;
}


 // Old penetration depth
double Face::getPenetrationDepth2( Edge& e ){

	vec x0 = this->points[0]->position; // A point in the plane	
	// We calculate which of the points is in the interior
	vec u0 = e.v1->position - x0;


	//if( dot( u0, this->normal ) < 0 )
		return abs(dot( u0, this->normal ));
	//else
	//	return 0;
}

// Pointing outwards
void Face::fixNormalOrientation( vector<PointMass *>& face_points ){
	if( !this->recompute )
		return;
	else
		this->computeNormal();

	vec x0 = this->points[0]->position; 
	vec vv = faceObjectCentroid( face_points );

	vv -= x0;

	cout << "Fixing normal: " << printvec(x0) << " and " << printvec(this->normal) << endl;

	if( dot(vv, this->normal) > 0 )
		this->normal = -this->normal;
	
}

vec Face::getCentroid( ){
	vec p1 = this->edges[0].v0->position;
	vec p2 = this->edges[1].v0->position;
	vec p3 = this->edges[2].v0->position;

	return (p1 + p2 + p3)/3;
}

/*
 * Project the point onto the faces
 */
vec Face::project( Edge& e ){
	double d = getPenetrationDepth( e );
	vec npos = e.v1->position + d*this->normal;
	return npos;
}

bool Face::isInside( vec pos ){
	// Computing the barycentric coordinates
	vec p1 = this->edges[0].v0->position;
	vec p2 = this->edges[1].v0->position;
	vec p3 = this->edges[2].v0->position;

	vec u0 = p2 - p1;
	vec u1 = p3 - p1;
	vec u2 = pos - p1;

	double d11 = dot( u1, u1 );
	double d00 = dot( u0, u0 );
	double d02 = dot( u0, u2 );
	double d01 = dot( u0, u1 );
	double d12 = dot( u1, u2 );


	double dT = d00*d11 - d01*d01;
	double lambda2 = (d11*d02 - d01*d12)/dT;
	double lambda3 = (d00*d12 - d01*d02)/dT;
	double lambda1 = 1 - lambda2 - lambda3;

	return lambda1 >= 0 && lambda2 >= 0 && lambda3 >= 0;
}
