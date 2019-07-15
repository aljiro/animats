#include "../include/cluster.h"

using namespace arma;
using namespace std;
using namespace Util;

//Cluster
Cluster::Cluster(){
    recompute = true;
}

void Cluster::addMember( int ind ){
    bool isin = false;

    for( int i = 0; i < this->members.size(); i++ )
        if( this->members[i] == ind )
            isin = true;

    if( !isin ){ 
        this->members.push_back( ind );
    }
}

void Cluster::printMembers(){
    cout << "Cluster members: ";

    for( int i = 0; i < this->members.size(); i ++ ){
        cout << this->members[i] << ", ";
    }

    cout << endl;
}

void Cluster::addAll ( int n ){
    for( int i = 0; i < n; i++ ){
        this->addMember(i);
    } 
}

void Cluster::reset(){
    this->members.clear();
}

int Cluster::size(){        
    return this->members.size();
}

int Cluster::get( int i ){
    return members[i];
}

void Cluster::computeCms( vector<PointMass *> points ){
    vec cg = zeros<vec>(3);
    vec ocg = zeros<vec>(3);
    double M = 0;
    PointMass *p;

    for(int i=0; i < this->members.size(); i++){
        p = points[this->get(i)];
        //Debug::log(string("Point position: ") + printvec(p.originalPosition));
        cg += p->mass*p->position;
        ocg += p->mass*p->originalPosition;
        M += p->mass;
    }

    cg /= M;
    ocg /= M;

    this->cm = cg;
    this->originalCm = ocg;
}

/* 
 Computes the center of gravity of the object
*/
vec Cluster::getCm( vector<PointMass *> points ){
    //if( this->recompute ) 
    this->computeCms( points );      

    //this->recompute = false;
    return this->cm;
}

/* 
    Computes the original center of mass
*/
vec Cluster::getOriginalCm( vector<PointMass *> points ){
    if( this->recompute ) 
        this->computeCms( points );      

    return this->originalCm;
}