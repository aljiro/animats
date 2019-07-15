#include "../include/body.h"

using namespace arma;
using namespace std;
using namespace Util;

int Body::numPoints(){
    return this->points.size();
}


Body::Body( MeshProvider* mp, double alpha, double density )
{
    double beta = 0.5;
    this->fext = zeros<vec>(3);
    this->h = 0.01;
    this->dk = new DeformationKernel( alpha, beta, h, "linear" );

    mp->populate(this);
    
    init( density );
}

void Body::init( double density ){

    vec v0 = zeros<vec>(3);       
    double mass = 1.0/density;

    for( int i = 0; i < this->points.size(); i++ ){
        PointMass *p = points[i];
        p->originalPosition = p->position;
        p->velocity = v0;
        p->mass = mass;
    }

    dk->init( &this->points );
}

vec Body::getCm(){
    vec cg = zeros<vec>(3);
    double M = 0;
    PointMass *p;

    for(int i=0; i < this->points.size(); i++){
        p = points[i];
        //Debug::log(string("Point position: ") + printvec(p.originalPosition));
        cg += p->mass*p->position;
        M += p->mass;
    }

    cg /= M;
    return cg;
}

void Body::fixNormals(){
    for( Face *f : faces ){
        f->fixNormalOrientation( this->points );
    }
}

vec Body::getOriginalCm(){
    vec cg = zeros<vec>(3);
    double M = 0;
    PointMass *p;

    for(int i=0; i < this->points.size(); i++){
        p = points[i];
        //Debug::log(string("Point position: ") + printvec(p.originalPosition));
        cg += p->mass*p->originalPosition;
        M += p->mass;
    }

    cg /= M;
    return cg;
}

/*
    Update step, performs the integration of the equations of motion
*/
void Body::updateVelocities( int step ){   
       
    vector<vec> vd = dk->getVelocityComponents( this->points ); 

    for( int i = 0; i < this->points.size(); i++ ){
        PointMass *p = this->points[i];
        PointMass *pre = p->pre;
        p->pre = new PointMass();

        if( !p->iscontact ){
            p->pre->velocity = p->velocity;
            p->pre->position = p->position;
        }else{
            p->pre->velocity = pre->velocity;
            p->pre->position = pre->position;
        }

        p->updateVelocity( h*fext/p->mass, vd[i], h) 
    }

    this->fext = zeros<vec>(3);
}

void Body::updatePositions( int step ){

    for( int i = 0; i < this->points.size(); i++ ){
        PointMass *p = this->points[i];
        p->updatePosition( h );    
    }

}



void Body::compensateMomentum( ){
    vec p_past = zeros<vec>(3);
    vec p = zeros<vec>(3);

    for( int i = 0; i < this->points.size(); i++ ){
        PointMass *q  = this->points[i];
        if( q->pre==NULL )
            return;
        p_past += q->pre->velocity*q->mass;
        p += q->velocity*q->mass;
    } 

    vec d = p - p_past;
    int n = this->points.size();

    for( int i = 0; i < this->points.size(); i++  ){
        PointMass *q =  this->points[i];
        //if( !q->iscontact )
            q->vc -= d/(n*q->mass);
    }

}

/*
    Adds external forces to be used in the simulation
*/
void Body::addExternalForce( double fx, double fy, double fz ){
    vec f = {{fx},{fy},{fz}};    
    this->fext += f; 
}

double Body::getMass(){
    return 1;
}

/* 
    Returns the bounds of the point cloud 
*/
vector<double> Body::bounds(){

    double xmin = points[0]->position[0];
    double xmax = points[0]->position[0];
    double ymin = points[0]->position[1];
    double ymax = points[0]->position[1];
    double zmin = points[0]->position[2];
    double zmax = points[0]->position[2];
    double x, y, z;

    for( int i = 1; i < this->numPoints(); i++ ){
        x = points[i]->position[0];
        y = points[i]->position[1];
        z = points[i]->position[2];

        if( x < xmin ) xmin = x;
        if( y < ymin ) ymin = y;
        if( z < zmin ) zmin = z;
        if( x > xmax ) xmax = x;
        if( y > ymax ) ymax = y;
        if( z > zmax ) zmax = z;
    } 

    vector<double> res(6);
    res[0] = xmin;
    res[1] = xmax;
    res[2] = ymin;
    res[3] = ymax;
    res[4] = zmin;
    res[5] = zmax;

    return res;
}

void Body::setSoftness( double alpha ){
    this->dk->alpha = alpha;
}

void Body::draw( SimGraphics& sgraphics ){
    std::array<float, 3> c1 = {0.5, 0.7, 0.8};   
    std::array<float, 3> c2 = {0.8, 0.2, 0.2};       

    Color color;
    color.normal = c1;
    color.highlighted = c2;

    sgraphics.drawObject( this, color, true );
}

string Body::toString(){
    return "body";
}

