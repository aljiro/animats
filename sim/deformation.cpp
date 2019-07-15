#include "../include/deformation.h"

using namespace arma;
using namespace std;
using namespace Util;

using arma::vec;
using arma::mat;

/*
    Computers the matrix Apq
*/
mat LinearMatchTransform::getApq( vector<PointMass *>& points, Cluster& cluster ){
    mat Apq = zeros<mat>(3,3);
    PointMass *currentPoint;
    vec q, p, cm, original_cm;
    cm = cluster.getCm( points );
    original_cm = cluster.getOriginalCm( points );

    for( int i = 0; i < cluster.size(); i++ ){
        currentPoint = points[cluster.get(i)];
        q = currentPoint->originalPosition - original_cm;
        p = currentPoint->position - cm;
        Apq += currentPoint->mass*(p*q.t());
    }

    return Apq;
}
 
/*
    Precomputes the static quatities and stores it in the cluster
*/
void LinearMatchTransform::precompute( vector<PointMass *>& points, Cluster& cluster ){
    mat M = zeros<mat>(3,3);
    PointMass *currentPoint;
    vec q;
    Debug::log(string("Precomputing Aqq. "));

    for( int i = 0; i < cluster.size(); i++ ){
        currentPoint = points[cluster.get(i)];
        q = currentPoint->originalPosition - cluster.getOriginalCm( points );
        M += currentPoint->mass*(q*q.t());
    }

    cluster.Aqq = pinv(M);
}    

/*
    Computes the transformation matrix
*/
void LinearMatchTransform::computeT( vector<PointMass *>& points, Cluster& cluster, double beta  ){        
    mat U, V, R, Apq, Aqq, A;
    vec s;
    Apq = this->getApq( points, cluster );    
    //Apq.print(); 
    // cluster.Aqq.print(); 
    // cin.get();
    try{
        svd( U, s, V, Apq );
        R = U*V.t(); // Apq*sqrt(Apq.t()*Apq)
        A = Apq*cluster.Aqq;
        double da = det(A);

        if( da < 0 )
        {
            cout << "ERROR: determinant < 0" << endl;
            cin.get();
        }
        A = A/pow(da, 1.0/3.0 );
        this->T = beta*A + (1 - beta)*R;
    }catch(const std::exception& e){
        cout << "Error: unable to compute the transformation: "<< e.what();
    }
}

/* 
    Applies the computed transformations to a given point
*/
vec LinearMatchTransform::transform( vec x ){
    //this->T.print();
    vec g = this->T*x; 
    return g; 
}


// QuadraticMatchTransform

vec QuadraticMatchTransform::quadraticVec( vec q ){
   vec qc = {q(0), q(1), q(2), q(0)*q(0), q(1)*q(1), q(2)*q(2), q(0)*q(1), q(1)*q(2), q(0)*q(2)};
   return qc;
}

mat QuadraticMatchTransform::getLinearApq( vector<PointMass *>& points, Cluster& cluster ){
    mat Apq = zeros<mat>(3,3);
    PointMass *currentPoint;
    vec q, p, cm, original_cm;
    cm = cluster.getCm( points );
    original_cm = cluster.getOriginalCm( points );

    for( int i = 0; i < cluster.size(); i++ ){
        currentPoint = points[cluster.get(i)];
        q = currentPoint->originalPosition - original_cm;
        p = currentPoint->position - cm;        
        Apq += currentPoint->mass*(p*q.t());
    }

    return Apq;
}

mat QuadraticMatchTransform::getApq( vector<PointMass *>& points, Cluster& cluster ){
    mat Apq = zeros<mat>(3,9);
    PointMass *currentPoint;
    vec q, qc, p, cm, original_cm, pc;
    cm = cluster.getCm( points );
    original_cm = cluster.getOriginalCm( points );

    for( int i = 0; i < cluster.size(); i++ ){
        currentPoint = points[cluster.get(i)];
        q = currentPoint->originalPosition - original_cm;
        p = currentPoint->position - cm;
        qc = quadraticVec(q);
        
        Apq += currentPoint->mass*(p*qc.t());
    }

    return Apq;
}

vec QuadraticMatchTransform::transform( vec x ){
    vec qc = quadraticVec( x );
    vec g = this->T*qc; 
    return g; 
}

void QuadraticMatchTransform::precompute( vector<PointMass *>& points, Cluster& cluster ){
    mat M = zeros<mat>(9,9);
    PointMass *currentPoint;
    vec q, qc;

    for( int i = 0; i < cluster.size(); i++ ){
        currentPoint = points[cluster.get(i)];
        q = currentPoint->originalPosition - cluster.getOriginalCm( points );
        qc = quadraticVec(q);
        M += currentPoint->mass*(qc*qc.t());
    }

    cluster.Aqq = pinv(M);
}

void QuadraticMatchTransform::computeT( vector<PointMass *>& points, Cluster& cluster, double beta  ){
    mat U, V, Rl, R, Apq, Aqq, A, Apq_linear;
    vec s;
    Apq = this->getApq( points, cluster ); 
    Apq_linear = this->getLinearApq( points, cluster );
    // Apq.print(); 
    // cluster.Aqq.print(); 

    try{
        svd( U, s, V, Apq_linear );
        Rl = U*V.t(); // Apq*sqrt(Apq.t()*Apq)
        R = join_rows(join_rows( Rl, zeros(3,3) ), zeros(3,3));
        A = Apq*cluster.Aqq;        
        //double da = det(A);
        //A = A/pow(da, 1.0/3.0 );
        this->T = beta*A + (1 - beta)*R;
    }catch(const std::exception& e){
        cout << "Error: unable to compute the transformation: "<< e.what();
    }
}

// Deformation kernel
DeformationKernel::DeformationKernel( double alpha, double beta, double h, string transformType ){
    this->alpha = alpha;
    this->beta = beta;
    this->h = h;
    this->transformType = transformType;

}

void DeformationKernel::init(  vector<PointMass*>* points ){
    this->points = points;

    this->generateClusters();
    this->setTransform( this->transformType );
    this->precompute(); 
}

void DeformationKernel::setTransform( string transformType ){
    if( transformType == "quadratic"){
        QuadraticMatchTransform *qt = new QuadraticMatchTransform();
        this->matchTransform = qt;            
    }else{
        LinearMatchTransform *lt = new LinearMatchTransform();
        this->matchTransform = lt;
    }
}

vector<vec> DeformationKernel::getVelocityComponents( vector<PointMass *>& points ){
    // Compute goals
    Debug::log(string("Computing goals"));
    this->computeGoals();
    // Return the velocity components
    vector<vec> components;

    for( PointMass* p : points ){
        //Debug::log(string("Creating velocity component"));
        //Debug::log(string("Goal pos: ")+printvec(p->goalPosition));
        //Debug::log(string("Pos: ")+printvec(p->position));
        if( p->iscontact )
            p->goalPosition = p->nposition;
        
        vec vc = this->alpha*(p->goalPosition - p->position)/this->h;
        components.push_back( vc );
    }

    Debug::log(string("Finished all the components"));

    return components;
}

/*
    Precomputes the static quantities for each of the clusters
*/
void DeformationKernel::precompute(){
    for( int k = 0; k < this->clusters.size(); k++ ){
        this->matchTransform->precompute( *this->points, this->clusters[k] );
    }
}

/*
    Computes the goal positions for each of the points in the clusters
*/
void DeformationKernel::computeGoals(){
    Cluster cluster;

    for( int i = 0; i < this->clusters.size(); i++ ){
        cluster = this->clusters[i];
        cout << "Computing the transform"<<endl;
        this->matchTransform->computeT( *this->points, cluster, this->beta );

        for( int j = 0; j < cluster.size(); j++ ){

            PointMass *p = (*this->points)[cluster.get(j)]; 
            
            vec tx = matchTransform->transform( 
                     p->originalPosition - cluster.getOriginalCm( *this->points ) );
            p->goalPosition = tx + cluster.getCm( *this->points ); 
        }
    }
}

/*
 Function with the logic to generate clusters.
 This can be changed to fit the application.
*/
void DeformationKernel::generateClusters(){
    Cluster cluster;
    cluster.addAll( this->points->size());
    this->clusters.push_back( cluster );
    cluster.printMembers();
}

