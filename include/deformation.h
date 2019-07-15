#ifndef DEFORMATION_H
#define DEFORMATION_H

#include <vector>
#include <iostream>
#include <armadillo>
#include "cluster.h"
using namespace arma;
using namespace std;
using namespace Util;
using namespace morph;

class MatchTransform{
public:
    virtual vec transform( vec x  ) = 0;
    virtual void precompute( vector<PointMass *>& points, Cluster& cluster ) = 0;
    virtual void computeT( vector<PointMass *>& points, Cluster& cluster, double beta  ) = 0;
};

class LinearMatchTransform : public MatchTransform{
private:
    mat T;
    /*
        Computers the matrix Apq
    */
    mat getApq( vector<PointMass *>& points, Cluster& cluster );
public: 

    /*
        Precomputes the static quatities and stores it in the cluster
    */
    void precompute( vector<PointMass *>& points, Cluster& cluster );

    /*
        Computes the transformation matrix
    */
    void computeT( vector<PointMass *>& points, Cluster& cluster, double beta  );

    /* 
        Applies the computed transformations to a given point
    */
    vec transform( vec x0 );

};

class QuadraticMatchTransform : public MatchTransform{
private:

    mat T;
    vec quadraticVec( vec q );
    mat getLinearApq( vector<PointMass*>& points, Cluster& cluster );
    mat getApq( vector<PointMass*>& points, Cluster& cluster );

public:
    vec transform( vec x );
    void precompute( vector<PointMass*>& points, Cluster& cluster );
    void computeT( vector<PointMass*>& points, Cluster& cluster, double beta  );

};

class DeformationKernel{

private: 
    string transformType;

    void setTransform( string transformType );
    /*
     Function with the logic to generate clusters.
     This can be changed to fit the application.
    */
    void generateClusters();
    /*
        Precomputes the static quantities for each of the clusters
    */
    void precompute();

    /*
        Computes the goal positions for each of the points in the clusters
    */
    void computeGoals();

public:
    double h;
    double alpha;               // rigidity (from 0  to 1, where 1 is rigid)
    double beta;  

    vector<PointMass*> *points;
    vector<Cluster> clusters;
    MatchTransform *matchTransform;

    DeformationKernel( double alpha, double beta, double h, string transformType );
    ~DeformationKernel();

    /*
        Precomputes the required quantities.
    */
    void init(  vector<PointMass*>* points );
    /*
        Returns the contributions of the force to the overall equations of motion
    */
    vector<vec> getVelocityComponents( vector<PointMass *>& points );

};

#endif