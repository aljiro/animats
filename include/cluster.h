#ifndef CLUSTER_H
#define CLUSTER_H

#include "util.h"
#include "primitives.h"
#include <vector>
#include <iostream>
#include <armadillo>

using namespace arma;
using namespace std;
using namespace morph;

class Cluster{
private:
    

public:
    vector<int> members;
    vec cm;
    vec originalCm;
    bool recompute;
    mat Aqq;

    Cluster();

    void addMember( int ind );
    void printMembers();
    void addAll ( int n );
    void reset();
    int size();
    int get( int i );
    void computeCms( vector<PointMass *> points );
    /* 
     Computes the center of gravity of the object
    */
    vec getCm( vector<PointMass *> points );
    /* 
        Computes the original center of mass
    */
    vec getOriginalCm( vector<PointMass *> points );
};
#endif