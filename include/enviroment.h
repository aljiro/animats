#ifndef ENVIROMENT_H
#define ENVIROMENT_H

#include "util.h"
#include "animats.h"
#include "brain.h"
#include "collisions.h"
#include "simgraphics.h"

#include <vector>
#include <iostream>
#include <armadillo>

using namespace arma;
using namespace std;
using namespace morph;

class StepListener;

class Enviroment{

public:
    CollisionManager cm;
    vector <Animat *> agents;
    vector <Plane *> planes;
    vector <StepListener *> listeners;
    /* 
        Adds barriers like floor and walls
    */
    Plane* addPlane();
    /* 
        Adds an agent to the simulation
    */
    Animat* addAgent( int id );
    Animat* addAgentSet( int id, int rate, int max_num );

    void registerListener( StepListener* sl );
    void notifyListeners( int step );

    void registerObjectsForCollision();
    /*
        Returns the number of agents 
    */
    int numAgents();
    void processEvents();
    /* 
        performEnviromentActions:
        Performs the actions of external forces like gravity. It modifies the goal positions of the agents.
    */ 
    void performEnviromentActions();
    /* 
        perfomAgentActions:
        Drives the agent's behavior in the enviroment. Mainly moving. 
        
    */
    void performAgentActions();
   
    void handleCollisions( int step );
    /*
        Integrates the equations of motion for all the agents.
    */
    void integrate( int step );
    /* 
        Draws al the animats on the screen.
    */
    bool draw( SimGraphics& sgraphics );

};

class StepListener{
public:
    virtual void notify( Enviroment *e, int step ) = 0;
};

class AnimatSet: public StepListener{
private:
    Animat *original;
    int rate;
    int max_num;
    int count;
public:
    AnimatSet( Animat *a, int rate, int max_num );
    void notify( Enviroment *e, int step );
};
#endif