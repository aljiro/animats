#include "../include/enviroment.h"

using namespace Util;

AnimatSet::AnimatSet( Animat *a, int rate, int max_num ){
    this->original = a;
    this->rate = rate;
    this->max_num = max_num;
    this->count = 1;
}

void AnimatSet::notify( Enviroment *e, int step ){
    if( (step % rate) == 0 && count < max_num ){
        Animat *a = this->original->clone();
        e->agents.push_back( a );
        e->cm.registerObject( a->body );
        a->body->cm = &e->cm;
        count++;
    }
}

/* 
    Adds barriers like floor and walls
*/
Plane* Enviroment::addPlane( ){
    Plane *p = new Plane(new PlaneMeshProvider(), true);
    this->planes.push_back( p );
    return p;    
}

void Enviroment::registerObjectsForCollision(){
    Debug::log("Registering planes");
    for( Plane *p : this->planes )
    {        
        cm.registerObject(p);
        p->cm = &cm; 
    }
    
    Debug::log("Registering animats");
    for( Animat *a: this->agents ){        
        this->cm.registerObject( a->body ); // Registering for collisions
        a->body->cm = &cm;
    }
}

/* 
    Adds an agent to the simulation
*/
Animat* Enviroment::addAgent( int id ){
    Debug::log(string("Initializing body"));
    Body *body = new Body( new ObjMeshProvider("./mesh.obj"), 0.5, 1 );
    Debug::log(string("Initializing brain"));
    Brain *brain = new DefaultBrain();
    Animat *a = new Animat( id, body, brain );
    this->agents.push_back( a );    
    return a;
}

Animat* Enviroment::addAgentSet( int id, int rate, int max_num ){
    Animat *a = this->addAgent( id );
    AnimatSet *as = new AnimatSet( a, rate, max_num );
    this->registerListener( as );
    return a;
}

void Enviroment::registerListener( StepListener* sl ){
    this->listeners.push_back( sl );
}

/*
    Returns the number of agents 
*/
int Enviroment::numAgents(){
    return this->agents.size();
}

void Enviroment::processEvents(){
    // Any event processing if any
}

/* 
    performEnviromentActions:
    Performs the actions of external forces like gravity. It modifies the goal positions of the agents.
*/ 
void Enviroment::performEnviromentActions(){
    for( int i = 0; i < this->numAgents(); i++ ){
        Debug::log("Adding external forces for animat.");
        this->agents[i]->body->addExternalForce( 0.0, 0.0, -1.0 );
    }
}

/* 
    perfomAgentActions:
    Drives the agent's behavior in the enviroment. Mainly moving. 
    
*/
void Enviroment::performAgentActions(){
    for( int i = 0; i < this->numAgents(); i++ ){
        Debug::log("Performing actions of animat.");
        this->agents[i]->act();
    }

}

void Enviroment::handleCollisions( int step ){
    // This is supposed to be deleted later
    for( Animat *a : this->agents ){
        a->body->fixNormals();
    }

    // Scan the data structures in search for collisions
    this->cm.findCollisions( step );
}

/*
    Integrates the equations of motion for all the agents.
*/
void Enviroment::integrate( int step ){
    for( int i = 0; i < this->numAgents(); i++ ){
        this->agents[i]->updateVelocities( step );
    }

    this->cm.correctVelocities();

    for( int i = 0; i < this->numAgents(); i++ ){
        this->agents[i]->updatePositions( step );
    }
}

void Enviroment::notifyListeners( int step ){
    for( StepListener *sl : listeners )
        sl->notify( this, step );
}

/* 
    Draws al the animats on the screen.
*/
bool Enviroment::draw( SimGraphics& sgraphics ){

    try {
        sgraphics.reset();
        
        for( int k = 0; k < this->planes.size(); k++ ){
            this->planes[k]->draw( sgraphics );
        }

        for( int j = 0; j < this->numAgents(); j++ ){
            Animat *a = this->agents[j];            
            a->draw( sgraphics );
        }

        sgraphics.redraw();
    } catch (const exception& e) {
        cerr << "Caught exception calling enviroment.draw: " << e.what() << endl;
        return true;
    }

    return false; // finished = false
}