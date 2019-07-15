#include "../include/simulation.h"

using namespace Util;
using namespace arma;
using namespace std;

/* Implementation of the simulation interface */

void Simulation::setupCapturesDir(){
	// Creating the directory structure
    mkdir("captures", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    DIR *dir;
    struct dirent *ent;
    if( (dir = opendir("captures")) != NULL ){
    	Debug::log("Directory captures opened. Deleting previous captures.");
    	while((ent = readdir(dir)) != NULL )
    		if( strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0 )
    			remove((string("captures/") + string(ent->d_name)).c_str());
    	closedir(dir);

    }else{
    	Debug::log("There was an error opening the captures dir");
    }
}


Simulation::Simulation( int width, int height ){
	this->finished = false;
	this->stepNum = 0;
	this->maxIterations = -1;
	this->width = width;
	this->height = height;
}

void Simulation::init(char *dir ){
	// Create some displays    
    string worldName( dir );
    string winTitle = worldName + ": animat arena";
    sgraphics.init( winTitle, width, height );

    // Loading the experiment details
    this->experiment = new ExperimentLoader( &this->enviroment );
    this->experiment->load( dir );
    setupCapturesDir();
    this->enviroment.registerObjectsForCollision(); 
}

int Simulation::step(){
	// Step the model
    try {       
        Debug::log(string("Performing simulation step ") + to_string(this->stepNum));
        enviroment.processEvents();
        Debug::log("Enviroment actions");
        enviroment.performEnviromentActions();
        Debug::log("Performing simulation step: Agent actions");
        enviroment.performAgentActions();
        
        Debug::log("Performing simulation step: Handling collisions");
        enviroment.handleCollisions( this->stepNum ); 
        Debug::log("Performing simulation step: Integrating");
        enviroment.integrate( this->stepNum );
 
        Debug::log("Performing simulation step: Drawing");          
        finished = enviroment.draw( sgraphics );  
    } catch (const exception& e) {
        cerr << "Caught exception calling step(): " << e.what() << endl;
        finished = true;
    }

    if( this->maxIterations != -1 && this->stepNum > this->maxIterations  ) 
    	this->finished = true;

    this->stepNum++;

    enviroment.notifyListeners( this->stepNum );
    return this->stepNum;
}

void Simulation::recordFrame(  ){
	sgraphics.takeSnapshot( this->stepNum );
}
