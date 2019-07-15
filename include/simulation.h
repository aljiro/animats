#ifndef SIMULATION_H
#define SIMULATION_H

#include "util.h"
#include "enviroment.h"
#include "experiments.h"

#include <iostream>
#include <vector>
#include <string>

#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

using namespace arma;
using namespace std;
using namespace morph;

class Simulation{
private:
	SimGraphics sgraphics;
	Enviroment enviroment;
	ExperimentLoader *experiment;
	int width, height;
	
	void setupCapturesDir();

public:
	bool finished;
	int stepNum;
	int maxIterations;

	Simulation( int width, int height );
	void init(char *name );
	int step();
	void recordFrame();
};
#endif