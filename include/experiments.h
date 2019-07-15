#ifndef EXPERIMENTS_H
#define EXPERIMENTS_H

#include "util.h"
#include "enviroment.h"
#include "body.h"
#include "brain.h"

#include <vector>
#include <iostream>
#include <armadillo>
#include <string>
#include <regex>

#include "tinyxml2.h"

using namespace arma;
using namespace std;
using namespace tinyxml2;
using namespace morph;

class ExperimentParser{
public:
	vec parseVector( const char * );
	bool parseBool( const char * );
	int parseInt( const char * );
	double parseDouble( const char * );
};

class ExperimentLoader{
private:
	Enviroment *enviroment;	
	ExperimentParser ep;
public:
	ExperimentLoader( Enviroment *e );
	void load( char *dir );
	void addPlane( XMLNode * );
	void addAnimat( XMLNode * );
};


#endif