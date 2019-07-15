#include "../include/experiments.h"

using namespace tinyxml2;
using namespace std;
using namespace arma;

ExperimentLoader::ExperimentLoader( Enviroment *e ){
	this->enviroment = e;
}

void ExperimentLoader::load( char *dir ){
	
	XMLDocument doc;
	chdir(dir); // Changes working directory
	string experimentFile = "./experiment.xml";
	Debug::log(string("Loading experiment from ") + experimentFile);
	doc.LoadFile( experimentFile.c_str() );
	XMLPrinter printer;
	
	XMLNode *root = doc.FirstChild();

	if( root != NULL &&  strcmp(root->ToElement()->Name(), "experiment") != 0 ){
		// Error
		Debug::log("Incorrect format.");
	}

	XMLNode *it = root->FirstChild();

	while( it != NULL ){
		const char* element = it->ToElement()->Name();

		if( strcmp( element, "plane" ) == 0 ){
			Debug::log(string("Adding a plane"));
			this->addPlane( it );
		}
		else if( strcmp( element, "animat" ) == 0 ){
			Debug::log(string("Adding an animat"));
			this->addAnimat( it );
		}
		else // error
			Debug::log("Unrecognized node in the experiment file.");

		it = it->NextSibling();
	}
	
	
}

void ExperimentLoader::addPlane( XMLNode* node ){
	// Initialize standard position plane
	Plane *p = this->enviroment->addPlane();	
	
	XMLNode *it = node->FirstChild();

	while( it != NULL ){
		XMLElement* element = it->ToElement();
		const char* elementName = element->Name();

		if( strcmp( elementName, "scale") == 0 ){
			vec sc = ep.parseVector( element->GetText() );
			ScaleTransform sct( sc(0), sc(1), 0 );
			p->mapTransform( sct );
		}else if( strcmp( elementName, "position" ) == 0 ){
			vec pos = ep.parseVector( element->GetText() );
			TranslateTransform tt( pos(0), pos(1), pos(2) );
			p->mapTransform( tt );
		}else if( strcmp( elementName, "rotation" ) == 0 ){
			vec angles = ep.parseVector( element->GetText() );
			RotateTransform rt( angles(0), angles(1), angles(2) );
			p->mapTransform( rt );
		}else if( strcmp( elementName, "collitionable" ) == 0 ){
			bool c = ep.parseBool( element->GetText() );
			//p.setCollitionable( c );
		}else
			Debug::log("Unrecognized plane options");// exception

		it = it->NextSibling();
	}

	
}

void ExperimentLoader::addAnimat( XMLNode *node ){
	
	Debug::log(string("Obtaining first child"));
	XMLNode *it = node->FirstChild();	
	Debug::log(string("Obtaining attribute"));	
	int id = node->ToElement()->IntAttribute("id");
	int rate = node->ToElement()->IntAttribute("rate");
	int max_num = node->ToElement()->IntAttribute("max");

	Debug::log(string("Adding animat to the enviroment"));
	Animat *a = this->enviroment->addAgentSet( id, rate, max_num );
	Debug::log(string("Starting iteration"));

	while( it != NULL ){
		XMLElement* element = it->ToElement();		
		const char* elementName = element->Name();

		if( strcmp( elementName, "scale") == 0 ){
			vec sc = ep.parseVector( element->GetText() );
			Debug::log(string("Scaling: ")+printvec(sc));
			ScaleTransform sct( sc(0), sc(1), sc(2) );
			a->body->mapTransform( sct );
		}else if( strcmp( elementName, "position" ) == 0 ){
			vec pos = ep.parseVector( element->GetText() );
			Debug::log(string("Translating: ")+printvec(pos));
			TranslateTransform tt( pos(0), pos(1), pos(2) );
			a->body->mapTransform( tt );
		}else if( strcmp( elementName, "rotate" ) == 0 ){
			vec angles = ep.parseVector( element->GetText() );
			Debug::log(string("Rotating: ")+printvec(angles));
			RotateTransform rt( angles(0), angles(1), angles(2) );
			a->body->mapTransform( rt );
		}else if( strcmp( elementName, "softness" ) == 0 ){
			double s = ep.parseDouble( element->GetText() );
			Debug::log(string("Softess: "));
			cout << s << endl;
			a->body->setSoftness( s );
		}else
			Debug::log("Unrecognized plane options");// exception

		it = it->NextSibling();
	}

}

// ExperimentParser
vec ExperimentParser::parseVector( const char *element ){
	string item(element);
	regex re("^\\(([0-9]+.[0-9]*),([0-9]+.[0-9]*),([0-9]+.[0-9]*)\\)");
	std::smatch match;
	std::regex_search(item, match, re);
	vec v = {0, 0, 0};

	if( match.size() > 1 )
	v = {stod(match.str(1)), stod(match.str(2)), stod(match.str(3))};

	return v;
}
	
bool ExperimentParser::parseBool( const char *element ){
	bool v = strcmp( element, "true" ) == 0;
	return v;
}

int ExperimentParser::parseInt( const char *element ){
	int v = stoi( element );
	return v;
}

double ExperimentParser::parseDouble( const char *element ){
	double v = stod( element );
	return v;
}
