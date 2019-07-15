/*!
 * A template RD system
 */

#include "../include/simulation.h"

using namespace std;
using namespace Util;

bool Debug::debugging = true;
int Debug::type = Debug::STD_OUT;

ofstream Debug::lout("animat_log.txt", std::ofstream::ate );

int getNumFlag( int argc, char** argv, char *flag, int defv ){
    for( int i = 0; i < argc; i++ ){
        string s = string(argv[i]);
        string sflag = string(flag);
        int f = s.find(sflag);

        if( f != string::npos )
            return stoi(s.substr(f + sflag.length(), s.length()));
    }

    return defv;
}

int main (int argc, char **argv)
{
    if (argc < 2) {
        cerr << "\nUsage: ./animado experimentdir \n\n";
        return -1;
    }

    int maxIterations = getNumFlag( argc, argv, "-mi", -1 );
    Simulation sim( 1000, 600 );
    sim.maxIterations = maxIterations;
    sim.init( argv[1] );

    while (!sim.finished) {
        sim.step();
        sim.recordFrame();         
    }

    return 0;
};
