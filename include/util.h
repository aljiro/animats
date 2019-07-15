#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <armadillo>

namespace Util{

	class Debug{
	public:
		static const int STD_OUT = 0;
		static const int FILE = 1;
		static bool debugging;
		static int type;
		static std::ofstream lout;

		static void log( std::string msg ){
			if( debugging ){
				if( type == STD_OUT ){
					std::cout << ">>" << msg << std::endl;
				}
				else if( type == FILE ){
					lout << ">>" << msg << std::endl;
				}
			}
		}

	};

	std::string printvec( arma::vec v );
	std::string printvec2( arma::vec v );
}
#endif


