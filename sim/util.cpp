#include "../include/util.h"

std::string Util::printvec( arma::vec v ){
	std::stringstream s;
	s << "(" << v(0) << ", "<<v(1)<<", "<<v(2)<<")";
	return s.str();
}

std::string Util::printvec2( arma::vec v ){
	std::stringstream s;
	s << "(" << v(0) << ", "<<v(1)<<")";
	return s.str();
}
