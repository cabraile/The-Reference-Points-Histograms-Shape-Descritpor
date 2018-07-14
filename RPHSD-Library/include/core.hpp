#ifndef GUARD_CORE_HPP
#define GUARD_CORE_HPP

#include <string>
#include <vector>

namespace fex {

typedef std::vector<double> Histogram;
typedef std::vector<double> Descriptor; 

/*-----------------------------------------------------------------------------
Debugging purposes declarations
-----------------------------------------------------------------------------*/
bool DEBUG_MSG = true;

void SET_DEBUG(const bool & val) { DEBUG_MSG = val; }

void debug(const std::string & msg, const bool & linebreak=true) {
	if(DEBUG_MSG) {
		std::cout << msg;
		if(linebreak) {
			std::cout << std::endl;
		}
	}
	return ;
}

template<typename T>
void debug(const std::vector<T> vec) {
	if(DEBUG_MSG) {
		std::cout << vec[0];
		for(std::size_t id = 1; id < vec.size(); id++) {
			std::cout << "," << vec[id];
		}
		std::cout << std::endl;
	}
	return ;
}


}

#endif
