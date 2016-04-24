/*
 * Distance.h
 *
 *  Created on: Mar 2, 2016
 *      Author: benny
 */

#ifndef BA_STUFF_SRC_DISTANCE_H_
#define BA_STUFF_SRC_DISTANCE_H_

class Distance {
public:
	Distance();
	virtual ~Distance();
};

#endif /* BA_STUFF_SRC_DISTANCE_H_ */

#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
