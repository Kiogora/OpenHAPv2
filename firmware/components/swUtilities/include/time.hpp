#ifndef  TIME_HPP
#define  TIME_HPP
#include <ctime>

namespace systemUtils
{
void obtainSystemTimeFromSntp(std::tm& timeinfo); 
bool isSystemTimeInvalid();
}

#endif