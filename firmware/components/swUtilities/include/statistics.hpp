#ifndef  STATS_HPP
#define  STATS_HPP

#include <cstddef>

namespace softwareUtilities
{
    namespace stats
    {
        template <typename T>
        T findMax(T x[], size_t size)
        {
            T max = x[0];
            for (size_t i = 1; i < size; ++i)
            {
                if(x[i] > max){max = x[i];};
            }
            return max;
        }
    }
}
#endif