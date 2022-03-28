
// Energia.h defines a "convienence" macro for min/max
// this clashes with C++'s definitions for std::min/std::max... so any time we include <algorithm>,
// either directly or transitively, we get a big compile error
// this header preemptively erases the min/max macro before including <vector> (which includes
// <algorithm> transitively), to prevent everything blowing up.

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

#include <vector>
