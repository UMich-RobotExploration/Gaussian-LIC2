#pragma once

#include <algorithm>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace cocolic
{
  // threads for the per-point loops, not one per core: Gaussian-LIC keeps several cores busy and a parallel loop waits
  // for every thread of its team (a depth map scan took 5 ms with 32 threads on 32 cores, 0.3 ms with 8)
  inline int NumPointThreads()
  {
#ifdef _OPENMP
    return std::min(8, std::max(1, omp_get_num_procs() / 2));
#else
    return 1;
#endif
  }
} // namespace cocolic
