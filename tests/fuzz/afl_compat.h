#pragma once

inline int balancer_afl_loop_fallback(int) {
  static bool first_iteration = true;
  if (!first_iteration) {
    return 0;
  }
  first_iteration = false;
  return 1;
}

#ifndef __AFL_LOOP
#define __AFL_LOOP(x) balancer_afl_loop_fallback(x)
#endif

