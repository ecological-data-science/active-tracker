
#pragma once

#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x) printf x
#else
  #define DEBUG_PRINT(x)
#endif
