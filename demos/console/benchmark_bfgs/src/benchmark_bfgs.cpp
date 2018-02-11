//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include "benchmark.h"

int main( int argc, char **argv )
{
  benchmark( "N010_T100_", "n010_t100.m", "stats.tex",  10, 100);
  benchmark( "N020_T100_", "n020_t100.m", "stats.tex",  20, 100);
  benchmark( "N040_T100_", "n040_t100.m", "stats.tex",  40, 100);
  benchmark( "N080_T100_", "n080_t100.m", "stats.tex",  80, 100);
  benchmark( "N160_T100_", "n160_t100.m", "stats.tex", 160, 100);
  //benchmark( "N320_T100_", "n320_t100.m", "stats.tex", 320, 100);
  return 0;
}
