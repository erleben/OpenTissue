#ifndef OPENTISSUE_UTILITY_UTILITY_QHULL_H
#define OPENTISSUE_UTILITY_UTILITY_QHULL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

  //////////////////////////////////////////////////////////////////
  //
  // This is a bit tricky, I cant just include qhull_a.h as
  // QHull advices, because MVC complains about math.h which
  // is included "indirectly by qhull_a.h
  //
  // Hopefully furture releases of QHull will allow me to
  // just write:
  //
  //  extern "C"
  //  {
  //    #include <Qhull/qhull_a.h>
  //  }
  //
#if defined(__cplusplus)
  extern "C"
  {
#endif
#include <stdio.h>
#include <stdlib.h>
#include <libqhull/libqhull.h>
#include <libqhull/mem.h>
#include <libqhull/qset.h>
#include <libqhull/geom.h>
#include <libqhull/merge.h>
#include <libqhull/poly.h>
#include <libqhull/io.h>
#include <libqhull/stat.h>
#if defined(__cplusplus)
  }
#endif

//OPENTISSUE_UTILITY_UTILITY_QHULL_H
#endif
