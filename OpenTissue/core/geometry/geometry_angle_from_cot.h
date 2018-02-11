#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ANGLE_FROM_COT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ANGLE_FROM_COT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>   // Needed for std::sqrt and std::fabs
#include <cassert> // Needed for assert

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Retrives the ``angle'' from the cotangent of two vectors, (pi-p) and (pj-p).
    * This implementation is based on the paper:
    *
    *    Meyer, M., Desbrun, M., Schröder, P., AND Barr, A. H. Discrete Differential Geometry Operators for Triangulated 2-Manifolds, 2002. VisMath.
    *
    * @param p      The common tail point of the two vectors
    * @param pi     The head point of one vector.
    * @param pj     The head point of the other vector.
    *
    * @return       The angle value of the cotangent of the angle between the two specified vectors.
    */
    template<typename vector_type>
    typename vector_type::value_type angle_from_cot(vector_type const & p,vector_type const & pi,vector_type const & pj)  
    {
      using std::sqrt;
      using std::fabs;
      using std::atan2;

      typedef typename vector_type::value_type   real_type;

      vector_type u = pi-p;
      vector_type v = pj-p;
      real_type dot = u*v;
      real_type denom = sqrt( (u*u)*(v*v) - dot*dot );

      return fabs(atan2(denom,dot));
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ANGLE_FROM_COT_H
#endif
