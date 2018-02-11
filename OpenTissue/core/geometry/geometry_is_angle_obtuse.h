#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_ANGLE_OBTUSE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_ANGLE_OBTUSE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace geometry
  {

    /** 
    * Obtuse Angle Testing
    * This method tests if the angle between the two vectors (pi-p) and (pj-p) is greather than \pi/2 radians.
    *
    * @param p      The common tail point of the two vectors
    * @param pi     The head point of one vector.
    * @param pj     The head point of the other vector.
    *
    * @return       If angle is obtuse then the return value is true otherwise it is false.
    */
    template<typename vector_type>
    bool is_angle_obtuse(vector_type const & p,vector_type const & pi,vector_type const & pj)
    {
      vector_type u = pi-p;
      vector_type v = pj-p;
      return ((u*v) < 0);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_IS_ANGLE_OBTUSE_H
#endif
