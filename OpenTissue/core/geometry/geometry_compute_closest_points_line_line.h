#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_CLOSEST_POINTS_LINE_LINE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_CLOSEST_POINTS_LINE_LINE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute Closest Points between Lines
    *
    *
    * @param pA   Point on first line.
    * @param uA   Direction of first line (unit vector).
    * @param pB   Point on second line.
    * @param uB   Direction of second line (unit vector).
    * @param s    The line parameter of the closest point on the first line pA + uA*s
    * @param t    The line parameter of the closest point on the second line pB + uB*t
    *
    */
    template<typename vector3_type,typename real_type>
    void compute_closest_points_line_line(
      vector3_type const & pA
      , vector3_type const & uA
      , vector3_type const & pB
      , vector3_type const & uB
      , real_type & s
      , real_type & t
      )
    {
      using std::fabs;

      vector3_type r = pB - pA;
      real_type k = uA*uB;
      real_type q1 = uA*r;
      real_type q2 = -uB*r;
      real_type w = 1 - k*k;
      s = 0;  t = 0;
      real_type epsilon = math::working_precision<real_type>();
      if(fabs(w) > epsilon)
      {
        s = (q1 + k*q2)/w;
        t = (q2 + k*q1)/w;
      }
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_CLOSEST_POINTS_LINE_LINE_H
#endif
