#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_SPHERE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_SPHERE_H
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
  namespace intersect
  {

    /**
    * Point Containment Test.
    *
    * @param p                A point
    * @param center           The center of the sphere
    * @param squared_radius
    *
    * @return     If the point p is contained in the sphere the return value is true otherwise it is false.
    */
    template<typename vector3_type,typename real_type>
    bool point_sphere(vector3_type const & p,vector3_type const & center, real_type const & squared_radius)
    {
      //using std::fabs;
      //vector3_type d = p-m_c;
      //return fabs(d*d) <= m_r2; 

      real_type test = sqr_length(p - center);
      real_type squared_distance = test - squared_radius;
      return squared_distance <= 0.0;
    }

    template<typename vector3_type,typename sphere_type>
    bool point_sphere(vector3_type const & p,sphere_type const & sphere)
    {
      return point_sphere(p,sphere.center(),sphere.squared_radius());
    }


  } // namespace intersect

} // namespace OpenTissue

//OPENTISSUE_COLLISION_INTERSECT_INTERSECT_POINT_SPHERE_H
#endif
