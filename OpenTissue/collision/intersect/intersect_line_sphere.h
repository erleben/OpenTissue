#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_SPHERE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_SPHERE_H
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
    * Line Segement  Sphere Intersection Test.
    *
    * @param b        The starting point of the line segment.
    * @param e        The ending point of the line segment.
    * @param sphere   The sphere
    *
    * @return         If an intersection is found then the return value is true otherwise it is false.
    */
    template<typename vector3_type, typename sphere_type>
    bool line_sphere(vector3_type const & b, vector3_type const & e, sphere_type const & sphere)
    {
      typedef typename vector3_type::value_type   real_type;

      vector3_type c  = sphere.center();
      real_type    r2 = sphere.squared_radius();

      vector3_type eb = e-b;
      vector3_type cb = c-b;
      real_type t = (cb*eb) / (eb*eb);
      if(t<0 || t>1.0)
        return false;
      //---    
      //---   p -c = b + (e-b)*t  - c = (b-c) + (e-b)*t =  (e-b)*t - (c-b)
      //---
      vector3_type pc = eb*t - cb;
      if (  (pc*pc) <= r2 )
        return true;
      return false;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_LINE_SPHERE_H
#endif
