#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_line_triangle.h>

namespace OpenTissue
{
  namespace intersect
  {

    /**
    * Triangle Intersection Testing.
    * This method is based on a all-edge triangle pair testing strategy. It
    * is pretty much brute force all the way.
    *
    * @param A         The first triangle.
    * @param B         The second triangle.
    *
    * @param  points   Upon return any intersection points are added to this container.
    *
    * @return          If intersection then the reutrn value is true otherwise it is false.
    */
    template<typename triangle_type,typename vector3_container>
    bool triangle_triangle(triangle_type const & A,triangle_type const & B,vector3_container & points)
    {
      typedef typename triangle_type::vector3_type   vector3_type;

      bool collision = false;
      vector3_type  p;
      if(line_triangle(A.p0(),A.p1(),B,p))
      {
        points.push_back(p);
        collision = true;
      }
      if(line_triangle(A.p1(),A.p2(),B,p))
      {
        points.push_back(p);
        collision = true;
      }
      if(line_triangle(A.p2(),A.p0(),B,p))
      {
        points.push_back(p);
        collision = true;
      }
      if(line_triangle(B.p0(),B.p1(),A,p))
      {
        points.push_back(p);
        collision = true;
      }
      if(line_triangle(B.p1(),B.p2(),A,p))
      {
        points.push_back(p);
        collision = true;
      }
      if(line_triangle(B.p2(),B.p0(),A,p))
      {
        points.push_back(p);
        collision = true;
      }
      return collision;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_H
#endif
