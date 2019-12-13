#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_SAT_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_SAT_H
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

    template<typename vector3_type>
    bool separation_triangle_triangle(
      vector3_type const & a0
      , vector3_type const & a1
      , vector3_type const & a2
      , vector3_type const & b0
      , vector3_type const & b1
      , vector3_type const & b2        
      );

    template<typename vector3_type>
    bool triangle_triangle_sat(
      vector3_type const & a0
      , vector3_type const & a1
      , vector3_type const & a2
      , vector3_type const & b0
      , vector3_type const & b1
      , vector3_type const & b2        
      )
    {
      return !separation_triangle_triangle(a0,a1,a2,b0,b1,b2);
    }


    /**
    * Triangle Separation Axis Test (SAT).
    * This methods assumed that all coordinates are given in the same coordinate frame.
    *
    * @param a0  First coordinate of triangle A.
    * @param a1  Second coordinate of triangle A.
    * @param a2  Third coordinate of triangle A.
    * @param b0  First coordinate of triangle B.
    * @param b1  Second coordinate of triangle B.
    * @param b2  Third coordinate of triangle B.
    *
    * @return        if the return value is true an separation axis was found otherwise
    *                the two triangles are not separated.
    */
    template<typename vector3_type>
    bool separation_triangle_triangle(
      vector3_type const & a0
      , vector3_type const & a1
      , vector3_type const & a2
      , vector3_type const & b0
      , vector3_type const & b1
      , vector3_type const & b2        
      )
    {
      typedef typename vector3_type::value_type        real_type;

      vector3_type a10 = a1 - a0;
      vector3_type a20 = a2 - a0;
      vector3_type na  = a10 % a20;
      //--- SAT face plane of A
      {
        real_type w = na*a0;            
        real_type d0 = na*b0 - w;
        real_type d1 = na*b1 - w;
        real_type d2 = na*b2 - w;
        if(d0 > 0 && d1 > 0 && d2 > 0)
          return true;
        if(d0 < 0 && d1 < 0 && d2 < 0)
          return true;
      }
      vector3_type b10 = b1 - b0;
      vector3_type b20 = b2 - b0;
      vector3_type nb  = b10 % b20;
      //--- SAT face plane of B
      {
        real_type w = nb*b0;            
        real_type d0 = nb*a0 - w;
        real_type d1 = nb*a1 - w;
        real_type d2 = nb*a2 - w;
        if(d0 > 0 && d1 > 0 && d2 > 0)
          return true;
        if(d0 < 0 && d1 < 0 && d2 < 0)
          return true;
      }
      vector3_type a21 = a2 - a1;
      vector3_type b21 = b2 - b1;
      //--- SAT: a10 crossed with all edges of B
      {
        vector3_type n  = a10 % b10;
        real_type w = n*a0;
        real_type d = n*a2 - w;
        real_type d0 = n*b0 - w;
        real_type d2 = n*b2 - w;
        if(d > 0 && d0 < 0  && d2 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d2 > 0 )
          return true;
      }
      {
        vector3_type n  = a10 % b20;
        real_type w = n*a0;
        real_type d = n*a2 - w;
        real_type d0 = n*b0 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d0 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d1 > 0 )
          return true;
      }
      {
        vector3_type n  = a10 % b21;
        real_type w = n*a0;
        real_type d = n*a2 - w;
        real_type d2 = n*b2 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d2 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d2 > 0  && d1 > 0 )
          return true;
      }
      //--- SAT: a20 crossed with all edges of B
      {
        vector3_type n  = a20 % b10;
        real_type w = n*a0;
        real_type d = n*a1 - w;
        real_type d0 = n*b0 - w;
        real_type d2 = n*b2 - w;
        if(d > 0 && d0 < 0  && d2 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d2 > 0 )
          return true;
      }
      {
        vector3_type n  = a20 % b20;
        real_type w = n*a0;
        real_type d = n*a1 - w;
        real_type d0 = n*b0 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d0 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d1 > 0 )
          return true;
      }
      {
        vector3_type n  = a20 % b21;
        real_type w = n*a0;
        real_type d = n*a1 - w;
        real_type d2 = n*b2 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d2 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d2 > 0  && d1 > 0 )
          return true;
      }
      //--- SAT: a21 crossed with all edges of B
      {
        vector3_type n  = a21 % b10;
        real_type w = n*a1;
        real_type d = n*a0 - w;
        real_type d0 = n*b0 - w;
        real_type d2 = n*b2 - w;
        if(d > 0 && d0 < 0  && d2 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d2 > 0 )
          return true;
      }
      {
        vector3_type n  = a21 % b20;
        real_type w = n*a1;
        real_type d = n*a0 - w;
        real_type d0 = n*b0 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d0 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d0 > 0  && d1 > 0 )
          return true;
      }
      {
        vector3_type n  = a21 % b21;
        real_type w = n*a1;
        real_type d = n*a0 - w;
        real_type d2 = n*b2 - w;
        real_type d1 = n*b1 - w;
        if(d > 0 && d2 < 0  && d1 < 0 )
          return true;
        if(d < 0 && d2 > 0  && d1 > 0 )
          return true;
      }
      return false;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_SAT_H
#endif
