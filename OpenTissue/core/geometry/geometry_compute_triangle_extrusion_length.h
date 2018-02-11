#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_EXTRUSION_LENGTH_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_EXTRUSION_LENGTH_H
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
    *  Compute Triangle Extrusion Length.
    *  Maximum extrusion along normals, which do not result in a ``swallow tail'' case.
    *
    *  To get inward extrusion, flip normal directions!
    *
    *
    * @param p1      Coordinate of first vertex.
    * @param p2      Coordinate of second vertex.
    * @param p3      Coordinate of third vertex.
    * @param n1      Outward pointing vertex normal of first vertex.
    * @param n2      Outward pointing vertex normal of second vertex.
    * @param n3      Outward pointing vertex normal of third vertex.
    * @param inward  Boolean flag indicating wheter an outward or inward
    *                extrusion is computed.  I.e. in case of ourward
    *                extrusion the i'th extruded vertex coordinate is
    *                given by q_i = p_i + epsilon n_i, in case of inward
    *                extrusion q_i = p_i - epsilon n_i
    *
    * @return     The extrusion length along the normal directions. 
    */
    template< typename vector3_type >
    typename vector3_type::value_type 
      compute_triangle_extrusion_length(
      vector3_type const & p1
      , vector3_type const & p2
      , vector3_type const & p3
      , vector3_type const & n1
      , vector3_type const & n2
      , vector3_type const & n3
      , bool inward = false
      )
    {
      using std::min;

      typedef typename vector3_type::value_type  real_type;
      real_type extrusion = static_cast<real_type>(10e30);

      unsigned int count = 0;
      real_type roots[ 2 ];

      vector3_type a,b,c;

      if(inward)
      {
        c = ( p2 - p1) % (p3 - p1 );
        b = ( p2 - p1) % (n1 - n3 ) + ( n1 - n2 ) % ( p3 - p1 );
        a = ( n1 - n2) % (n1 - n3 );
      }
      else
      {
        c = ( p2 - p1) % (p3 - p1 );
        b = ( p2 - p1) % (n3 - n1 ) + ( n2 - n1 ) % ( p3 - p1 );
        a = ( n2 - n1) % (n3 - n1 );
      }

      real_type a1 = n1 * a;
      real_type b1 = n1 * b;
      real_type c1 = n1 * c;
      math::compute_polynomial_roots( c1, b1,  a1, count, roots );
      for ( unsigned int i = 0;i < count;++i )
        if ( roots[ i ] > 0 )
          extrusion = min( roots[ i ], extrusion );

      real_type a2 = n2 * a;
      real_type b2 = n2 * b;
      real_type c2 = n2 * c;
      math::compute_polynomial_roots( c2, b2, a2, count, roots );
      for ( unsigned int i = 0;i < count;++i )
        if ( roots[ i ] > 0 )
          extrusion = min( roots[ i ], extrusion );

      real_type a3 = n3 * a;
      real_type b3 = n3 * b;
      real_type c3 = n3 * c;
      math::compute_polynomial_roots( c3, b3, a3, count, roots );
      for ( unsigned int i = 0;i < count;++i )
        if ( roots[ i ] > 0 )
          extrusion = min( roots[ i ], extrusion );

      return extrusion;
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TRIANGLE_EXTRUSION_LENGTH_H
#endif
