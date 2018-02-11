#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_RADIUS_LENGTH_QUALITY_MEASURE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_RADIUS_LENGTH_QUALITY_MEASURE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_compute_inscribed_sphere.h>

#include <boost/cast.hpp> // needed for boost::numeric_cast

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute Tetrahedron Quality Measure.
    * The quality measure of a tetrahedron is:
    *
    *     2 * sqrt ( 6 ) * RIN / LMAX
    *
    * where
    *
    *      RIN = radius of the inscribed sphere;
    *      LMAX = length of longest side of the tetrahedron.
    *
    * An equilateral tetrahredron achieves the maximum possible quality of 1.
    *
    * @param p0     The first coordinate of the tetrahedron.
    * @param p1     The second coordinate of the tetrahedron.
    * @param p2     The third coordinate of the tetrahedron.
    * @param p3     The fourth coordinate of the tetrahedron.
    *
    * @return The computed quality measure.
    *
    */
    template<typename vector3_type>
    inline typename vector3_type::value_type   
      compute_inscribed_radius_length_quality_measure ( 
          vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
    {
      using std::max;

      typedef typename vector3_type::value_type                   real_type;
      typedef OpenTissue::math::BasicMathTypes<real_type, size_t> math_types;
      typedef typename math_types::value_traits                   value_traits;

      real_type l0 = length(p0-p1);
      real_type l1 = length(p1-p2);
      real_type l2 = length(p2-p0);
      real_type l3 = length(p0-p3);
      real_type l4 = length(p1-p3);
      real_type l5 = length(p2-p3);
      real_type l_max = max( l0, max( l1, max( l2, max( l3, max( l4, l5 ) ) ) ) );

      real_type r_in;
      vector3_type center;

      compute_tetrahedron_inscribed_sphere( p0, p1, p2, p3, center, r_in);

      r_in = (r_in<0) ? 0 : r_in;

      real_type const fraction = boost::numeric_cast<real_type>( 2.0 * sqrt ( 6.0 ) );
      real_type quality =  fraction * r_in / l_max;
      assert( is_number( l_max )             || !"maximum edge length was not a number"                   );
      assert( l_max >= value_traits::zero()  || !"maximum edge length must be non-negative"               );
      assert( is_number( r_in )              || !"circumscribed radius was not a number"                   );
      assert( r_in >= value_traits::zero()   || !"inscribed radius must be non-negative"                   );
      assert( is_number( quality )           || !"quality was not a number"                                );
      assert( quality>= value_traits::zero() || !"invalid qualtiy value"                                   );
      assert( quality<= value_traits::one()  || !"invalid qualtiy value"                                   );
      return quality;
    };



  } // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_RADIUS_LENGTH_QUALITY_MEASURE_H
#endif
