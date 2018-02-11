#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_CIRCUMSCRIBED_RADIUS_QUALITY_MEASURE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_CIRCUMSCRIBED_RADIUS_QUALITY_MEASURE_H
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
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_circumscribed_sphere.h>
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
    * The quality of a tetrahedron is 3.0 times the ratio of the radius of
    * the inscribed sphere divided by that of the circumscribed sphere.
    *
    * An equilateral tetrahredron achieves the maximum possible quality of 1.
    *
    * @param p0     The first coordinate of the tetrahedron.
    * @param p1     The second coordinate of the tetrahedron.
    * @param p2     The third coordinate of the tetrahedron.
    * @param p3     The fourth coordinate of the tetrahedron.
    *
    * @return The computed quality measure.
    */
    template<typename vector3_type>
    inline typename vector3_type::value_type   
      compute_inscribed_circumscribed_radius_quality_measure ( 
          vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
    {
      typedef typename vector3_type::value_type                   real_type;
      typedef OpenTissue::math::BasicMathTypes<real_type, size_t> math_types;
      typedef Sphere<math_types>                                  sphere_type;
      typedef typename math_types::value_traits                   value_traits;

      sphere_type sphere;
      compute_circumscribed_sphere( p0, p1, p2, p3, sphere);
      real_type r_out = sphere.radius();

      real_type r_in;
      vector3_type center;
      compute_tetrahedron_inscribed_sphere( p0, p1, p2, p3, center, r_in);

      if(r_in<value_traits::zero())
      {
        //tetrahedron was inverted, so we imaginary flip it!
        r_in = -r_in;
      }

      real_type quality = boost::numeric_cast<real_type>(3.0) * (r_in / r_out);
      assert( is_number( r_out )             || !"circumscribed radius was not a number"                   );
      assert( r_out >= value_traits::zero()  || !"circumscribed radius must be non-negative"               );
      assert( is_number( r_in )              || !"circumscribed radius was not a number"                   );
      assert( r_in >= value_traits::zero()   || !"inscribed radius must be non-negative"                   );
      assert( r_in <= r_out                  || !"inscribed radius must be less than circumscribed radius" );
      assert( is_number( quality )           || !"quality was not a number"                                );
      assert( quality>= value_traits::zero() || !"invalid qualtiy value"                                   );
      assert( quality<= value_traits::one()  || !"invalid qualtiy value"                                   );
      return quality;
    };

  } // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_INSCRIBED_CIRCUMSCRIBED_RADIUS_QUALITY_MEASURE_H
#endif
