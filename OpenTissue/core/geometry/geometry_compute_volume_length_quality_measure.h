#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_VOLUME_LENGTH_QUALITY_MEASURE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_VOLUME_LENGTH_QUALITY_MEASURE_H
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
#include <OpenTissue/core/geometry/geometry_tetrahedron.h>

#include <boost/cast.hpp> // needed for boost::numeric_cast

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Compute Tetrahedron Quality Measure.
    * This routine computes the eigenvalue or mean ratio of a tetrahedron.
    *
    *      12 * ( 3 * volume )**(2/3) / (sum of square of edge lengths).
    *
    * This value may be used as a shape quality measure for the tetrahedron.
    *
    * For an equilateral tetrahedron, the value of this quality measure
    * will be 1.  For any other tetrahedron, the value will be between
    * 0 and 1.
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
      compute_volume_length_quality_measure ( 
          vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
    {
      using std::pow;
      using std::fabs;

      typedef typename vector3_type::value_type                   real_type;
      typedef OpenTissue::math::BasicMathTypes<real_type, size_t> math_types;
      typedef typename math_types::value_traits                   value_traits;
      typedef Tetrahedron<math_types>                             tetrahedron_type;

      vector3_type e0 = (p0-p1);
      vector3_type e1 = (p1-p2);
      vector3_type e2 = (p2-p0);
      vector3_type e3 = (p0-p3);
      vector3_type e4 = (p1-p3);
      vector3_type e5 = (p2-p3);
      real_type l0 = inner_prod(e0,e0);
      real_type l1 = inner_prod(e1,e1);
      real_type l2 = inner_prod(e2,e2);
      real_type l3 = inner_prod(e3,e3);
      real_type l4 = inner_prod(e4,e4);
      real_type l5 = inner_prod(e5,e5);
      real_type denom = l0 + l1 + l2 + l3 + l4 + l5;

      if ( denom == value_traits::zero() )
        return value_traits::zero();

      tetrahedron_type T(p0,p1,p2,p3);

      real_type const exponent = boost::numeric_cast<real_type>( 2.0 / 3.0 );
      real_type base = boost::numeric_cast<real_type>( 3.0 * fabs( T.volume() ) );
      real_type nominator = boost::numeric_cast<real_type>(12.0 * pow (base , exponent ));
      real_type quality = nominator / denom;
      assert( is_number( base )                  || !"base was not a number"          );
      assert( base >= value_traits::zero()       || !"base must be non-negative"      );
      assert( is_number( nominator )             || !"nominator was not a number"     );
      assert( nominator >= value_traits::zero()  || !"nominator must be non-negative" );
      assert( is_number( denom )                 || !"denom was not a number"         );
      assert( denom > value_traits::zero()       || !"denom must be positive"         );
      assert( is_number( quality )               || !"quality was not a number"       );
      assert( quality>= value_traits::zero()     || !"invalid qualtiy value"          );
      assert( quality<= value_traits::one()      || !"invalid qualtiy value"          );
      return quality;
    };

  } // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_VOLUME_LENGTH_QUALITY_MEASURE_H
#endif
