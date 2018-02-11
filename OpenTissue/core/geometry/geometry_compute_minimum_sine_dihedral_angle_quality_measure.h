#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_MINIMUM_SINE_DIHEDRAL_ANGLE_QUALITY_MEASURE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_MINIMUM_SINE_DIHEDRAL_ANGLE_QUALITY_MEASURE_H
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
    * This routine computes minimum sine of the dihedral angles of a tetrahedron.
    *
    *   \frac{9 \sqrt{2}}{8} V min_{1 \leq i < j \leq 4} \frac{l_{ij}}{A_k A_l}
    *
    * where l_ij is the length of the edge between vertices i and j. A_k is the
    * signed area of the triangle opposing the k'th vertex. V is the signed
    * volume of the tetrahedron.
    *
    * As described in the paper: 
    * 
    *      What Is a Good Linear Finite element? Interpolation, Conditioning, Anisotropy, and Quality Measures (Preprint) 
    *
    * by Jonathan Shewchuk.
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
      compute_minimum_sine_dihedral_angle_quality_measure ( 
          vector3_type const & p0
        , vector3_type const & p1
        , vector3_type const & p2
        , vector3_type const & p3
        )
    {
      using std::sqrt;
      using std::min;

      typedef typename vector3_type::value_type                   real_type;
      typedef OpenTissue::math::BasicMathTypes<real_type, size_t> math_types;
      typedef typename math_types::value_traits                   value_traits;
      typedef Tetrahedron<math_types>                             tetrahedron_type;

      real_type const sqr2_mul_9_div_8 = boost::numeric_cast<real_type>( 1.5909902576697319299018998147359 );

      vector3_type e_01 = (p0-p1);
      vector3_type e_12 = (p1-p2);
      vector3_type e_20 = (p2-p0);
      vector3_type e_03 = (p0-p3);
      vector3_type e_13 = (p1-p3);
      vector3_type e_23 = (p2-p3);

      real_type A3 =  value_traits::half() * length( cross( e_20, e_01 ) );  // cross(e_20, e_10) = cross(   e_20, - e_01) = - cross(e_20, e_01)
      real_type A2 =  value_traits::half() * length( cross( e_01, e_03 ) );  // cross(e_10, e_30) = cross( - e_01, - e_03) =   cross(e_01, e_03)
      real_type A1 =  value_traits::half() * length( cross( e_03, e_20 ) );  // cross(e_30, e_20) = cross( - e_03,   e_20) = - cross(e_03, e_20)
      real_type A0 =  value_traits::half() * length( cross( e_12, e_13 ) );  // cross(e_21, e_31) = cross( - e_12, - e_13) = - cross(e_12, e_13)

      assert( is_number( A0 ) || !"A0 was not a number" );
      assert( is_number( A1 ) || !"A1 was not a number" );
      assert( is_number( A2 ) || !"A2 was not a number" );
      assert( is_number( A3 ) || !"A3 was not a number" );

      // If any of the A's are zero then we will have a division by zero
      // when computing the quality measure
      //
      // If A becomes zero then we have a flat tetrahedron dihedral
      // angles must then be 0 and 180 degress, implying that sin(theta_{ij})
      // is always zero. By definition the quality measure should be zero
      //
      //   \frac{3 \sqrt{2}}{4} min_{i,j} sin(theta_{ij})
      //
      if( !( A0 > value_traits::zero() || A0 < value_traits::zero()) )
        return value_traits::zero();
      if( !( A1 > value_traits::zero() || A1 < value_traits::zero()) )
        return value_traits::zero();
      if( !( A2 > value_traits::zero() || A2 < value_traits::zero()) )
        return value_traits::zero();
      if( !( A3 > value_traits::zero() || A3 < value_traits::zero()) )
        return value_traits::zero();

      real_type l_01 = sqrt( inner_prod(e_01,e_01) );
      real_type l_12 = sqrt( inner_prod(e_12,e_12) );
      real_type l_20 = sqrt( inner_prod(e_20,e_20) );
      real_type l_03 = sqrt( inner_prod(e_03,e_03) );
      real_type l_13 = sqrt( inner_prod(e_13,e_13) );
      real_type l_23 = sqrt( inner_prod(e_23,e_23) );

      tetrahedron_type T(p0,p1,p2,p3);

      real_type c = sqr2_mul_9_div_8 * T.volume();

      assert( is_number( c ) || !"factor c was not a number" );

      real_type q1 = c * (l_01) / (A2*A3);
      real_type q2 = c * (l_12) / (A0*A3);
      real_type q3 = c * (l_20) / (A1*A3);
      real_type q4 = c * (l_03) / (A1*A2);
      real_type q5 = c * (l_13) / (A0*A2);
      real_type q6 = c * (l_23) / (A0*A1);

      assert( is_number( q1 ) || !"q1 was not a number" );
      assert( is_number( q2 ) || !"q2 was not a number" );
      assert( is_number( q3 ) || !"q3 was not a number" );
      assert( is_number( q4 ) || !"q4 was not a number" );
      assert( is_number( q5 ) || !"q5 was not a number" );
      assert( is_number( q6 ) || !"q6 was not a number" );

      real_type min_q = min( q1, min( q2, min( q3, min( q4, min( q5, q6 ) ) ) ) );

      assert( is_number( min_q ) || !"minimum sine was not a number" );

      return min_q;
    }

  } // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_MINIMUM_SINE_DIHEDRAL_ANGLE_QUALITY_MEASURE_H
#endif
