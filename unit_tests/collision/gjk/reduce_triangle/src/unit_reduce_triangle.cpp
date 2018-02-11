//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_reduce_triangle.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_reduce_triangle);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;

  typedef OpenTissue::gjk::Simplex<vector3_type>           simplex_type;



  // Inside face-region new simplex should be ABC
  {
    vector3_type const a = vector3_type(-1.0, -1.0, 0.0);
    vector3_type const b = vector3_type( 1.0, -1.0, 0.0);
    vector3_type const c = vector3_type( 0.5,  0.1, 0.0);

    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    for(size_t i=1;i<10;++i)
      for(size_t j=1;j<10;++j)
        for(size_t k=1;k<10;++k)
        {
          real_type u = i*0.1;
          real_type v = j*0.1;
          real_type w = k*0.1;

          real_type lgh = u + v +w;
          u /= lgh;
          v /= lgh;
          w /= lgh;

          vector3_type const p = u*a + v*b + w*c;

          OpenTissue::gjk::detail::reduce_triangle( p, S );

          BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 3u );

          int bit_A    = 0;
          size_t idx_A = 0;
          int bit_B    = 0;
          size_t idx_B = 0;
          int bit_C    = 0;
          size_t idx_C = 0;
          OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B , idx_C, bit_C );

          BOOST_CHECK(S.m_bitmask == (bit_A | bit_B | bit_C));
          BOOST_CHECK(S.m_v[idx_A] == a);
          BOOST_CHECK(S.m_a[idx_A] == a);
          BOOST_CHECK(S.m_b[idx_A] == a);

          BOOST_CHECK(S.m_v[idx_B] == b);
          BOOST_CHECK(S.m_a[idx_B] == b);
          BOOST_CHECK(S.m_b[idx_B] == b);

          BOOST_CHECK(S.m_v[idx_C] == c);
          BOOST_CHECK(S.m_a[idx_C] == c);
          BOOST_CHECK(S.m_b[idx_C] == c);

          BOOST_CHECK_CLOSE(S.m_w[idx_A], u, 0.01);
          BOOST_CHECK_CLOSE(S.m_w[idx_B], v, 0.01);
          BOOST_CHECK_CLOSE(S.m_w[idx_C], w, 0.01);

        }
  }


  // First we create a simplex that represents an triangle
  vector3_type const a = vector3_type(-1.0, -1.0, 0.0);
  vector3_type const b = vector3_type( 1.0, -1.0, 0.0);
  vector3_type const c = vector3_type( 0.0,  1.0, 0.0);

  // Inside face-region new simplex should be ABC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( 0.0, 0.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 3u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    int bit_C    = 0;
    size_t idx_C = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B , idx_C, bit_C );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B | bit_C));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);

    BOOST_CHECK(S.m_v[idx_C] == c);
    BOOST_CHECK(S.m_a[idx_C] == c);
    BOOST_CHECK(S.m_b[idx_C] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.25, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.25, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_C], 0.5, 0.01);
  }
  // Inside A voronoi region new simplex should be A
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( -2.0, -1.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // Inside B voronoi region new simplex should be B
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type(  2.0, -1.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.00, 0.01);
  }
  // Inside C voronoi region new simplex should be C
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type(  0.0, 2.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A  );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == c);
    BOOST_CHECK(S.m_a[idx_A] == c);
    BOOST_CHECK(S.m_b[idx_A] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // Inside AB voronoi region new simplex should be AB
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( 0.0, -2.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // Inside BC voronoi region new simplex should be BC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( 1.5, 0.5,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // Inside AC voronoi region new simplex should be AC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( -1.5, 0.5,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }


  // On vertex A new simplex should be A
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    OpenTissue::gjk::detail::reduce_triangle( a, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // On vertex B new simplex should be B
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    OpenTissue::gjk::detail::reduce_triangle( b, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.00, 0.01);
  }
  // On vertex C new simplex should be C
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    OpenTissue::gjk::detail::reduce_triangle( c, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A  );

    BOOST_CHECK(S.m_bitmask == bit_A);
    BOOST_CHECK(S.m_v[idx_A] == c);
    BOOST_CHECK(S.m_a[idx_A] == c);
    BOOST_CHECK(S.m_b[idx_A] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // On edge AB new simplex should be AB
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( 0.0, -1.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // On edge BC new simplex should be BC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( 0.5, 0.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // On edge AC new simplex should be AC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = vector3_type( -0.5, 0.0,  1.0);

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }


  // Assymmetric test cases

  { // New simplex should be AB
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = 0.4*a + 0.6*b;

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.4, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.6, 0.01);
  }
  // New simplex should be BC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = 0.4*b + 0.6*c;

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.4, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.6, 0.01);
  }
  // New simplex should be AC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = 0.6*a + 0.4*c;

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B ));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == c);
    BOOST_CHECK(S.m_a[idx_B] == c);
    BOOST_CHECK(S.m_b[idx_B] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.6, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.4, 0.01);
  }


  // New simplex should be ABC
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);
    OpenTissue::gjk::add_point_to_simplex( c, c, c, S);

    vector3_type const p = 0.1*a + 0.2*b + 0.7*c;

    OpenTissue::gjk::detail::reduce_triangle( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 3u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    int bit_C    = 0;
    size_t idx_C = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B , idx_C, bit_C );

    BOOST_CHECK(S.m_bitmask == (bit_A | bit_B | bit_C));
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);

    BOOST_CHECK(S.m_v[idx_C] == c);
    BOOST_CHECK(S.m_a[idx_C] == c);
    BOOST_CHECK(S.m_b[idx_C] == c);

    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.1, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.2, 0.01);
    BOOST_CHECK_CLOSE(S.m_w[idx_C], 0.7, 0.01);
  }

}

BOOST_AUTO_TEST_SUITE_END();
