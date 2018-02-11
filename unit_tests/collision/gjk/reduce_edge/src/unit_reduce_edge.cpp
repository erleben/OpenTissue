//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_reduce_edge.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_reduce_edge);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;

  typedef OpenTissue::gjk::Simplex<vector3_type>           simplex_type;

  // First we create a simplex that represents an edge
  vector3_type const a = vector3_type(1.0, 0.0, 0.0);
  vector3_type const b = vector3_type(0.0, 0.0, 0.0);

  // First we use a test point that does not lie on the line

  // Front side of A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 1.5, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);

  }
  // Back side of A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.5, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK( S.m_bitmask == (bit_A | bit_B));
    
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // In A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 1.0, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }

  // Front side of B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( -0.5, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // Back side of B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.5, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK( S.m_bitmask == (bit_A | bit_B));
    
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // In B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.0, 1.0,  1.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }

  // Second we use a test point that lies on the line

  // Front side of A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 1.5, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // Back side of A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.5, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK( S.m_bitmask == (bit_A | bit_B));
    
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // In A voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 1.0, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }

  // Front side of B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( -0.5, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }
  // Back side of B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.5, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK( S.m_bitmask == (bit_A | bit_B));
    
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.5, 0.01);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.5, 0.01);
  }
  // In B voronoi plane
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = vector3_type( 0.0, 0.0,  0.0);

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 1u );

    int bit_A    = 0;
    size_t idx_A = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A );

    BOOST_CHECK(S.m_bitmask == bit_A );
    BOOST_CHECK(S.m_v[idx_A] == b);
    BOOST_CHECK(S.m_a[idx_A] == b);
    BOOST_CHECK(S.m_b[idx_A] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 1.0, 0.01);
  }

  // Assymetric test case when simplex is AB
  {
    simplex_type S;

    OpenTissue::gjk::add_point_to_simplex( a, a, a, S);
    OpenTissue::gjk::add_point_to_simplex( b, b, b, S);

    vector3_type const p = (2.0*a + 3.0*b)/5.0;

    OpenTissue::gjk::detail::reduce_edge( p, S );

    BOOST_CHECK( OpenTissue::gjk::dimension( S ) == 2u );

    int bit_A    = 0;
    size_t idx_A = 0;
    int bit_B    = 0;
    size_t idx_B = 0;
    OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );

    BOOST_CHECK( S.m_bitmask == (bit_A | bit_B));
    
    BOOST_CHECK(S.m_v[idx_A] == a);
    BOOST_CHECK(S.m_a[idx_A] == a);
    BOOST_CHECK(S.m_b[idx_A] == a);
    BOOST_CHECK_CLOSE(S.m_w[idx_A], 0.4, 0.01);

    BOOST_CHECK(S.m_v[idx_B] == b);
    BOOST_CHECK(S.m_a[idx_B] == b);
    BOOST_CHECK(S.m_b[idx_B] == b);
    BOOST_CHECK_CLOSE(S.m_w[idx_B], 0.6, 0.01);
  }



}

BOOST_AUTO_TEST_SUITE_END();
