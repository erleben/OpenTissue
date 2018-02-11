//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_simplex.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_simplex);

BOOST_AUTO_TEST_CASE(simplex_testing)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;

  typedef OpenTissue::gjk::Simplex<vector3_type>  simplex_type;

  vector3_type const not_in_simplex = vector3_type(1.5, 5.0, 1.0);


  simplex_type S;

  // Vertify that simplex have been initialized correctly as being empty and ``zeroed''
  BOOST_CHECK( S.m_bitmask == 0u );
  for(size_t i=0u; i<4; ++i)
  {
    BOOST_CHECK( S.m_w[0] == 0.0 );
    for(size_t j=0u; j<3; ++j)
    {
      BOOST_CHECK( S.m_v[i](j) == 0.0 );
      BOOST_CHECK( S.m_a[i](j) == 0.0 );
      BOOST_CHECK( S.m_b[i](j) == 0.0 );
    }
  }

  // Vertify how different query methods behave on an empty Simplex

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );

  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 0u );

  int bit_A    = 0;
  size_t idx_A = 0;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ), std::invalid_argument );

  int bit_B    = 0;
  size_t idx_B = 0;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ), std::invalid_argument );

  int bit_C    = 0;
  size_t idx_C = 0;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ), std::invalid_argument );

  // Next try to insert one simplex vertex into the simplex

  vector3_type const p1 = vector3_type(1.0, 0.0, 0.0);
  vector3_type const a1 = vector3_type(1.0, 1.0, 0.0);
  vector3_type const b1 = vector3_type(1.0, 0.0, 1.0);

  BOOST_CHECK_NO_THROW( OpenTissue::gjk::add_point_to_simplex( p1, a1, b1, S ) );

  // Verify how differnt query method works on a 1-simplex
  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 1u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ), std::logic_error );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ), std::logic_error );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );

  // Next try to insert one more simplex vertex into the simplex

  vector3_type const p2 = vector3_type(2.0, 0.5, 1.0);
  vector3_type const a2 = vector3_type(2.0, 1.0, 7.0);
  vector3_type const b2 = vector3_type(2.0, 0.5, 1.0);

  BOOST_CHECK_NO_THROW( OpenTissue::gjk::add_point_to_simplex( p2, a2, b2, S ) );

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 2u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p2 );
  BOOST_CHECK( S.m_a[idx_B] == a2 );
  BOOST_CHECK( S.m_b[idx_B] == b2 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ), std::logic_error );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p2, S )             );


  // Insert one more simplex vertex

  vector3_type const p3 = vector3_type(2.3, 7.5, 1.2);
  vector3_type const a3 = vector3_type(2.1, 1.1, 2.3);
  vector3_type const b3 = vector3_type(2.2, 2.5, 0.1);

  BOOST_CHECK_NO_THROW( OpenTissue::gjk::add_point_to_simplex( p3, a3, b3, S ) );

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 3u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p2 );
  BOOST_CHECK( S.m_a[idx_B] == a2 );
  BOOST_CHECK( S.m_b[idx_B] == b2 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p2 );
  BOOST_CHECK( S.m_a[idx_B] == a2 );
  BOOST_CHECK( S.m_b[idx_B] == b2 );

  BOOST_CHECK( bit_C == 4 );
  BOOST_CHECK( idx_C == 2 );
  BOOST_CHECK( S.m_v[idx_C] == p3 );
  BOOST_CHECK( S.m_a[idx_C] == a3 );
  BOOST_CHECK( S.m_b[idx_C] == b3 );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p2, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p3, S )             );


  // Insert one more vertex then we have a full simplex

  vector3_type const p4 = vector3_type(1.3, 1.5, 1.2);
  vector3_type const a4 = vector3_type(1.1, 1.1, 1.3);
  vector3_type const b4 = vector3_type(1.2, 1.5, 1.1);

  BOOST_CHECK_NO_THROW( OpenTissue::gjk::add_point_to_simplex( p4, a4, b4, S ) );

  BOOST_CHECK( OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 4u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p2 );
  BOOST_CHECK( S.m_a[idx_B] == a2 );
  BOOST_CHECK( S.m_b[idx_B] == b2 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p2 );
  BOOST_CHECK( S.m_a[idx_B] == a2 );
  BOOST_CHECK( S.m_b[idx_B] == b2 );

  BOOST_CHECK( bit_C == 4 );
  BOOST_CHECK( idx_C == 2 );
  BOOST_CHECK( S.m_v[idx_C] == p3 );
  BOOST_CHECK( S.m_a[idx_C] == a3 );
  BOOST_CHECK( S.m_b[idx_C] == b3 );

  BOOST_CHECK( S.m_v[3] == p4 );
  BOOST_CHECK( S.m_a[3] == a4 );
  BOOST_CHECK( S.m_b[3] == b4 );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p2, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p3, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p4, S )             );

  // Test what happens if we try to insert five vertices into the simplex

  vector3_type const p5 = vector3_type(2.3, 2.5, 2.2);
  vector3_type const a5 = vector3_type(2.1, 2.1, 2.3);
  vector3_type const b5 = vector3_type(2.2, 2.5, 2.1);

  BOOST_CHECK_THROW( OpenTissue::gjk::add_point_to_simplex( p5, a5, b5, S ), std::logic_error );

  // Now let us erase one of the simplex vertices

  S.m_bitmask = S.m_bitmask & ~bit_B;

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 3u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 4 );
  BOOST_CHECK( idx_B == 2 );
  BOOST_CHECK( S.m_v[idx_B] == p3 );
  BOOST_CHECK( S.m_a[idx_B] == a3 );
  BOOST_CHECK( S.m_b[idx_B] == b3 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 4 );
  BOOST_CHECK( idx_B == 2 );
  BOOST_CHECK( S.m_v[idx_B] == p3 );
  BOOST_CHECK( S.m_a[idx_B] == a3 );
  BOOST_CHECK( S.m_b[idx_B] == b3 );

  BOOST_CHECK( bit_C == 8 );
  BOOST_CHECK( idx_C == 3 );
  BOOST_CHECK( S.m_v[idx_C] == p4 );
  BOOST_CHECK( S.m_a[idx_C] == a4 );
  BOOST_CHECK( S.m_b[idx_C] == b4 );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( p2, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p3, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p4, S )             );

  // Let us erase one more simplex

  S.m_bitmask = S.m_bitmask & ~bit_B;

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 2u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 8 );
  BOOST_CHECK( idx_B == 3 );
  BOOST_CHECK( S.m_v[idx_B] == p4 );
  BOOST_CHECK( S.m_a[idx_B] == a4 );
  BOOST_CHECK( S.m_b[idx_B] == b4 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ), std::logic_error );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( p2, S )             );
  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( p3, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p4, S )             );


  // Insert a new simplex vertex

  BOOST_CHECK_NO_THROW( OpenTissue::gjk::add_point_to_simplex( p5, a5, b5, S ) );

  BOOST_CHECK( !OpenTissue::gjk::is_full_simplex(S) );
  BOOST_CHECK( OpenTissue::gjk::dimension(S) == 3u );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A ) );
  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p5 );
  BOOST_CHECK( S.m_a[idx_B] == a5 );
  BOOST_CHECK( S.m_b[idx_B] == b5 );

  bit_A  = 0xFFFF;
  idx_A  = 0xFFFF;
  bit_B  = 0xFFFF;
  idx_B  = 0xFFFF;
  bit_C  = 0xFFFF;
  idx_C  = 0xFFFF;
  BOOST_CHECK_NO_THROW( OpenTissue::gjk::get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B, idx_C, bit_C ) );

  BOOST_CHECK( bit_A == 1 );
  BOOST_CHECK( idx_A == 0 );
  BOOST_CHECK( S.m_v[idx_A] == p1 );
  BOOST_CHECK( S.m_a[idx_A] == a1 );
  BOOST_CHECK( S.m_b[idx_A] == b1 );

  BOOST_CHECK( bit_B == 2 );
  BOOST_CHECK( idx_B == 1 );
  BOOST_CHECK( S.m_v[idx_B] == p5 );
  BOOST_CHECK( S.m_a[idx_B] == a5 );
  BOOST_CHECK( S.m_b[idx_B] == b5 );

  BOOST_CHECK( bit_C == 8 );
  BOOST_CHECK( idx_C == 3 );
  BOOST_CHECK( S.m_v[idx_C] == p4 );
  BOOST_CHECK( S.m_a[idx_C] == a4 );
  BOOST_CHECK( S.m_b[idx_C] == b4 );

  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( not_in_simplex, S ) );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p1, S )             );
  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( p2, S )             );
  BOOST_CHECK( !OpenTissue::gjk::is_point_in_simplex( p3, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p4, S )             );
  BOOST_CHECK(  OpenTissue::gjk::is_point_in_simplex( p5, S )             );

}

BOOST_AUTO_TEST_SUITE_END();
