//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk.h>
#include <OpenTissue/core/geometry/geometry_obb.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk);

BOOST_AUTO_TEST_CASE(face_aligned_separated_boxes)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::real_type                            real_type;

  vector3_type p_a;                   
  vector3_type p_b;                   
  real_type tol = 0.01;
  OpenTissue::gjk::obsolete::detail::GJK<vector3_type > gjk;          ///< GJK collision detection Algorithm.
  OpenTissue::geometry::OBB<math_types> A;
  OpenTissue::geometry::OBB<math_types> B;

  A.init(1.0,1.0,1.0);
  B.init(1.0,1.0,1.0);

  vector3_type vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector3_type vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector3_type vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector3_type vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector3_type vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector3_type vector_mz = vector3_type( 0.0, 0.0,-1.0);

  coordsys_type  Awcs = coordsys_type( vector_mx, quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector_px, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  real_type distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector_mz = vector3_type( 0.0, 0.0,-1.0);
  Awcs = coordsys_type( vector_px, quaternion_type());   
  Bwcs = coordsys_type( vector_mx, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector_mz = vector3_type( 0.0, 0.0,-1.0);
  Awcs = coordsys_type( vector_py, quaternion_type());   
  Bwcs = coordsys_type( vector_my, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector_mz = vector3_type( 0.0, 0.0,-1.0);
  Awcs = coordsys_type( vector_my, quaternion_type());   
  Bwcs = coordsys_type( vector_py, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector_mz = vector3_type( 0.0, 0.0,-1.0);
  Awcs = coordsys_type( vector_pz, quaternion_type());   
  Bwcs = coordsys_type( vector_mz, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  vector_py = vector3_type( 0.0, 1.0, 0.0);
  vector_my = vector3_type( 0.0,-1.0, 0.0);
  vector_pz = vector3_type( 0.0, 0.0, 1.0);
  vector_mz = vector3_type( 0.0, 0.0,-1.0);
  Awcs = coordsys_type( vector_mz, quaternion_type());   
  Bwcs = coordsys_type( vector_pz, quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );
}

BOOST_AUTO_TEST_CASE(non_aligned_cases)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::real_type                            real_type;

  vector3_type p_a;                   
  vector3_type p_b;                   
  real_type tol = 0.01;
  OpenTissue::gjk::obsolete::detail::GJK<vector3_type > gjk;          ///< GJK collision detection Algorithm.
  OpenTissue::geometry::OBB<math_types> A;
  OpenTissue::geometry::OBB<math_types> B;

  A.init(1.0,1.0,1.0);
  B.init(1.0,1.0,1.0);

  real_type sqrt_half = std::sqrt(0.5);
  real_type test_distance = 1.0 - (sqrt_half - 0.5);

  quaternion_type q;
  q.Rz( math_types::value_traits::pi()/4.0 );

  vector3_type vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector3_type vector_mx = vector3_type(-1.0, 0.0, 0.0);
  coordsys_type  Awcs = coordsys_type( vector_mx, quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector_px, q);      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  real_type distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, test_distance, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  Awcs = coordsys_type( vector_mx, quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 1.0,0.1,0.1), q);      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, test_distance, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.5), quaternion_type());   
  Bwcs = coordsys_type( vector_px, q);      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, test_distance, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  vector_mx = vector3_type(-1.0, 0.0, 0.0);
  Awcs = coordsys_type( vector3_type(-1.0,0.0,-0.5), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.5), quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK_CLOSE(distance, 1.0, tol);
  BOOST_CHECK( !gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( !gjk.get_common_point(A,B,vector_px,p_a,p_b) );
}

BOOST_AUTO_TEST_CASE(penetrating_cases)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::real_type                            real_type;

  vector3_type p_a;                   
  vector3_type p_b;                   

  OpenTissue::gjk::obsolete::detail::GJK<vector3_type > gjk;          ///< GJK collision detection Algorithm.
  OpenTissue::geometry::OBB<math_types> A;
  OpenTissue::geometry::OBB<math_types> B;

  A.init(1.0,1.0,1.0);
  B.init(1.0,1.0,1.0);

  vector3_type vector_px = vector3_type( 1.0, 0.0, 0.0);
  coordsys_type  Awcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector3_type( 0.25,0.25,0.25), quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);
  gjk.get_closest_points(A,B,p_a,p_b);
  real_type distance = length( p_a - p_b);
  BOOST_CHECK(fabs(distance) < 10e-7);
  BOOST_CHECK( gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( gjk.get_common_point(A,B,vector_px,p_a,p_b) );

  B.init(0.9,0.9,0.9);

  vector_px = vector3_type( 1.0, 0.0, 0.0);
  Awcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());      
  A.place(Awcs);
  B.place(Bwcs);

  gjk.get_closest_points(A,B,p_a,p_b);
  distance = length( p_a - p_b);
  BOOST_CHECK(fabs(distance) < 10e-7);
  BOOST_CHECK( gjk.is_intersecting(A,B,vector_px) );
  BOOST_CHECK( gjk.get_common_point(A,B,vector_px,p_a,p_b) );
}

BOOST_AUTO_TEST_SUITE_END();
