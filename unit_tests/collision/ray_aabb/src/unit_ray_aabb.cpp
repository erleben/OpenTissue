//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/collision_ray_aabb.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

BOOST_AUTO_TEST_SUITE(opentissue_collision_ray_aabb);

BOOST_AUTO_TEST_CASE(simple_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  

  vector3_type min_coord( -1.0, -1.0, -1.0 );
  vector3_type max_coord(  1.0,  1.0,  1.0 );
  vector3_type r;
  vector3_type p;
  bool collision = false;
  
  // x-faces are separating planes

  r = vector3_type(1.0, 0.0, 0.0 ) ;
  p = vector3_type(2.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );

  r = vector3_type(-1.0, 0.0, 0.0 ) ;
  p = vector3_type(2.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(1.0, 0.0, 0.0 ) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(-1.0, 0.0, 0.0 ) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  // y-faces are separating planes

  r = vector3_type(0.0, 1.0, 0.0 ) ;
  p = vector3_type(0.0, 2.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );

  r = vector3_type(0.0, -1.0, 0.0 ) ;
  p = vector3_type(0.0,  2.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(0.0, 1.0, 0.0 ) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(0.0, -1.0, 0.0 ) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  // z-faces are separating planes

  r = vector3_type(0.0, 0.0, 1.0 ) ;
  p = vector3_type(0.0, 0.0, 2.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );

  r = vector3_type(0.0, 0.0, -1.0 ) ;
  p = vector3_type(0.0, 0.0,  2.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(0.0, 0.0, 1.0) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );

  r = vector3_type(0.0, 0.0, -1.0) ;
  p = vector3_type(0.0, 0.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( collision );


  // cross-produces are separating planes
  r = vector3_type(0.01, 0.01,   1.0) ;
  p = vector3_type(-2.0, -2.0, 0.0 ) ;
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );

  r = vector3_type(0.01, 1.0, 0.01);
  p = vector3_type(-2.0, 0.0, -2.0);
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );

  r = vector3_type(1.0, 0.01, 0.01);
  p = vector3_type(0.0, -2.0, -2.0);
  collision = OpenTissue::collision::ray_aabb(p,r,min_coord,max_coord);
  BOOST_CHECK( !collision );
}

BOOST_AUTO_TEST_SUITE_END();
