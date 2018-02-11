//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_plane_box.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_geometry_util_plane_box);

BOOST_AUTO_TEST_CASE(case_by_case_testing)
{

  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;

  typedef OpenTissue::geometry::PlaneBox<math_types>  plane_box_type;  
  plane_box_type plane_box;

  real_type tol = 0.0001;

  vector3_type min_coord(0,0,0);
  vector3_type max_coord(1,1,1);

  plane_box.init(min_coord,max_coord);
  BOOST_CHECK( plane_box.box().min().is_equal( min_coord, tol ) );
  BOOST_CHECK( plane_box.box().max().is_equal( max_coord, tol ) );

  BOOST_CHECK( plane_box.n() == vector3_type(1,0,0) );
  BOOST_CHECK( plane_box.plane().n() == vector3_type(1,0,0) );

  plane_box.set_y_axis();
  BOOST_CHECK( plane_box.n() == vector3_type(0,1,0) );
  BOOST_CHECK( plane_box.plane().n() == vector3_type(0,1,0) );

  plane_box.set_z_axis();
  BOOST_CHECK( plane_box.n() == vector3_type(0,0,1) );
  BOOST_CHECK( plane_box.plane().n() == vector3_type(0,0,1) );

  plane_box.set_x_axis();
  BOOST_CHECK( plane_box.n() == vector3_type(1,0,0) );
  BOOST_CHECK( plane_box.plane().n() == vector3_type(1,0,0) );

  real_type w1 = plane_box.plane().w();
  plane_box.decrement();
  real_type w2 = plane_box.plane().w();
  BOOST_CHECK( w2 < w1 );
  plane_box.increment();
  real_type w3 = plane_box.plane().w();
  BOOST_CHECK( w3 > w2 );

  plane_box.set_x_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.increment();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }

  plane_box.set_y_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.increment();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }

  plane_box.set_z_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.increment();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }


  plane_box.set_x_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.decrement();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }

  plane_box.set_y_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.decrement();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }

  plane_box.set_z_axis();
  for(size_t i=0;i<500;++i)
  {
    plane_box.decrement();
    BOOST_CHECK( plane_box.p0() <= max_coord );
    BOOST_CHECK( plane_box.p0() >= min_coord );
    BOOST_CHECK( plane_box.p1() <= max_coord );
    BOOST_CHECK( plane_box.p1() >= min_coord );
    BOOST_CHECK( plane_box.p2() <= max_coord );
    BOOST_CHECK( plane_box.p2() >= min_coord );
    BOOST_CHECK( plane_box.p3() <= max_coord );
    BOOST_CHECK( plane_box.p3() >= min_coord );
  }

}

BOOST_AUTO_TEST_SUITE_END();
