//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_compute_minimum_sine_dihedral_angle_quality_measure.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_geometry_util_compute_minimum_sine_dihedral_angle_measure);

  BOOST_AUTO_TEST_CASE(case_by_case_testing)
  {
    using std::sqrt;

    typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
    typedef math_types::vector3_type                         vector3_type;
    typedef math_types::real_type                            real_type;

    real_type tol = 0.0001;

    {
      // semi regular
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 0, 2);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;

      BOOST_CHECK_CLOSE( quality, 0.903508, tol);
    }
    {
      // close to flat
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 0, 0.01);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.010606071427483767, tol);
    }
    {
      // flat but still a simplex
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 0, 0);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.0, tol);
    }
    {
      // flat but no longer a simplex - quad-like
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 1, 1, 0);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.0, tol);
    }
    {
      // inverted
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 0, -1);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, -1.06066, tol);
    }
    {
      // degenerate 2 nodes
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 1,-1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 1, 0);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.0, tol);
    }
    {
      // degenerate 3 nodes
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 0, 1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 1, 0);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.0, tol);
    }
    {
      // degenerate 4 nodes
      vector3_type p0(-1,-1, 0);
      vector3_type p1( 0, 1, 0);
      vector3_type p2( 0, 1, 0);
      vector3_type p3( 0, 1, 0);
      real_type quality = OpenTissue::geometry::compute_minimum_sine_dihedral_angle_quality_measure(p0,p1,p2,p3);
      std::cout << quality << std::endl;
      BOOST_CHECK_CLOSE( quality, 0.0, tol);
    }

  }

BOOST_AUTO_TEST_SUITE_END();
