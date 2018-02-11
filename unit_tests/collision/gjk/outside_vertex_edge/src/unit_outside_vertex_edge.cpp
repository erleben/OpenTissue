//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_outside_vertex_edge_voronoi_plane.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_outside_vertex_edge);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         V;

  V const a = V(1.0, 0.0, 0.0);
  V const b = V(0.0, 0.0, 0.0);

  // First we use a test point that does not lie on the line

  // Front side of A voronoi plane
  {
    V p = V( 2.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( outside );
  }
  // Back side of A voronoi plane
  {
    V p = V( 0.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( !outside );
  }
  // In A voronoi plane
  {
    V p = V( 1.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( outside );
  }

  // Front side of B voronoi plane
  {
    V p = V( -1.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a);
    BOOST_CHECK( outside );
  }
  // Back side of B voronoi plane
  {
    V p = V( 1.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a);
    BOOST_CHECK( !outside );
  }
  // In B voronoi plane
  {
    V p = V( 0.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a);
    BOOST_CHECK( outside );
  }

  // Second we use a test point that lies on the line

  // Front side of A voronoi plane
  {
    V p = V( 2.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( outside );
  }
  // Back side of A voronoi plane
  {
    V p = V( 0.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( !outside );
  }
  // In A voronoi plane
  {
    V p = V( 1.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, a, b);
    BOOST_CHECK( outside );
  }

  // Front side of B voronoi plane
  {
    V p = V( -1.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a);
    BOOST_CHECK( outside );
  }
  // Back side of B voronoi plane
  {
    V p = V( 1.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a);
    BOOST_CHECK( !outside );
  }
  // In B voronoi plane
  {
    V p = V( 0.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_vertex_edge_voronoi_plane(p, b, a );
    BOOST_CHECK( outside );
  }

}

BOOST_AUTO_TEST_SUITE_END();
