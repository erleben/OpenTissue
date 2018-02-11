//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_outside_edge_face_voronoi_plane.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_outside_edge_face);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         V;


  V const a = V(0.0, 0.0, 0.0);
  V const b = V(1.0, 0.0, 0.0);;
  V const c = V(0.0, 1.0, 0.0);;

  // Front side of AB voronoi plane
  {
    V p = V(-0.5, -1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AB voronoi plane
  {
    V p = V(-0.5, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( !outside2 );
  }
  // In AB voronoi plane
  {
    V p = V(-0.5, 0.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Front side of AC voronoi plane
  {
    V p = V(-1.0, 0.5,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AC voronoi plane
  {
    V p = V( 1.0, 0.5,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( !outside2 );
  }
  // In AC voronoi plane
  {
    V p = V( 0.0, 0.5,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Front side of BC voronoi plane
  {
    V p = V( 1.0, 1.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }
  // Back side of BC voronoi plane
  {
    V p = V( 0.0, 0.0,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( !outside2 );
  }
  // In BC voronoi plane
  {
    V p = V( 0.5, 0.5,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }

  // We just used a point above the face plane, next we will try using a test point lying in the face plane

  // Front side of AB voronoi plane
  {
    V p = V(-0.5, -1.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AB voronoi plane
  {
    V p = V(-0.5, 1.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( !outside2 );
  }
  // In AB voronoi plane
  {
    V p = V(-0.5, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Front side of AC voronoi plane
  {
    V p = V(-1.0, 0.5,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AC voronoi plane
  {
    V p = V( 1.0, 0.5,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( !outside2 );
  }
  // In AC voronoi plane
  {
    V p = V( 0.0, 0.5,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Front side of BC voronoi plane
  {
    V p = V( 1.0, 1.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }
  // Back side of BC voronoi plane
  {
    V p = V( 0.0, 0.0,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( !outside2 );
  }
  // In BC voronoi plane
  {
    V p = V( 0.5, 0.5,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }

  // Finally we will use a test point lying below the face-plane

  // Front side of AB voronoi plane
  {
    V p = V(-0.5, -1.0,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AB voronoi plane
  {
    V p = V(-0.5, 1.0,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( !outside2 );
  }
  // In AB voronoi plane
  {
    V p = V(-0.5, 0.0,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK( outside2 );
  }
  // Front side of AC voronoi plane
  {
    V p = V(-1.0, 0.5,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Back side of AC voronoi plane
  {
    V p = V( 1.0, 0.5,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( !outside2 );
  }
  // In AC voronoi plane
  {
    V p = V( 0.0, 0.5,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK( outside2 );
  }
  // Front side of BC voronoi plane
  {
    V p = V( 1.0, 1.0,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }
  // Back side of BC voronoi plane
  {
    V p = V( 0.0, 0.0, -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK( !outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( !outside2 );
  }
  // In BC voronoi plane
  {
    V p = V( 0.5, 0.5, -1.0);
    bool outside = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK( outside );

    bool outside2 = OpenTissue::gjk::detail::outside_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK( outside2 );
  }

}

BOOST_AUTO_TEST_SUITE_END();
