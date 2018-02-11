//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_signed_distance_to_edge_face_voronoi_plane.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_signed_distance_to_edge_face_vp);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;


  vector3_type a = vector3_type(0.0, 0.0, 0.0);
  vector3_type b = vector3_type(1.0, 0.0, 0.0);;
  vector3_type c = vector3_type(0.0, 1.0, 0.0);;

  // Front side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, -1.0,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 1.0,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 0.0,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of AC voronoi plane
  {
    vector3_type p = vector3_type(-1.0, 0.5,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 0.5,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.5,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of BC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 1.0,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );
  }
  // Back side of BC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.0,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );
  }
  // In BC voronoi plane
  {
    vector3_type p = vector3_type( 0.5, 0.5,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }



  // We just used a point above the face plane, next we will try using a test point lying in the face plane



  // Front side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, -1.0,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 1.0,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 0.0,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of AC voronoi plane
  {
    vector3_type p = vector3_type(-1.0, 0.5,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 0.5,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.5,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of BC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 1.0,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );
  }
  // Back side of BC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.0,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );
  }
  // In BC voronoi plane
  {
    vector3_type p = vector3_type( 0.5, 0.5,  0.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }



  // Finally we will use a test point lying below the face-plane



  // Front side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, -1.0,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 1.0,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AB voronoi plane
  {
    vector3_type p = vector3_type(-0.5, 0.0,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, b, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, a, c);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of AC voronoi plane
  {
    vector3_type p = vector3_type(-1.0, 0.5,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }
  // Back side of AC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 0.5,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }
  // In AC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.5,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, a, c, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, a, b);      
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }
  // Front side of BC voronoi plane
  {
    vector3_type p = vector3_type( 1.0, 1.0,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.70710678118654752440084436210485, 0.01 );
  }
  // Back side of BC voronoi plane
  {
    vector3_type p = vector3_type( 0.0, 0.0, -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, -0.70710678118654752440084436210485, 0.01 );
  }
  // In BC voronoi plane
  {
    vector3_type p = vector3_type( 0.5, 0.5, -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, b, c, a); 
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );

    sign_p = OpenTissue::gjk::detail::signed_distance_to_edge_face_voronoi_plane(p, c, b, a);
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }

}

BOOST_AUTO_TEST_SUITE_END();
