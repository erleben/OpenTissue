//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/geometry/geometry_compute_inscribed_circumscribed_radius_quality_measure.h>
#include <OpenTissue/core/geometry/geometry_compute_volume_length_quality_measure.h>
#include <OpenTissue/core/geometry/geometry_compute_inscribed_radius_length_quality_measure.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_compute_mesh_quality.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_t4mesh_compute_mesh_quality);

BOOST_AUTO_TEST_CASE(quality_testing)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t>   math_types;
  typedef OpenTissue::t4mesh::T4Mesh< math_types >           mesh_type;
  typedef math_types::vector3_type                           vector3_type;
  typedef vector3_type::value_type                           real_type;

  std::vector<vector3_type> c;
  c.resize( 6 );
  c[0] = vector3_type(0,0,0);
  c[1] = vector3_type(0,0,1);
  c[2] = vector3_type(0,1,0);
  c[3] = vector3_type(0,1,1);
  c[4] = vector3_type(1,0,0);
  c[5] = vector3_type(1,0,1);

  mesh_type M;
  M.insert( c[0] );
  M.insert( c[1] );
  M.insert( c[2] );
  M.insert( c[3] );
  M.insert( c[4] );
  M.insert( c[5] );
  M.insert(0,1,2,3);
  M.insert(0,1,2,4);

  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M, c, Q, &(compute_inscribed_circumscribed_radius_quality_measure<vector3_type>)  );

    BOOST_CHECK(Q.size() == 2);
  }
  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M,Q, &(compute_inscribed_circumscribed_radius_quality_measure<vector3_type>)  );

    BOOST_CHECK(Q.size() == 2);
  }
  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M,c, Q, &(compute_volume_length_quality_measure<vector3_type>)  );

    BOOST_CHECK(Q.size() == 2);
  }
  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M,Q, &(compute_volume_length_quality_measure<vector3_type>)  );

    BOOST_CHECK(Q.size() == 2);
  }
  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M, c, Q, &(compute_inscribed_radius_length_quality_measure<vector3_type>)  );

    BOOST_CHECK(Q.size() == 2);
  }
  {
    using namespace OpenTissue::geometry;
    std::vector<real_type> Q;
    OpenTissue::t4mesh::compute_mesh_quality( M, Q, &(compute_inscribed_radius_length_quality_measure<vector3_type>)  );
    BOOST_CHECK(Q.size() == 2);
  }

}

BOOST_AUTO_TEST_SUITE_END();
