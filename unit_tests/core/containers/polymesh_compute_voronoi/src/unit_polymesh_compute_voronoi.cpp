//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_voronoi.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_mesh_polymesh_voronoi);

BOOST_AUTO_TEST_CASE(simple_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;

  mesh_type   mesh;

  std::vector<vector3_type> sites(5);

  sites[0](0) =  1.0; sites[0](1) = -1.0; sites[0](2) =  0.0;
  sites[1](0) =  1.0; sites[1](1) =  1.0; sites[1](2) =  0.0;
  sites[2](0) = -1.0; sites[2](1) =  1.0; sites[2](2) =  0.0;
  sites[3](0) = -1.0; sites[3](1) = -1.0; sites[3](2) =  0.0;
  sites[4](0) =  0.0; sites[4](1) =  0.0; sites[4](2) =  0.0;

  OpenTissue::polymesh::compute_voronoi(sites,mesh);

  BOOST_CHECK(mesh.size_vertices() == 4 );
  BOOST_CHECK(mesh.size_faces() == 1 );
  BOOST_CHECK(mesh.size_edges() == 4 );
}

BOOST_AUTO_TEST_CASE(simple_test2)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;

  mesh_type   mesh;

  std::vector<vector3_type> sites(8);

  sites[0](0) =  1.0; sites[0](1) = -1.0; sites[0](2) =  0.0;
  sites[1](0) =  1.0; sites[1](1) =  1.0; sites[1](2) =  0.0;
  sites[2](0) = -1.0; sites[2](1) =  1.0; sites[2](2) =  0.0;
  sites[3](0) = -1.0; sites[3](1) = -1.0; sites[3](2) =  0.0;
  sites[4](0) =  0.0; sites[4](1) =  0.0; sites[4](2) =  0.0;
  sites[5](0) =  2.0; sites[5](1) =  0.0; sites[5](2) =  0.0;
  sites[6](0) =  3.0; sites[6](1) =  1.0; sites[6](2) =  0.0;
  sites[7](0) =  3.0; sites[7](1) = -1.0; sites[7](2) =  0.0;

  OpenTissue::polymesh::compute_voronoi(sites,mesh);

  BOOST_CHECK(mesh.size_vertices() == 8 );
  BOOST_CHECK(mesh.size_faces() == 2 );
  BOOST_CHECK(mesh.size_edges() == 9 );
}


BOOST_AUTO_TEST_SUITE_END();
