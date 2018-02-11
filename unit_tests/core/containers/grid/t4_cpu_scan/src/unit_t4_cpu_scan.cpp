//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>

#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/geometry/t4_cpu_scan/t4_cpu_scan.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_make_sphere.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_face_normal.h>
#include <cmath> 

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_grid_util_t4_cpu_scan);

BOOST_AUTO_TEST_CASE(signed_test_case)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::grid::Grid<float,math_types>                grid_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;
  grid_type phi;
  mesh_type surface;

  real_type radius = 1.0;

  size_t I = 8;
  size_t J = 8;
  size_t K = 8;

  OpenTissue::polymesh::make_sphere( radius, 5, surface);

  phi.create(vector3_type(-1.0,-1.0,-1.0),vector3_type(1.0,1.0,1.0), I, J, K);

  OpenTissue::t4_cpu_scan(surface, radius, phi, OpenTissue::t4_cpu_signed() );

  real_type tol = 5.0;

  for(size_t i = 0;i<I;++i)
    for(size_t j = 0;j<J;++j)
      for(size_t k = 0;k<K;++k)
      {
        vector3_type coord;
        OpenTissue::grid::idx2coord(phi, i,j,k,coord);
        real_type tst = OpenTissue::math::length(coord) - radius;
        real_type value = phi(i,j,k);
        BOOST_CHECK_CLOSE(tst, value, tol);
      }
}

BOOST_AUTO_TEST_CASE(unsigned_test_case)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::grid::Grid<float,math_types>                grid_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;
  grid_type phi;
  mesh_type surface;

  real_type radius = 1.0;

  size_t I = 8;
  size_t J = 8;
  size_t K = 8;

  OpenTissue::polymesh::make_sphere( radius, 5, surface);

  phi.create(vector3_type(-1.0,-1.0,-1.0),vector3_type(1.0,1.0,1.0), I, J, K);

  OpenTissue::t4_cpu_scan(surface, radius, phi, OpenTissue::t4_cpu_unsigned() );

  real_type tol = 5.0;

  for(size_t i = 0;i<I;++i)
    for(size_t j = 0;j<J;++j)
      for(size_t k = 0;k<K;++k)
      {
        vector3_type coord;
        OpenTissue::grid::idx2coord(phi, i,j,k,coord);
        real_type tst = fabs( OpenTissue::math::length(coord) - radius );
        real_type value = phi(i,j,k);
        BOOST_CHECK_CLOSE(tst, value, tol);
      }
}


BOOST_AUTO_TEST_SUITE_END();
