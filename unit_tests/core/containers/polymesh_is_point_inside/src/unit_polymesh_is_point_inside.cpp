//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_point_inside.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_make_box.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

BOOST_AUTO_TEST_SUITE(opentissue_polymesh_is_point_inside);

BOOST_AUTO_TEST_CASE(box_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::matrix3x3_type                       matrix3x3_type;
  typedef math_types::real_type                            real_type;

  OpenTissue::polymesh::PolyMesh<math_types> mesh;

  OpenTissue::mesh::make_box(1.0,1.0,1.0,mesh);


  //std::vector<vector3_type> profile;
  //profile.push_back(vector3_type(0.0,0.0,0.0));
  //profile.push_back(vector3_type(5.0,0.0,5.0));
  //profile.push_back(vector3_type(5.0,0.0,10.0));
  //profile.push_back(vector3_type(0.0,0.0,15.0));
  //OpenTissue::mesh::profile_sweep(profile.begin(),profile.end(),2*OpenTissue::math::detail::pi<real_type>(),32,m_mesh);

  bool inside = false;
  vector3_type p;
  real_type val = 0.0;

  p = vector3_type(0.0,0.0,0.0);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );

  val = 0.45;
  p = vector3_type(val,val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(val,val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(val,-val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(val,-val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(-val,val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(-val,val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(-val,-val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );
  p = vector3_type(-val,-val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( inside );

  val = 0.8;
  p = vector3_type(val,val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(val,val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(val,-val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(val,-val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(-val,val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(-val,val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(-val,-val,val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );
  p = vector3_type(-val,-val,-val);
  inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  BOOST_CHECK( !inside );

}

BOOST_AUTO_TEST_CASE(advanced_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::matrix3x3_type                       matrix3x3_type;
  typedef math_types::real_type                            real_type;

  OpenTissue::polymesh::PolyMesh<math_types> mesh;

  //std::vector<vector3_type> profile;
  //profile.push_back(vector3_type(0.0,0.0,0.0));
  //profile.push_back(vector3_type(5.0,0.0,5.0));
  //profile.push_back(vector3_type(5.0,0.0,10.0));
  //profile.push_back(vector3_type(0.0,0.0,15.0));
  //OpenTissue::mesh::profile_sweep(profile.begin(),profile.end(),2*OpenTissue::math::detail::pi<real_type>(),32,m_mesh);

  //bool inside = false;
  vector3_type p;

  //p = vector3_type(0.0,0.0,0.0);
  //inside = OpenTissue::polymesh::is_point_inside( mesh, p );  
  //BOOST_CHECK( inside );
}

BOOST_AUTO_TEST_SUITE_END();
