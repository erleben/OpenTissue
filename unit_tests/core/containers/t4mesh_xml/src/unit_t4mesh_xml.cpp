//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_write.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_read.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_t4mesh_t4mesh);

BOOST_AUTO_TEST_CASE(xml_testing)
{
  typedef OpenTissue::math::BasicMathTypes<int, size_t>    math_types;
  typedef OpenTissue::t4mesh::T4Mesh< math_types >         mesh_type;
  typedef math_types::vector3_type                         vector3_type;

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
    mesh_type W;
    bool success = true;
    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_read("do_not_exist.xml",W) );
    BOOST_CHECK(!success);
  }
  {
    mesh_type W;
    std::vector<vector3_type> d;
    bool success = true;
    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_read("do_not_exist.xml",W,d) );
    BOOST_CHECK(!success);
  }
  {
    mesh_type W;

    bool success = false;
    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_write("t4mesh_test1.xml",M) );
    BOOST_CHECK(success);

    bool has_tags = true;
    BOOST_CHECK_NO_THROW( has_tags = OpenTissue::t4mesh::xml_has_tags("t4mesh_test1.xml") );
    BOOST_CHECK(!has_tags);

    BOOST_CHECK( W.size_nodes() == 0 );
    BOOST_CHECK( W.size_tetrahedra() == 0 );

    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_read("t4mesh_test1.xml",W) );
    BOOST_CHECK(success);

    BOOST_CHECK( M.size_nodes() == W.size_nodes() );
    BOOST_CHECK( M.size_tetrahedra() == W.size_tetrahedra() );
    for(size_t n = 0; n < M.size_nodes(); ++n)
    {
      BOOST_CHECK( M.node(n)->idx() == W.node(n)->idx());
      BOOST_CHECK( M.node(n)->m_coord == W.node(n)->m_coord );
    }
    for(size_t t = 0; t < M.size_tetrahedra(); ++t)
    {
      BOOST_CHECK( M.tetrahedron(t)->idx() == W.tetrahedron(t)->idx());
      BOOST_CHECK( M.tetrahedron(t)->i()->idx() == W.tetrahedron(t)->i()->idx());
      BOOST_CHECK( M.tetrahedron(t)->j()->idx() == W.tetrahedron(t)->j()->idx());
      BOOST_CHECK( M.tetrahedron(t)->k()->idx() == W.tetrahedron(t)->k()->idx());
      BOOST_CHECK( M.tetrahedron(t)->m()->idx() == W.tetrahedron(t)->m()->idx());

    }
  }
  {
    mesh_type W;
    std::vector<vector3_type> d;

    bool success = false;
    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_write("t4mesh_test2.xml",M,c) );
    BOOST_CHECK(success);

    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_read("t4mesh_test2.xml",W,d) );
    BOOST_CHECK(success);

    BOOST_CHECK( M.size_nodes() == W.size_nodes() );
    BOOST_CHECK( M.size_tetrahedra() == W.size_tetrahedra() );
    for(size_t n = 0; n < M.size_nodes(); ++n)
    {
      BOOST_CHECK( M.node(n)->idx() == W.node(n)->idx());
      BOOST_CHECK( d[n] == c[n] );
    }
    for(size_t t = 0; t < M.size_tetrahedra(); ++t)
    {
      BOOST_CHECK( M.tetrahedron(t)->idx() == W.tetrahedron(t)->idx());
      BOOST_CHECK( M.tetrahedron(t)->i()->idx() == W.tetrahedron(t)->i()->idx());
      BOOST_CHECK( M.tetrahedron(t)->j()->idx() == W.tetrahedron(t)->j()->idx());
      BOOST_CHECK( M.tetrahedron(t)->k()->idx() == W.tetrahedron(t)->k()->idx());
      BOOST_CHECK( M.tetrahedron(t)->m()->idx() == W.tetrahedron(t)->m()->idx());

    }
  }
}

    template<typename math_types>
  class NodeTraits 
    : public OpenTissue::t4mesh::DefaultNodeTraits<math_types>
    , public OpenTissue::utility::default_tag_supported_type
  {
  };

//    template<typename math_types>
  class TetrahedronTraits 
    : public OpenTissue::t4mesh::DefaultTetrahedronTraits
    , public OpenTissue::utility::default_tag_supported_type
  {
  };


BOOST_AUTO_TEST_CASE(xml_testing_with_tags)
{

  typedef OpenTissue::math::BasicMathTypes<int, size_t>                            math_types;
  typedef OpenTissue::t4mesh::T4Mesh< math_types, NodeTraits<math_types>, TetrahedronTraits >  mesh_type;
  typedef math_types::vector3_type                                                 vector3_type;

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

  OpenTissue::utility::set_tag( *(M.node(0)) , 0 );
  OpenTissue::utility::set_tag( *(M.node(1)) , 1 );
  OpenTissue::utility::set_tag( *(M.node(2)) , 2 );
  OpenTissue::utility::set_tag( *(M.node(3)) , 3 );
  OpenTissue::utility::set_tag( *(M.node(4)) , 4 );
  OpenTissue::utility::set_tag( *(M.node(5)) , 5 );
  OpenTissue::utility::set_tag( *(M.tetrahedron(0)) , 0 );
  OpenTissue::utility::set_tag( *(M.tetrahedron(1)) , 1 );

  {
    mesh_type W;

    bool success = false;
    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_write("t4mesh_test3.xml",M) );
    BOOST_CHECK(success);

    bool has_tags = true;
    BOOST_CHECK_NO_THROW( has_tags = OpenTissue::t4mesh::xml_has_tags("t4mesh_test3.xml") );
    BOOST_CHECK(has_tags);

    BOOST_CHECK( W.size_nodes() == 0 );
    BOOST_CHECK( W.size_tetrahedra() == 0 );

    BOOST_CHECK_NO_THROW( success = OpenTissue::t4mesh::xml_read("t4mesh_test3.xml",W) );
    BOOST_CHECK(success);

    BOOST_CHECK( M.size_nodes() == W.size_nodes() );
    BOOST_CHECK( M.size_tetrahedra() == W.size_tetrahedra() );
    for(size_t n = 0; n < M.size_nodes(); ++n)
    {
      BOOST_CHECK( M.node(n)->idx() == W.node(n)->idx());
      BOOST_CHECK( M.node(n)->m_coord == W.node(n)->m_coord );
      BOOST_CHECK( tag_value( *( M.node(n))) ==  OpenTissue::utility::tag_value( *(W.node(n)))  );
    }
    for(size_t t = 0; t < M.size_tetrahedra(); ++t)
    {
      BOOST_CHECK( M.tetrahedron(t)->idx() == W.tetrahedron(t)->idx());
      BOOST_CHECK( M.tetrahedron(t)->i()->idx() == W.tetrahedron(t)->i()->idx());
      BOOST_CHECK( M.tetrahedron(t)->j()->idx() == W.tetrahedron(t)->j()->idx());
      BOOST_CHECK( M.tetrahedron(t)->k()->idx() == W.tetrahedron(t)->k()->idx());
      BOOST_CHECK( M.tetrahedron(t)->m()->idx() == W.tetrahedron(t)->m()->idx());
      BOOST_CHECK( OpenTissue::utility::tag_value( *(M.tetrahedron(t))) ==  OpenTissue::utility::tag_value( *(W.tetrahedron(t)) ) );
    }
  }
}

BOOST_AUTO_TEST_SUITE_END();
