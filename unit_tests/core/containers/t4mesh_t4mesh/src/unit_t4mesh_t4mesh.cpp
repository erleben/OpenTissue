//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


template<typename mesh_type>
void t4mesh_compile_testing()
{
  typedef typename mesh_type::node_iterator               node_iterator;
  typedef typename mesh_type::tetrahedron_iterator        tetrahedron_iterator;
  typedef typename mesh_type::const_node_iterator         const_node_iterator;
  typedef typename mesh_type::const_tetrahedron_iterator  const_tetrahedron_iterator;
  typedef typename mesh_type::math_types                  math_types;
  typedef typename math_types::vector3_type               vector3_type;
  typedef typename math_types::value_traits               value_traits;
  mesh_type mesh;

  {
    node_iterator i = mesh.insert();
    node_iterator j = mesh.insert();
    node_iterator k = mesh.insert();
    node_iterator m = mesh.insert();
    tetrahedron_iterator T = mesh.insert(i,j,k,m);
  }

  mesh.size_nodes();
  mesh.size_tetrahedra();

  node_iterator n0 = mesh.node(0);
  node_iterator n1 = mesh.node(1);
  node_iterator n2 = mesh.node(2);
  node_iterator n3 = mesh.node(3);
  tetrahedron_iterator t0 = mesh.tetrahedron(0);

  node_iterator const_n0 = mesh.const_node(0);
  node_iterator const_n1 = mesh.const_node(1);
  node_iterator const_n2 = mesh.const_node(2);
  node_iterator const_n3 = mesh.const_node(3);
  tetrahedron_iterator const_t0 = mesh.tetrahedron(0);

  for(typename mesh_type::node_type::tetrahedron_circulator c= n0->begin();c!=n0->end();++c);

  n0->isolated();
  n0->size_tetrahedra();
  n0->idx();
  n0->owner();

  {
    node_iterator i = t0->i();
    node_iterator j = t0->j();
    node_iterator k = t0->k();
    node_iterator m = t0->m();

    tetrahedron_iterator jkm = t0->jkm();
    tetrahedron_iterator ijm = t0->ijm();
    tetrahedron_iterator kim = t0->kim();
    tetrahedron_iterator ikj = t0->ikj();

    t0->idx();
    t0->owner();
    t0->node(0);
    t0->global2local(25);
    t0->local2global(0);

    t0->has_face(i,j,k);
  }

  mesh.clear();

  {
    vector3_type coord( value_traits::one()  , value_traits::zero() , value_traits::zero());

    node_iterator i = mesh.insert( coord );
    node_iterator j = mesh.insert( coord );
    node_iterator k = mesh.insert( coord  );
    node_iterator m = mesh.insert( coord );
    tetrahedron_iterator T = mesh.insert(i->idx(),j->idx(),k->idx(),m->idx());
    tetrahedron_iterator T2 = mesh.find(i,j,k,m);
    mesh.erase(T2);
  }

  for(node_iterator n = mesh.node_begin();n!=mesh.node_end();++n);
  for(const_node_iterator n = mesh.node_begin();n!=mesh.node_end();++n);

  for(tetrahedron_iterator t = mesh.tetrahedron_begin();t!=mesh.tetrahedron_end();++t);
  for(const_tetrahedron_iterator t = mesh.tetrahedron_begin();t!=mesh.tetrahedron_end();++t);


  OpenTissue::t4mesh::default_point_container<mesh_type> points(&mesh);

  OpenTissue::t4mesh::T4BoundaryFaces<mesh_type, OpenTissue::t4mesh::DefaultT4FaceTraits > faces( mesh );
  faces.begin();
  faces.end();

  OpenTissue::t4mesh::T4Edges<mesh_type, OpenTissue::t4mesh::DefaultT4EdgeTraits > edges( mesh );

  edges.begin();
  edges.end();

}









BOOST_AUTO_TEST_SUITE(opentissue_t4mesh_t4mesh);

BOOST_AUTO_TEST_CASE(compile_test)
{
  typedef OpenTissue::t4mesh::T4Mesh<OpenTissue::math::BasicMathTypes<float, size_t> >            float_mesh_type;
  typedef OpenTissue::t4mesh::T4Mesh<OpenTissue::math::BasicMathTypes<double, size_t> >           double_mesh_type;

  void (*ptr_float)() = &t4mesh_compile_testing< float_mesh_type >;
  ptr_float = 0;
  void (*ptr_double)() = &t4mesh_compile_testing< double_mesh_type >;
  ptr_double = 0;
}


BOOST_AUTO_TEST_CASE(mesh_manipulation_testing)
{

  typedef OpenTissue::math::BasicMathTypes<int, size_t>    math_types;
  typedef OpenTissue::t4mesh::T4Mesh< math_types >         mesh_type;

  typedef mesh_type::node_iterator               node_iterator;
  typedef mesh_type::tetrahedron_iterator        tetrahedron_iterator;
  typedef mesh_type::const_node_iterator         const_node_iterator;
  typedef mesh_type::const_tetrahedron_iterator  const_tetrahedron_iterator;
  typedef mesh_type::math_types                  math_types;
  typedef math_types::vector3_type               vector3_type;
  typedef math_types::value_traits               value_traits;

  mesh_type M;
  BOOST_CHECK(M.size_nodes() == 0 );
  M.clear();
  BOOST_CHECK(M.size_nodes() == 0 );
  BOOST_CHECK_THROW(  M.tetrahedron(0), std::out_of_range);
  BOOST_CHECK_THROW(  M.tetrahedron(1), std::out_of_range);
  mesh_type const & R = M;
  BOOST_CHECK_THROW(  R.tetrahedron(0), std::out_of_range);
  BOOST_CHECK_THROW(  R.tetrahedron(1), std::out_of_range);
  BOOST_CHECK_THROW(  M.node(0), std::out_of_range);
  BOOST_CHECK_THROW(  M.node(1), std::out_of_range);
  BOOST_CHECK_THROW(  M.const_node(0), std::out_of_range);
  BOOST_CHECK_THROW(  M.const_node(1), std::out_of_range);

  {
    node_iterator n   = M.node_begin();
    node_iterator end = M.node_end();
    BOOST_CHECK( n==end );
  }
  {
    const_node_iterator n   = M.node_begin();
    const_node_iterator end = M.node_end();
    BOOST_CHECK( n==end );
  }
  {
    tetrahedron_iterator t   = M.tetrahedron_begin();
    tetrahedron_iterator end = M.tetrahedron_end();
    BOOST_CHECK( t==end );
  }
  {
    const_tetrahedron_iterator t   = M.tetrahedron_begin();
    const_tetrahedron_iterator end = M.tetrahedron_end();
    BOOST_CHECK( t==end );
  }
  {
    node_iterator n0 = M.insert();
    BOOST_CHECK(M.size_nodes()==1 );
    node_iterator n1 = M.insert();
    BOOST_CHECK(M.size_nodes()==2 );
    node_iterator n2 = M.insert();
    BOOST_CHECK(M.size_nodes()==3 );
    node_iterator n3 = M.insert();
    BOOST_CHECK(M.size_nodes()==4 );
    BOOST_CHECK(n0->idx() == 0);
    BOOST_CHECK(n1->idx() == 1);
    BOOST_CHECK(n2->idx() == 2);
    BOOST_CHECK(n3->idx() == 3);
    BOOST_CHECK_THROW(  M.node(4), std::out_of_range);
    BOOST_CHECK_THROW(  M.node(5), std::out_of_range);
    BOOST_CHECK_THROW(  M.const_node(4), std::out_of_range);
    BOOST_CHECK_THROW(  M.const_node(5), std::out_of_range);
    BOOST_CHECK_NO_THROW(  M.node(n0->idx()) );
    BOOST_CHECK_NO_THROW(  M.node(n1->idx()) );
    BOOST_CHECK_NO_THROW(  M.node(n2->idx()) );
    BOOST_CHECK_NO_THROW(  M.node(n3->idx()) );
    node_iterator nn0 = M.node(n0->idx());
    node_iterator nn1 = M.node(n1->idx());
    node_iterator nn2 = M.node(n2->idx());
    node_iterator nn3 = M.node(n3->idx());
    BOOST_CHECK(n0 == nn0);
    BOOST_CHECK(n1 == nn1);
    BOOST_CHECK(n2 == nn2);
    BOOST_CHECK(n3 == nn3);
    BOOST_CHECK(n0->idx() == nn0->idx());
    BOOST_CHECK(n1->idx() == nn1->idx());
    BOOST_CHECK(n2->idx() == nn2->idx());
    BOOST_CHECK(n3->idx() == nn3->idx());
    const_node_iterator cn0 = M.node(n0->idx());
    const_node_iterator cn1 = M.node(n1->idx());
    const_node_iterator cn2 = M.node(n2->idx());
    const_node_iterator cn3 = M.node(n3->idx());
    BOOST_CHECK( cn0->owner() == &M  );
    BOOST_CHECK( cn1->owner() == &M  );
    BOOST_CHECK( cn2->owner() == &M  );
    BOOST_CHECK( cn3->owner() == &M  );
    BOOST_CHECK(n0 == cn0);
    BOOST_CHECK(n1 == cn1);
    BOOST_CHECK(n2 == cn2);
    BOOST_CHECK(n3 == cn3);
    BOOST_CHECK(n0->idx() == cn0->idx());
    BOOST_CHECK(n1->idx() == cn1->idx());
    BOOST_CHECK(n2->idx() == cn2->idx());
    BOOST_CHECK(n3->idx() == cn3->idx());
    node_iterator m;
    BOOST_CHECK_THROW( ++m     , std::logic_error);
    BOOST_CHECK_THROW( *m      , std::logic_error);
    BOOST_CHECK_THROW( m->idx(), std::logic_error);
    node_iterator n;
    BOOST_CHECK_NO_THROW( n = M.node_begin() );
    node_iterator end = M.node_end();
    BOOST_CHECK( n0->idx() == n->idx() );
    BOOST_CHECK( n0 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n1->idx() == n->idx() );
    BOOST_CHECK( n1 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n2->idx() == n->idx() );
    BOOST_CHECK( n2 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n3->idx() == n->idx() );
    BOOST_CHECK( n3 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n==end                );
    BOOST_CHECK_THROW( n->idx() , std::out_of_range );
    BOOST_CHECK_THROW( *n , std::out_of_range );
    BOOST_CHECK_THROW( ++n , std::out_of_range );
    BOOST_CHECK_THROW( n->idx() , std::out_of_range );
    BOOST_CHECK_THROW( *n , std::out_of_range );
    M.clear();
    BOOST_CHECK(M.size_nodes() == 0 );
  }
  {
    node_iterator n0 = M.insert();
    BOOST_CHECK(M.size_nodes()==1 );
    node_iterator n1 = M.insert();
    BOOST_CHECK(M.size_nodes()==2 );
    node_iterator n2 = M.insert();
    BOOST_CHECK(M.size_nodes()==3 );
    node_iterator n3 = M.insert();
    BOOST_CHECK(M.size_nodes()==4 );
    const_node_iterator m;
    BOOST_CHECK_THROW( ++m     , std::logic_error);
    BOOST_CHECK_THROW( *m      , std::logic_error);
    BOOST_CHECK_THROW( m->idx(), std::logic_error);
    const_node_iterator n;
    BOOST_CHECK_NO_THROW( n = M.node_begin() );
    const_node_iterator end = M.node_end();
    BOOST_CHECK( n0->idx() == n->idx() );
    BOOST_CHECK( n0 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n1->idx() == n->idx() );
    BOOST_CHECK( n1 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n2->idx() == n->idx() );
    BOOST_CHECK( n2 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n3->idx() == n->idx() );
    BOOST_CHECK( n3 == n               );
    BOOST_CHECK( n!=end                );
    BOOST_CHECK_NO_THROW( ++n );
    BOOST_CHECK( n==end                );
    BOOST_CHECK_THROW( n->idx() , std::out_of_range );
    BOOST_CHECK_THROW( *n , std::out_of_range );
    BOOST_CHECK_THROW( ++n , std::out_of_range );
    BOOST_CHECK_THROW( n->idx() , std::out_of_range );
    BOOST_CHECK_THROW( *n , std::out_of_range );
    M.clear();
    BOOST_CHECK(M.size_nodes() == 0 );
  }
  typedef mesh_type::node_type::tetrahedron_circulator   tetrahedron_circulator;
  {
    vector3_type c0(1,0,0);
    vector3_type c1(0,0,1);
    vector3_type c2(0,1,0);
    vector3_type c3(1,0,1);
    node_iterator n0 = M.insert( c0 );
    node_iterator n1 = M.insert( c1 );
    node_iterator n2 = M.insert( c2 );
    node_iterator n3 = M.insert( c3 );
    BOOST_CHECK( n0->m_coord == c0 );
    BOOST_CHECK( n1->m_coord == c1 );
    BOOST_CHECK( n2->m_coord == c2 );
    BOOST_CHECK( n3->m_coord == c3 );

    BOOST_CHECK( n0->isolated()  );
    BOOST_CHECK( n1->isolated()  );
    BOOST_CHECK( n2->isolated()  );
    BOOST_CHECK( n3->isolated()  );

    BOOST_CHECK( n0->size_tetrahedra() == 0  );
    BOOST_CHECK( n1->size_tetrahedra() == 0  );
    BOOST_CHECK( n2->size_tetrahedra() == 0  );
    BOOST_CHECK( n3->size_tetrahedra() == 0  );

    BOOST_CHECK( n0->owner() == &M  );
    BOOST_CHECK( n1->owner() == &M  );
    BOOST_CHECK( n2->owner() == &M  );
    BOOST_CHECK( n3->owner() == &M  );

    tetrahedron_circulator c = n0->begin();
    tetrahedron_circulator end = n0->end();
    BOOST_CHECK( c == end );

    M.clear();
  }

  {
    BOOST_CHECK_THROW( M.insert(0,1,2,3), std::out_of_range );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    node_iterator n0 = M.insert( );
    BOOST_CHECK_THROW( M.insert(0,1,2,3), std::out_of_range );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    node_iterator n1 = M.insert( );
    BOOST_CHECK_THROW( M.insert(0,1,2,3), std::out_of_range );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    node_iterator n2 = M.insert( );
    BOOST_CHECK_THROW( M.insert(0,1,2,3), std::out_of_range );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    node_iterator n3 = M.insert( );

    node_iterator n4 = M.insert( );
    node_iterator n5 = M.insert( );

    BOOST_CHECK_THROW( M.insert(0,0,0,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(0,0,0,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(0,0,1,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(0,0,1,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(0,1,1,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,0,0,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,0,0,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,0,1,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,0,1,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,1,0,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,1,0,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,1,1,0), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );
    BOOST_CHECK_THROW( M.insert(1,1,1,1), std::logic_error );
    BOOST_CHECK( M.size_tetrahedra() == 0 );

    tetrahedron_iterator t0;
    BOOST_CHECK_NO_THROW( t0 = M.insert(0,1,2,3) );
    BOOST_CHECK( M.size_tetrahedra() == 1 );

    BOOST_CHECK( !n0->isolated()  );
    BOOST_CHECK( !n1->isolated()  );
    BOOST_CHECK( !n2->isolated()  );
    BOOST_CHECK( !n3->isolated()  );
    BOOST_CHECK( n4->isolated()  );
    BOOST_CHECK( n5->isolated()  );

    BOOST_CHECK( n0->size_tetrahedra() == 1  );
    BOOST_CHECK( n1->size_tetrahedra() == 1  );
    BOOST_CHECK( n2->size_tetrahedra() == 1  );
    BOOST_CHECK( n3->size_tetrahedra() == 1  );
    BOOST_CHECK( n4->size_tetrahedra() == 0  );
    BOOST_CHECK( n5->size_tetrahedra() == 0  );

    BOOST_CHECK( t0->owner() == &M  );

    tetrahedron_circulator c0 = n0->begin();
    tetrahedron_circulator end0 = n0->end();
    BOOST_CHECK( c0->idx() == t0->idx() );
    BOOST_CHECK_NO_THROW( ++c0);
    BOOST_CHECK( c0 == end0 );

    tetrahedron_circulator c1 = n1->begin();
    tetrahedron_circulator end1 = n1->end();
    BOOST_CHECK( c1->idx() == t0->idx() );
    BOOST_CHECK_NO_THROW( ++c1);
    BOOST_CHECK( c1 == end1 );

    tetrahedron_circulator c2 = n2->begin();
    tetrahedron_circulator end2 = n2->end();
    BOOST_CHECK( c2->idx() == t0->idx() );
    BOOST_CHECK_NO_THROW( ++c2);
    BOOST_CHECK( c2 == end2 );

    tetrahedron_circulator c3 = n3->begin();
    tetrahedron_circulator end3 = n3->end();
    BOOST_CHECK( c3->idx() == t0->idx() );
    BOOST_CHECK_NO_THROW( ++c3);
    BOOST_CHECK( c3 == end3 );

    BOOST_CHECK( n4->isolated() );
    BOOST_CHECK( n4->size_tetrahedra() == 0 );
    tetrahedron_circulator c4 = n4->begin();
    tetrahedron_circulator end4 = n4->end();
    BOOST_CHECK( c4 == end4 );

    BOOST_CHECK( n5->isolated() );
    BOOST_CHECK( n5->size_tetrahedra() == 0 );
    tetrahedron_circulator c5 = n5->begin();
    tetrahedron_circulator end5 = n5->end();
    BOOST_CHECK( c5 == end5 );

    tetrahedron_iterator tt0;
    BOOST_CHECK_THROW( M.tetrahedron(1) , std::out_of_range);
    BOOST_CHECK_NO_THROW( tt0 = M.tetrahedron(0) );
    BOOST_CHECK(tt0 == t0 );
    BOOST_CHECK(tt0->idx() == t0->idx() );

    const_tetrahedron_iterator ct0;
    mesh_type const &  R = M;
    BOOST_CHECK_NO_THROW( ct0 = R.tetrahedron(0) );
    BOOST_CHECK(ct0->idx() == t0->idx() );


    tetrahedron_iterator t1;
    {
      BOOST_CHECK_NO_THROW( t1 = M.insert(0,1,2,4) );
      BOOST_CHECK( M.size_tetrahedra() == 2 );

      BOOST_CHECK( !n0->isolated()  );
      BOOST_CHECK( !n1->isolated()  );
      BOOST_CHECK( !n2->isolated()  );
      BOOST_CHECK( !n3->isolated()  );
      BOOST_CHECK( !n4->isolated()  );
      BOOST_CHECK( n5->isolated()  );

      BOOST_CHECK( n0->size_tetrahedra() == 2  );
      BOOST_CHECK( n1->size_tetrahedra() == 2  );
      BOOST_CHECK( n2->size_tetrahedra() == 2  );
      BOOST_CHECK( n3->size_tetrahedra() == 1  );
      BOOST_CHECK( n4->size_tetrahedra() == 1  );
      BOOST_CHECK( n5->size_tetrahedra() == 0  );

      BOOST_CHECK( t0->owner() == &M  );

      tetrahedron_circulator c0 = n0->begin();
      tetrahedron_circulator end0 = n0->end();
      BOOST_CHECK( c0->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c0);
      BOOST_CHECK( c0->idx() == t1->idx() );
      BOOST_CHECK_NO_THROW( ++c0);
      BOOST_CHECK( c0 == end0 );

      tetrahedron_circulator c1 = n1->begin();
      tetrahedron_circulator end1 = n1->end();
      BOOST_CHECK( c1->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c1);
      BOOST_CHECK( c1->idx() == t1->idx() );
      BOOST_CHECK_NO_THROW( ++c1);
      BOOST_CHECK( c1 == end1 );

      tetrahedron_circulator c2 = n2->begin();
      tetrahedron_circulator end2 = n2->end();
      BOOST_CHECK( c2->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c2);
      BOOST_CHECK( c2->idx() == t1->idx() );
      BOOST_CHECK_NO_THROW( ++c2);
      BOOST_CHECK( c2 == end2 );

      tetrahedron_circulator c3 = n3->begin();
      tetrahedron_circulator end3 = n3->end();
      BOOST_CHECK( c3->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c3);
      BOOST_CHECK( c3 == end3 );

      tetrahedron_circulator c4 = n4->begin();
      tetrahedron_circulator end4 = n4->end();
      BOOST_CHECK( c4->idx() == t1->idx() );
      BOOST_CHECK_NO_THROW( ++c4);
      BOOST_CHECK( c4 == end4 );

      BOOST_CHECK( n5->isolated() );
      BOOST_CHECK( n5->size_tetrahedra() == 0 );
      tetrahedron_circulator c5 = n5->begin();
      tetrahedron_circulator end5 = n5->end();
      BOOST_CHECK( c5 == end5 );
    }
    {
      tetrahedron_iterator t = M.tetrahedron_begin();
      tetrahedron_iterator end = M.tetrahedron_end();
      BOOST_CHECK( t0->idx() == t->idx() );
      BOOST_CHECK( t0 == t               );
      BOOST_CHECK( t!=end                );
      BOOST_CHECK_NO_THROW( ++t );
      BOOST_CHECK( t1->idx() == t->idx() );
      BOOST_CHECK( t1 == t               );
      BOOST_CHECK( t!=end                );
      BOOST_CHECK_NO_THROW( ++t );
      BOOST_CHECK( t==end                );
      BOOST_CHECK_THROW( t->idx() , std::out_of_range );
      BOOST_CHECK_THROW( *t       , std::out_of_range );
      BOOST_CHECK_THROW( ++t      , std::out_of_range );
      BOOST_CHECK_THROW( t->idx() , std::out_of_range );
      BOOST_CHECK_THROW( *t       , std::out_of_range );
    }
    {
      const_tetrahedron_iterator t = M.tetrahedron_begin();
      const_tetrahedron_iterator end = M.tetrahedron_end();
      BOOST_CHECK( t0->idx() == t->idx() );
      BOOST_CHECK( t!=end                );
      BOOST_CHECK_NO_THROW( ++t );
      BOOST_CHECK( t1->idx() == t->idx() );
      BOOST_CHECK( t!=end                );
      BOOST_CHECK_NO_THROW( ++t );
      BOOST_CHECK( t==end                );
      BOOST_CHECK_THROW( t->idx() , std::out_of_range );
      BOOST_CHECK_THROW( *t       , std::out_of_range );
      BOOST_CHECK_THROW( ++t      , std::out_of_range );
      BOOST_CHECK_THROW( t->idx() , std::out_of_range );
      BOOST_CHECK_THROW( *t       , std::out_of_range );
    }



    tetrahedron_iterator f0;
    BOOST_CHECK_NO_THROW( f0 = M.find(n0,n1,n2,n3) );
    BOOST_CHECK( f0==t0);
    tetrahedron_iterator f1; 
    BOOST_CHECK_NO_THROW( f1 = M.find(n0,n1,n2,n4) );
    BOOST_CHECK( f1==t1);

    tetrahedron_iterator f3; 
    BOOST_CHECK_NO_THROW( f3 = M.find(n0,n1,n2,n5) );
    BOOST_CHECK( f3 != t0);
    BOOST_CHECK( f3 != t1);
    BOOST_CHECK( f3 == M.tetrahedron_end() );

    node_iterator dummy; 
    BOOST_CHECK_THROW( f3 = M.find(dummy,dummy,dummy,dummy), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(dummy,dummy,dummy,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(dummy,dummy,n1,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(dummy,n3,n1,n0), std::logic_error );

    BOOST_CHECK_THROW( f3 = M.find(n0,n0,n0,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n1,n0,n0,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n1,n0,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n0,n1,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n0,n0,n1), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n1,n2,n0,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n1,n2,n0), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n0,n1,n2), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n0,n1,n0,n2), std::logic_error );
    BOOST_CHECK_THROW( f3 = M.find(n1,n0,n2,n0), std::logic_error );


    tetrahedron_iterator fake;
    BOOST_CHECK_THROW( M.erase(fake), std::exception );
    BOOST_CHECK_NO_THROW( M.erase(t0) );

    BOOST_CHECK_NO_THROW( t0 = M.tetrahedron(0) );
    BOOST_CHECK_THROW( M.tetrahedron(1), std::out_of_range );
    BOOST_CHECK( M.size_tetrahedra() == 1);

    tetrahedron_iterator t = M.tetrahedron_begin();
    tetrahedron_iterator end = M.tetrahedron_end();
    BOOST_CHECK( t0->idx() == t->idx() );
    BOOST_CHECK( t0 == t               );
    BOOST_CHECK( t!=end                );
    BOOST_CHECK_NO_THROW( ++t );
    BOOST_CHECK( t==end                );
    {
      tetrahedron_circulator c0 = n0->begin();
      tetrahedron_circulator end0 = n0->end();
      BOOST_CHECK( c0->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c0);
      BOOST_CHECK( c0 == end0 );

      tetrahedron_circulator c1 = n1->begin();
      tetrahedron_circulator end1 = n1->end();
      BOOST_CHECK( c1->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c1);
      BOOST_CHECK( c1 == end1 );

      tetrahedron_circulator c2 = n2->begin();
      tetrahedron_circulator end2 = n2->end();
      BOOST_CHECK( c2->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c2);
      BOOST_CHECK( c2 == end2 );

      BOOST_CHECK( n3->isolated() );
      BOOST_CHECK( n3->size_tetrahedra() == 0 );
      tetrahedron_circulator c3 = n3->begin();
      tetrahedron_circulator end3 = n3->end();
      BOOST_CHECK( c3 == end3 );

      tetrahedron_circulator c4 = n4->begin();
      tetrahedron_circulator end4 = n4->end();
      BOOST_CHECK( c4->idx() == t0->idx() );
      BOOST_CHECK_NO_THROW( ++c4);
      BOOST_CHECK( c4 == end4 );

      BOOST_CHECK( n5->isolated() );
      BOOST_CHECK( n5->size_tetrahedra() == 0 );
      tetrahedron_circulator c5 = n5->begin();
      tetrahedron_circulator end5 = n5->end();
      BOOST_CHECK( c5 == end5 );
    }

    // 2007-05-15, Kenny some tests are missing! We should test members:
    // i,j,k,m, has_face, node, local2global, global2local and jkm, ijm,
    // kim, and ikj on the tetrahedron class

  }

}

BOOST_AUTO_TEST_SUITE_END();
