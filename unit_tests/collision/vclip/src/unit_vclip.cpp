//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/vclip/vclip.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_make_box.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;



template<typename mesh_type, typename coordsys_type,typename real_type>
void vclip_test_all_feature_pairs(mesh_type & A, mesh_type & B,coordsys_type const & AtoB,coordsys_type const & BtoA,real_type const & test_distance)
{ 
  typename coordsys_type::vector3_type p_a;                   
  typename coordsys_type::vector3_type p_b;                   

  unsigned int max_iterations = 500;

  OpenTissue::vclip::VClip vclip;

  real_type tol = 0.1;
  {
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb     = B.vertex_begin();
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb_end = B.vertex_end();
    for(;vb!=vb_end;++vb)
    {
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va     = A.vertex_begin();
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va_end = A.vertex_end();
      for(;va!=va_end;++va)
      {
        OpenTissue::vclip::Feature * seed_a = &( * va );
        OpenTissue::vclip::Feature * seed_b = &( * vb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb     = B.vertex_begin();
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb_end = B.vertex_end();
    for(;vb!=vb_end;++vb)
    {
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea     = A.halfedge_begin();
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea_end = A.halfedge_end();
      for(;ea!=ea_end;++ea)
      {
        OpenTissue::vclip::Feature * seed_a = &( * ea );
        OpenTissue::vclip::Feature * seed_b = &( * vb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb     = B.vertex_begin();
    OpenTissue::vclip::vclip_mesh_type::vertex_iterator vb_end = B.vertex_end();
    for(;vb!=vb_end;++vb)
    {
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa     = A.face_begin();
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa_end = A.face_end();
      for(;fa!=fa_end;++fa)
      {
        OpenTissue::vclip::Feature * seed_a = &( * fa );
        OpenTissue::vclip::Feature * seed_b = &( * vb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb     = B.halfedge_begin();
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb_end = B.halfedge_end();
    for(;eb!=eb_end;++eb)
    {
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va     = A.vertex_begin();
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va_end = A.vertex_end();
      for(;va!=va_end;++va)
      {
        OpenTissue::vclip::Feature * seed_a = &( * va );
        OpenTissue::vclip::Feature * seed_b = &( * eb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb     = B.halfedge_begin();
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb_end = B.halfedge_end();
    for(;eb!=eb_end;++eb)
    {
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea     = A.halfedge_begin();
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea_end = A.halfedge_end();
      for(;ea!=ea_end;++ea)
      {
        OpenTissue::vclip::Feature * seed_a = &( * ea );
        OpenTissue::vclip::Feature * seed_b = &( * eb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb     = B.halfedge_begin();
    OpenTissue::vclip::vclip_mesh_type::halfedge_iterator eb_end = B.halfedge_end();
    for(;eb!=eb_end;++eb)
    {
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa     = A.face_begin();
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa_end = A.face_end();
      for(;fa!=fa_end;++fa)
      {
        OpenTissue::vclip::Feature * seed_a = &( * fa );
        OpenTissue::vclip::Feature * seed_b = &( * eb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb     = B.face_begin();
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb_end = B.face_end();
    for(;fb!=fb_end;++fb)
    {
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va     = A.vertex_begin();
      OpenTissue::vclip::vclip_mesh_type::vertex_iterator va_end = A.vertex_end();
      for(;va!=va_end;++va)
      {
        OpenTissue::vclip::Feature * seed_a = &( * va );
        OpenTissue::vclip::Feature * seed_b = &( * fb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance>0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance<0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb     = B.face_begin();
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb_end = B.face_end();
    for(;fb!=fb_end;++fb)
    {
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea     = A.halfedge_begin();
      OpenTissue::vclip::vclip_mesh_type::halfedge_iterator ea_end = A.halfedge_end();
      for(;ea!=ea_end;++ea)
      {
        OpenTissue::vclip::Feature * seed_a = &( * ea );
        OpenTissue::vclip::Feature * seed_b = &( * fb );
        real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
        if(test_distance > 0)
          BOOST_CHECK_CLOSE( distance, test_distance, tol );
        else if(test_distance < 0)
          BOOST_CHECK( distance < 0 );
        else
          BOOST_CHECK( fabs(distance) < 10e-7 );
      }
    }
  }
  {
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb     = B.face_begin();
    OpenTissue::vclip::vclip_mesh_type::face_iterator fb_end = B.face_end();
    for(;fb!=fb_end;++fb)
    {
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa     = A.face_begin();
      OpenTissue::vclip::vclip_mesh_type::face_iterator fa_end = A.face_end();
      for(;fa!=fa_end;++fa)
      {
        OpenTissue::vclip::Feature * seed_a = &( * fa );
        OpenTissue::vclip::Feature * seed_b = &( * fb );
        BOOST_CHECK_THROW( vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations),  std::invalid_argument);
      }
    }
  }
}


BOOST_AUTO_TEST_SUITE(opentissue_collision_vclip);

BOOST_AUTO_TEST_CASE(face_aligned_separated_boxes)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::real_type                            real_type;

  OpenTissue::vclip::vclip_mesh_type A;
  OpenTissue::vclip::vclip_mesh_type B;

  OpenTissue::polymesh::PolyMesh<> tmp;
  OpenTissue::mesh::make_box(1.0,1.0,1.0, tmp);
  OpenTissue::mesh::make_box(1.0,1.0,1.0, tmp);

  OpenTissue::vclip::convert(tmp,A);
  OpenTissue::vclip::convert(tmp,B);

  coordsys_type  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.0), quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.0), quaternion_type());      
  coordsys_type AtoB = model_update(Awcs,Bwcs);
  coordsys_type BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);

  Awcs = coordsys_type( vector3_type( 1.0,0.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type(-1.0,0.0,0.0), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);

  Awcs = coordsys_type( vector3_type( 0.0, 1.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0,-1.0,0.0), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);

  Awcs = coordsys_type( vector3_type( 0.0,-1.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0, 1.0,0.0), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);

  Awcs = coordsys_type( vector3_type( 0.0, 0.0,  1.0 ), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0, 0.0, -1.0 ), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);

  Awcs = coordsys_type( vector3_type( 0.0, 0.0, -1.0 ), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0, 0.0,  1.0 ), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);
}

BOOST_AUTO_TEST_CASE(non_aligned_cases)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::value_traits                         value_traits;
  typedef math_types::real_type                            real_type;

  OpenTissue::vclip::vclip_mesh_type A;
  OpenTissue::vclip::vclip_mesh_type B;

  OpenTissue::polymesh::PolyMesh<> tmp;
  OpenTissue::mesh::make_box(1.0,1.0,1.0, tmp);
  OpenTissue::mesh::make_box(1.0,1.0,1.0, tmp);

  OpenTissue::vclip::convert(tmp,A);
  OpenTissue::vclip::convert(tmp,B);

  real_type sqrt_half = std::sqrt(0.5);

  real_type test_distance = 1.0 - (sqrt_half - 0.5);

  quaternion_type q;
  q.Rz( value_traits::pi()/4.0 );

  coordsys_type  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.0), quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.0), q);      
  coordsys_type AtoB = model_update(Awcs,Bwcs);
  coordsys_type BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,test_distance);

  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 1.0,0.1,0.1), q);      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,test_distance);


  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.5), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.0), q);      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,test_distance);

  Awcs = coordsys_type( vector3_type(-1.0,0.0,-0.5), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.5), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA,1.0);
}

BOOST_AUTO_TEST_CASE(penetrating_cases)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::value_traits                         value_traits;
  typedef math_types::real_type                            real_type;

  OpenTissue::vclip::vclip_mesh_type A;
  OpenTissue::vclip::vclip_mesh_type B;

  OpenTissue::polymesh::PolyMesh<> tmp;
  OpenTissue::mesh::make_box(1.0,1.0,1.0, tmp);
  OpenTissue::vclip::convert(tmp,A);

  OpenTissue::mesh::make_box(1.0,1.0, 1.0, tmp);
  OpenTissue::vclip::convert(tmp,B);

  coordsys_type  Awcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());   
  coordsys_type  Bwcs = coordsys_type( vector3_type( 0.25,0.25,0.25), quaternion_type());      
  coordsys_type AtoB = model_update(Awcs,Bwcs);
  coordsys_type BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA, -1.0);


  OpenTissue::mesh::make_box(0.9,0.9, 0.9, tmp);
  OpenTissue::vclip::convert(tmp,B);

  Awcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());   
  Bwcs = coordsys_type( vector3_type( 0.0,0.0,0.0), quaternion_type());      
  AtoB = model_update(Awcs,Bwcs);
  BtoA = inverse(AtoB);
  vclip_test_all_feature_pairs(A,B,AtoB,BtoA, -1.0);
}

BOOST_AUTO_TEST_SUITE_END();
