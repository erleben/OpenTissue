//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/mesh/trimesh/trimesh.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
// #include <OpenTissue/utility/gl/gl_draw_mesh.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

void trimesh_compile_test()
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::matrix3x3_type                       matrix3x3_type;
  typedef math_types::real_type                            real_type;


  OpenTissue::trimesh::TriMesh<math_types> mesh;

  typedef OpenTissue::trimesh::TriMesh<> mesh_type;
  mesh_type   m_mesh;

  std::list<mesh_type> m_pieces;

  typedef OpenTissue::polymesh::PolyMesh<> mesh_type2;
  mesh_type2   m_mesh2;

  {
    //--- Test Mesh                //
    //---                          //
    //---    6     5               //
    //---     +---+                //
    //---    /4\1/5\               //
    //---  1+---+0--+ 4            //
    //---    \2/6\3/               //
    //---     +---+                //
    //---    2     3               //
    //---                          //
    //--- v0 = (  0,  0, 0)        //
    //--- v1 = (-10,  0, 0)        //
    //--- v2 = ( -5,-10, 0)        //
    //--- v3 = (  5, 10, 0)        //
    //--- v4 = ( 10,  0, 0)        //
    //--- v5 = (  5, 10, 0)        //
    //--- v6 = ( -5, 10, 0)        //
    //---                          //
    m_mesh.clear();

    mesh_type::vertex_handle v0 = m_mesh.add_vertex(vector3_type(0,0,0));
    mesh_type::vertex_handle v1 = m_mesh.add_vertex(vector3_type(-10,0,0));
    mesh_type::vertex_handle v2 = m_mesh.add_vertex(vector3_type(-5,-10,0));
    mesh_type::vertex_handle v3 = m_mesh.add_vertex(vector3_type(5,-10,0));
    mesh_type::vertex_handle v4 = m_mesh.add_vertex(vector3_type(10,0,0));
    mesh_type::vertex_handle v5 = m_mesh.add_vertex(vector3_type(5,10,0));
    mesh_type::vertex_handle v6 = m_mesh.add_vertex(vector3_type(-5,10,0));
    std::vector<mesh_type::vertex_handle> vhandles(3);

    vhandles[0] = 5;
    vhandles[1] = 6;
    vhandles[2] = 0;
    mesh_type::face_handle f0 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    vhandles[0] = 1;
    vhandles[1] = 2;
    vhandles[2] = 0;
    mesh_type::face_handle f1 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    vhandles[0] = 3;
    vhandles[1] = 4;
    vhandles[2] = 0;
    mesh_type::face_handle f2 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    vhandles[0] = 6;
    vhandles[1] = 1;
    vhandles[2] = 0;
    mesh_type::face_handle f3 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    vhandles[0] = 4;
    vhandles[1] = 5;
    vhandles[2] = 0;
    mesh_type::face_handle f4 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    vhandles[0] = 2;
    vhandles[1] = 3;
    vhandles[2] = 0;
    mesh_type::face_handle f5 = m_mesh.add_face(vhandles.begin(),vhandles.end());

    m_mesh.remove_face(f5);

    m_mesh.remove_face(f4);

    m_mesh.remove_face(f3);

    m_mesh.remove_face(f2);

    m_mesh.remove_face(f1);

    m_mesh.remove_face(f0);

    m_mesh.remove_vertex(v6);

    m_mesh.remove_vertex(v5);

    m_mesh.remove_vertex(v4);

    m_mesh.remove_vertex(v3);

    m_mesh.remove_vertex(v2);

    m_mesh.remove_vertex(v1);

    m_mesh.remove_vertex(v0);

    m_mesh.clear();
  }
  {
    //--- Test Mesh                //
    //---                          //
    //---    6     5               //
    //---     +---+                //
    //---    /4\1/5\               //
    //---  1+---+0--+ 4            //
    //---    \2/6\3/               //
    //---     +---+                //
    //---    2     3               //
    //---                          //
    //--- v0 = (  0,  0, 0)        //
    //--- v1 = (-10,  0, 0)        //
    //--- v2 = ( -5,-10, 0)        //
    //--- v3 = (  5, 10, 0)        //
    //--- v4 = ( 10,  0, 0)        //
    //--- v5 = (  5, 10, 0)        //
    //--- v6 = ( -5, 10, 0)        //
    //---                          //
    m_mesh.clear();

    mesh_type::vertex_handle v0 = m_mesh.add_vertex(vector3_type(0,0,0));
    mesh_type::vertex_handle v1 = m_mesh.add_vertex(vector3_type(-10,0,0));
    mesh_type::vertex_handle v2 = m_mesh.add_vertex(vector3_type(-5,-10,0));
    mesh_type::vertex_handle v3 = m_mesh.add_vertex(vector3_type(5,-10,0));
    mesh_type::vertex_handle v4 = m_mesh.add_vertex(vector3_type(10,0,0));
    mesh_type::vertex_handle v5 = m_mesh.add_vertex(vector3_type(5,10,0));
    mesh_type::vertex_handle v6 = m_mesh.add_vertex(vector3_type(-5,10,0));
    std::vector<mesh_type::vertex_handle> vhandles(3);

    vhandles[0] = 5;
    vhandles[1] = 6;
    vhandles[2] = 0;
    mesh_type::face_handle f0 = m_mesh.add_face(vhandles.begin(),vhandles.end());
    vhandles[0] = 1;
    vhandles[1] = 2;
    vhandles[2] = 0;
    mesh_type::face_handle f1 = m_mesh.add_face(vhandles.begin(),vhandles.end());
    vhandles[0] = 3;
    vhandles[1] = 4;
    vhandles[2] = 0;
    mesh_type::face_handle f2 = m_mesh.add_face(vhandles.begin(),vhandles.end());
    vhandles[0] = 6;
    vhandles[1] = 1;
    vhandles[2] = 0;
    mesh_type::face_handle f3 = m_mesh.add_face(vhandles.begin(),vhandles.end());
    vhandles[0] = 4;
    vhandles[1] = 5;
    vhandles[2] = 0;
    mesh_type::face_handle f4 = m_mesh.add_face(vhandles.begin(),vhandles.end());
    vhandles[0] = 2;
    vhandles[1] = 3;
    vhandles[2] = 0;
    mesh_type::face_handle f5 = m_mesh.add_face(vhandles.begin(),vhandles.end());
  }
  mesh_type tmp(m_mesh);  //--- copy constructor test
  m_mesh.clear();
  m_mesh = tmp;           //--- assignment operator test

  m_mesh.clear();

  OpenTissue::mesh::make_box(1.0,1.0,1.0,m_mesh);

  std::vector<vector3_type> profile;
  profile.push_back(vector3_type(0.0,0.0,0.0));
  profile.push_back(vector3_type(5.0,0.0,5.0));
  profile.push_back(vector3_type(5.0,0.0,10.0));
  profile.push_back(vector3_type(0.0,0.0,15.0));
  OpenTissue::mesh::profile_sweep(profile.begin(),profile.end(),2*OpenTissue::math::detail::pi<real_type>(),32,m_mesh);

  OpenTissue::mesh::make_disk(5.0,2.0,12,4,m_mesh);
  OpenTissue::mesh::make_cylinder(5.0,2.0,12,m_mesh);
  OpenTissue::mesh::make_sphere(5.0,12,5,m_mesh);

  OpenTissue::mesh::translate(m_mesh,vector3_type(1,0,0));
  OpenTissue::mesh::uniform_scale(m_mesh,0.5);
  OpenTissue::mesh::scale(m_mesh,vector3_type(1,1,2));
  OpenTissue::mesh::twist(m_mesh,vector3_type(0,1,0),2.0);
  OpenTissue::mesh::bend(m_mesh,vector3_type(0,1,0),vector3_type(1,0,0),1.0);
  OpenTissue::mesh::spherical_bend(m_mesh,vector3_type(0,0,1),1.0);
  OpenTissue::mesh::make_unit(m_mesh);

  std::vector<vector3_type> points(20);
  for(unsigned int i=0;i<20;++i)
    random(points[i]);
  OpenTissue::mesh::convex_hull(points.begin(),points.end(),m_mesh);
  mesh_type::vertex_iterator end    = m_mesh.vertex_end();
  mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
  for(;vertex!=end;++vertex)
    random(vertex->m_color);

  OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_mesh);

  OpenTissue::mesh::compute_mean_vertex_normals(m_mesh);
  {
    OpenTissue::geometry::Plane<math_types> Q;
    Q.n() = vector3_type(0,0,1);
    Q.w() = 0;

    mesh_type tmp(m_mesh);
    mesh_type tmp2;

    OpenTissue::mesh::plane_clipper(tmp,Q,m_mesh,tmp2);
  }

  {
    mesh_type::vertex_iterator end    = m_mesh.vertex_end();
    mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
    for(;vertex!=end;++vertex)
      random(vertex->m_color);
  }

  OpenTissue::mesh::convert(m_mesh,m_mesh2);
  {
    mesh_type tmp(m_mesh);
    OpenTissue::mesh::flip(tmp,m_mesh);
  }
  {
    OpenTissue::mesh::VolumeIntegrator<mesh_type> integrator(m_mesh);

    real_type V;
    integrator.get_volume(V);

    real_type density=1.0;
    real_type M;
    integrator.get_mass(density,M);

    vector3_type r;
    integrator.get_center_of_mass(density,r);

    matrix3x3_type I;
    integrator.get_inertia_tensor(density,I);
  }
  {
    vector3_type mean;
    matrix3x3_type cov;
    OpenTissue::mesh::compute_surface_covariance(m_mesh,mean,cov);
  }
  {
    std::string data_path = opentissue_path;
    std::string filename = data_path + "/demos/data/obj/teapot.obj";
    OpenTissue::mesh::obj_read(filename,m_mesh);
    {
      mesh_type manifold;
      OpenTissue::mesh::remove_redundant_vertices(m_mesh,manifold);
    }
  }
  OpenTissue::mesh::default_write("test.msh",m_mesh);
  OpenTissue::mesh::tetgen_write("test.poly",m_mesh);
  OpenTissue::mesh::obj_write("test.obj",m_mesh);
  OpenTissue::mesh::vrml_write("test.vrml",m_mesh);
  OpenTissue::mesh::obj_read("test.obj",m_mesh);

  OpenTissue::mesh::make_plane(.25,.25,5,5,m_mesh);

  {
    mesh_type::face_iterator fend = m_mesh.face_end();
    mesh_type::face_iterator face = m_mesh.face_begin();
    for(;face!=fend;++face)
    {
      vector3_type c,n;
      OpenTissue::mesh::compute_face_center(*face,c);
      OpenTissue::trimesh::compute_face_normal(*face,n);
    }
  }

//   OpenTissue::gl::MeshDrawDisplayLists<mesh_type> display_lists(m_mesh);
//   display_lists();
//   OpenTissue::gl::DrawMesh(m_mesh);
//
//   OpenTissue::gl::MeshDrawArray<mesh_type> varray(m_mesh);
//   varray();
}


BOOST_AUTO_TEST_SUITE(opentissue_mesh_trimesh);

BOOST_AUTO_TEST_CASE(compile_testing)
{
  void (*ptr)() = &trimesh_compile_test;
  ptr = 0;
}

BOOST_AUTO_TEST_SUITE_END();
