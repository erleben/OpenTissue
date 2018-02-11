#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MAKE_SPHERE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MAKE_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_angle_weighted_vertex_normals.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_subdivide.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Sphere Generation.
    *
    * @param radius             The size of the radius of the sphere.
    * @param n                  The number of sub-divisions (if use_tetrahedron is on then #faces = 4^n otherwise #faces = 20^n).
    * @param mesh               Upon return this argument holds the generated sphere.
    * @param use_tetrahedorn    Controls whether sphere is genereted by subdivision of tetrahedron or icosahedron. Default value is false.
    *
    * @return        If succesful then true otherwise false.
    */
    template<typename mesh_type>
    bool make_sphere(
      typename mesh_type::math_types::real_type const & radius
      , unsigned int n
      , mesh_type & mesh
      , bool use_tetrahedron = false
      )
    {
      typedef typename mesh_type::math_types        math_types;
      typedef typename math_types::value_traits     value_traits;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename math_types::real_type        real_type;
      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      typedef typename mesh_type::vertex_handle     vertex_handle;

      mesh.clear();

      if(use_tetrahedron)
      {
        //--- generate from tetrahedron

        vector3_type coord0( -1, -1, -1 );
        vector3_type coord1(  1,  1, -1 );
        vector3_type coord2(  1, -1,  1 );
        vector3_type coord3( -1,  1,  1 );

        //vector3_type coord0(               0,                0,        1  );
        //vector3_type coord1(               0,  2*sqrt(2.0)/3.0,  -1.0/3.0 );
        //vector3_type coord2(  -sqrt(6.0)/3.0,   -sqrt(2.0)/3.0,  -1.0/3.0 );
        //vector3_type coord3(   sqrt(6.0)/3.0,   -sqrt(2.0)/3.0,  -1.0/3.0 );

        vertex_handle p0 = mesh.add_vertex(coord0);
        vertex_handle p1 = mesh.add_vertex(coord1);
        vertex_handle p2 = mesh.add_vertex(coord2);
        vertex_handle p3 = mesh.add_vertex(coord3);
        mesh.add_face(p0,p1,p2);
        mesh.add_face(p1,p0,p3);
        mesh.add_face(p2,p1,p3);
        mesh.add_face(p0,p2,p3);

      }
      else
      {
        //--- generate from icosahedron
        real_type const X = 0.525731112119133606;
        real_type const Z = 0.850650808352039932;

        vector3_type coord0(-X, 0.0, Z);
        vector3_type coord1(X, 0.0, Z);
        vector3_type coord2(-X, 0.0, -Z);
        vector3_type coord3(X, 0.0, -Z);
        vector3_type coord4(0.0, Z, X);
        vector3_type coord5(0.0, Z, -X);
        vector3_type coord6(0.0, -Z, X);
        vector3_type coord7(0.0, -Z, -X);
        vector3_type coord8(Z, X, 0.0);
        vector3_type coord9(-Z, X, 0.0);
        vector3_type coord10(Z, -X, 0.0);
        vector3_type coord11(-Z, -X, 0.0);

        vertex_handle p0  = mesh.add_vertex( coord0  );
        vertex_handle p1  = mesh.add_vertex( coord1  );
        vertex_handle p2  = mesh.add_vertex( coord2  );
        vertex_handle p3  = mesh.add_vertex( coord3  );
        vertex_handle p4  = mesh.add_vertex( coord4  );
        vertex_handle p5  = mesh.add_vertex( coord5  );
        vertex_handle p6  = mesh.add_vertex( coord6  );
        vertex_handle p7  = mesh.add_vertex( coord7  );
        vertex_handle p8  = mesh.add_vertex( coord8  );
        vertex_handle p9  = mesh.add_vertex( coord9  );
        vertex_handle p10 = mesh.add_vertex( coord10 );
        vertex_handle p11 = mesh.add_vertex( coord11 );

        mesh.add_face(  p4,  p0,  p1 );
        mesh.add_face(  p9,  p0,  p4 );
        mesh.add_face(  p5,  p9,  p4 );
        mesh.add_face(  p5,  p4,  p8 );
        mesh.add_face(  p8,  p4,  p1 );
        mesh.add_face(  p10, p8,  p1 );
        mesh.add_face(  p3,  p8, p10 );
        mesh.add_face(  p3,  p5,  p8 );
        mesh.add_face(  p2,  p5,  p3 );
        mesh.add_face(  p7,  p2,  p3 );
        mesh.add_face(  p10, p7,  p3 );
        mesh.add_face(  p6,  p7, p10 );
        mesh.add_face(  p11, p7,  p6 );
        mesh.add_face( p0,  p11,  p6 );
        mesh.add_face(  p1,  p0,  p6 );
        mesh.add_face(  p1,  p6, p10 );
        mesh.add_face(  p0,  p9, p11 );
        mesh.add_face(  p11, p9,  p2 );
        mesh.add_face(  p2,  p9,  p5 );
        mesh.add_face(  p2,  p7, p11 );
      }

      //--- force vertex coords onto sphere
      for(vertex_iterator v = mesh.vertex_begin();v!=mesh.vertex_end();++v)
        v->m_coord = radius*unit(v->m_coord);
      //--- recursively subdivide
      for(unsigned int i=0;i<n;++i)
      {
        subdivide( mesh, 0.0);
        for(vertex_iterator v = mesh.vertex_begin();v!=mesh.vertex_end();++v)
          v->m_coord = radius*unit(v->m_coord);
      }

      mesh::compute_angle_weighted_vertex_normals(mesh);

      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_MAKE_SPHERE_H
#endif
