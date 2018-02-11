#ifndef OPENTISSUE_CORE_GEOMETRY_T4_CPU_SCAN_T4_CPU_SCAN_H
#define OPENTISSUE_CORE_GEOMETRY_T4_CPU_SCAN_T4_CPU_SCAN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_tetrahedron.h>
#include <OpenTissue/core/geometry/geometry_compute_tetrahedron_aabb.h>
#include <OpenTissue/core/geometry/geometry_tetrahedron_z_slicer.h>
#include <OpenTissue/core/geometry/scan_conversion/scan_conversion_fragment_iterator.h>
#include <OpenTissue/core/geometry/geometry_local_triangle_frame.h>
#include <OpenTissue/core/geometry/geometry_compute_distance_to_triangle.h>
#include <OpenTissue/core/geometry/geometry_compute_signed_distance_to_triangle.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_angle_weighted_vertex_normals.h>

#include <boost/cast.hpp>

#include <cassert>
#include <cmath>


namespace OpenTissue
{

  struct t4_cpu_unsigned {};
  struct t4_cpu_signed {};

  /**
  * CPU Signed Distance Field Computation.
  *
  * @param surface             A surface mesh, used to generate the
  *                         distance field from.
  * @param thickness        An offset parameter, describing the extent
  *                         of the generated distance field along vertex normals
  *                         (not the same as face normals!).
  * @param phi              Upon return holds the generated distance field.
  *
  */
  template<typename surface_mesh, typename grid_type>
  inline void t4_cpu_scan(
      surface_mesh /*const*/ & surface
    , double const & thickness
    , grid_type & phi
    , t4_cpu_signed const & /*tag*/
    )
  {
    using std::ceil;
    using std::floor;
    using std::fabs;

    assert(thickness>0 || "t4_cpu_scan(): thickness was non-positive");

    typedef typename surface_mesh::math_types                math_types;
    typedef typename math_types::vector3_type                vector3_type;
    typedef typename math_types::real_type                   real_type;
    typedef typename surface_mesh::face_iterator             face_iterator;
    typedef typename surface_mesh::face_halfedge_circulator  face_halfedge_circulator;
    typedef typename grid_type::value_type                    value_type;

    typedef          OpenTissue::geometry::ZTetrahedronSlicer<vector3_type>               slicer_type;
    typedef          OpenTissue::geometry::LocalTriangleFrame<vector3_type>               local_frame_type;
    typedef          OpenTissue::geometry::Tetrahedron<math_types>                        tetrahedron_type;
    typedef          OpenTissue::scan_conversion::FragmentIterator<vector3_type>          fragment_iterator;


    local_frame_type  local_frame;
    tetrahedron_type  tetrahedra[5];
    vector3_type      vertices[4];

    real_type const min_x = phi.min_coord(0);
    real_type const min_y = phi.min_coord(1);
    real_type const min_z = phi.min_coord(2);
    real_type const dx = phi.dx();   //--- space between grid nodes along x-axis
    real_type const dy = phi.dy();   //--- space between grid nodes along y-axis
    real_type const dz = phi.dz();   //--- space between grid nodes along z-axis

    mesh::compute_angle_weighted_vertex_normals(surface);

    face_iterator end    = surface.face_end();
    face_iterator face   = surface.face_begin();
    for(;face!=end;++face)
    {
      assert(valency( *face )==3 || !"t4_cpu_scan(): Only triangular faces are supported!");

      vector3_type nf;
      vector3_type ne_i;
      vector3_type ne_j;
      vector3_type ne_k;

      ne_i.clear();
      ne_j.clear();
      ne_k.clear();

      // get face normal
      compute_face_normal( *face ,nf);

      face_halfedge_circulator h(*face);
      vector3_type const & p_i  = h->get_origin_iterator()->m_coord;
      vector3_type const & nv_i  = h->get_origin_iterator()->m_normal;
      if(!h->get_twin_iterator()->get_face_handle().is_null())
        compute_face_normal( *(h->get_twin_iterator()->get_face_iterator()),ne_k);
      ++h;

      vector3_type const & p_j  = h->get_origin_iterator()->m_coord;  
      vector3_type const & nv_j  = h->get_origin_iterator()->m_normal;
      if(!h->get_twin_iterator()->get_face_handle().is_null())
        compute_face_normal( *(h->get_twin_iterator()->get_face_iterator()),ne_i);
      ++h;

      vector3_type const & p_k  = h->get_origin_iterator()->m_coord;
      vector3_type const & nv_k  = h->get_origin_iterator()->m_normal;
      if(!h->get_twin_iterator()->get_face_handle().is_null())
        compute_face_normal( *(h->get_twin_iterator()->get_face_iterator()),ne_j);

      //--- compute edge pseudo normals
      ne_i = unit(nf + ne_i);
      ne_j = unit(nf + ne_j);
      ne_k = unit(nf + ne_k);

      local_frame.init( p_i, p_j, p_k);
      // For each OBB, the 8 corners are numerated as:
      //
      //     2*-----*3        y
      //     /|    /|         ^
      //    / |   / |         |
      //  6*-----*7 |         |
      //   | 0*--|--*1        +--->x
      //   | /   | /         /
      //   |/    |/        |/_
      //  4*-----5         z
      //
      vector3_type c0 = local_frame.v0() - local_frame.unit_a()*thickness - local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c1 = local_frame.v1() + local_frame.unit_a()*thickness - local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c2 = local_frame.v0() - local_frame.unit_a()*thickness + local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c3 = local_frame.v1() + local_frame.unit_a()*thickness + local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c4 = local_frame.v0() - local_frame.unit_a()*thickness - local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c5 = local_frame.v1() + local_frame.unit_a()*thickness - local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c6 = local_frame.v0() - local_frame.unit_a()*thickness + local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c7 = local_frame.v1() + local_frame.unit_a()*thickness + local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());

      tetrahedra[0].set( c0, c4, c5, c6 );
      tetrahedra[1].set( c0, c5, c1, c3 );
      tetrahedra[2].set( c6, c2, c3, c0 );
      tetrahedra[3].set( c7, c6, c3, c5 );
      tetrahedra[4].set( c6, c5, c0, c3 );

      for(int n=0;n<5;++n)
      {
        slicer_type slicer( 
            tetrahedra[n].p0()
          , tetrahedra[n].p1()
          , tetrahedra[n].p2()
          , tetrahedra[n].p3() 
          );

        vector3_type min_coord; 
        vector3_type max_coord; 
        OpenTissue::geometry::compute_tetrahedron_aabb(tetrahedra[n],min_coord,max_coord);

        //--- Determine the z value to start slicing (min) and the z-value to stop slicing (max)
        int k_min = boost::numeric_cast<int>(ceil(  (min_coord(2) - min_z)/dz ));
        int k_max = boost::numeric_cast<int>(floor( (max_coord(2) - min_z)/dz ));

        for(int k = k_min;k<=k_max;++k)
        {
          real_type z = k*dz + min_z;

          int number_of_vertices = slicer.intersect(z,vertices);

          if(number_of_vertices<3)
            continue;

          //--- transform vertices into ``screen space'', that is make a parallel projection onto the I and J axes of phi
          //--- In openGL one would do it like
          //--- real_type left   = min_x - dx/2;
          //--- real_type right  = max_x + dx/2;
          //--- real_type bottom = min_y - dy/2;
          //--- real_type top    = max_y + dy/2;
          //--- gluOrtho2D( left, right, bottom, top);
          //--- glViewport(0,0,phi.I(),phi.J());
          //---
          //--- However, we can just do it straightforward,...
          for(int m=0;m<number_of_vertices;++m)
          {
            vertices[m](0) = (vertices[m](0) - min_x)/dx;
            vertices[m](1) = (vertices[m](1) - min_y)/dy;
          }

          int number_of_triangles = (number_of_vertices==3)? 1 : 2;

          for(int c=1;c<=number_of_triangles;++c)
          {
            fragment_iterator pixel(vertices[0],vertices[c],vertices[c+1]);
            while(pixel())
            {
              int i = pixel.x();
              int j = pixel.y();
              //--- compute corresponding world coordinates of pixel
              vector3_type p ( 
                  i*dx + min_x
                , j*dy + min_y
                , k*dz + min_z
                );

              //--- compute closest signed distance to triangle
              //real_type value = OpenTissue::geometry::compute_distance_to_triangle(p,p_i,p_j,p_k);
              real_type value = OpenTissue::geometry::compute_signed_distance_to_triangle( p, p_i, p_j, p_k, nv_i, nv_j, nv_k, ne_i, ne_j, ne_k);

              // store signed distance into phi at location  (i,j,k)
              if(
                ( fabs(value) <= thickness ) 
                && 
                ( fabs(value) < fabs( phi(i,j,k) ) ) 
                )
                phi(i,j,k) = boost::numeric_cast<value_type>( value );
              ++pixel;
            }
          }

        }
      }
    }
  }


  /**
  * CPU Unsgined Distance Field Computation.
  *
  * @param surface             A surface mesh, used to generate the
  *                         distance field from.
  * @param thickness        An offset parameter, describing the extent
  *                         of the generated distance field along vertex normals
  *                         (not the same as face normals!).
  * @param phi              Upon return holds the generated distance field.
  *
  */
  template<typename surface_mesh, typename grid_type>
  inline void t4_cpu_scan(
      surface_mesh /*const*/ & surface
    , double const & thickness
    , grid_type & phi
    , t4_cpu_unsigned const & /*tag*/
    )
  {
    using std::ceil;
    using std::floor;
    using std::fabs;

    assert(thickness>0 || "t4_cpu_scan(): thickness was non-positive");

    typedef typename surface_mesh::math_types                math_types;
    typedef typename math_types::vector3_type                vector3_type;
    typedef typename math_types::real_type                   real_type;
    typedef typename surface_mesh::face_iterator             face_iterator;
    typedef typename surface_mesh::face_halfedge_circulator  face_halfedge_circulator;
    typedef typename grid_type::value_type                    value_type;

    typedef          OpenTissue::geometry::ZTetrahedronSlicer<vector3_type>        slicer_type;
    typedef          OpenTissue::geometry::LocalTriangleFrame<vector3_type>        local_frame_type;
    typedef          OpenTissue::geometry::Tetrahedron<math_types>                 tetrahedron_type;
    typedef          OpenTissue::scan_conversion::FragmentIterator<vector3_type>   fragment_iterator;

    local_frame_type  local_frame;
    tetrahedron_type  tetrahedra[5];
    vector3_type      vertices[4];

    real_type const min_x = phi.min_coord(0);
    real_type const min_y = phi.min_coord(1);
    real_type const min_z = phi.min_coord(2);
    real_type const max_x = phi.max_coord(0);
    real_type const max_y = phi.max_coord(1);
    real_type const max_z = phi.max_coord(2);
    real_type const dx = phi.dx();   //--- space between grid nodes along x-axis
    real_type const dy = phi.dy();   //--- space between grid nodes along y-axis
    real_type const dz = phi.dz();   //--- space between grid nodes along z-axis

    mesh::compute_angle_weighted_vertex_normals(surface);

    face_iterator end    = surface.face_end();
    face_iterator face   = surface.face_begin();
    for(;face!=end;++face)
    {
      assert(valency( *face )==3 || !"t4_cpu_scan(): Only triangular faces are supported!");

      face_halfedge_circulator h(*face);
      vector3_type const & p_i  = h->get_origin_iterator()->m_coord;
      ++h;
      vector3_type const & p_j  = h->get_origin_iterator()->m_coord;  
      ++h;
      vector3_type const & p_k  = h->get_origin_iterator()->m_coord;

      local_frame.init( p_i, p_j, p_k);
      // For each OBB, the 8 corners are numerated as:
      //
      //     2*-----*3        y
      //     /|    /|         ^
      //    / |   / |         |
      //  6*-----*7 |         |
      //   | 0*--|--*1        +--->x
      //   | /   | /         /
      //   |/    |/        |/_
      //  4*-----5         z
      //
      vector3_type c0 = local_frame.v0() - local_frame.unit_a()*thickness - local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c1 = local_frame.v1() + local_frame.unit_a()*thickness - local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c2 = local_frame.v0() - local_frame.unit_a()*thickness + local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c3 = local_frame.v1() + local_frame.unit_a()*thickness + local_frame.n()*thickness - local_frame.unit_h()*thickness;
      vector3_type c4 = local_frame.v0() - local_frame.unit_a()*thickness - local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c5 = local_frame.v1() + local_frame.unit_a()*thickness - local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c6 = local_frame.v0() - local_frame.unit_a()*thickness + local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());
      vector3_type c7 = local_frame.v1() + local_frame.unit_a()*thickness + local_frame.n()*thickness + local_frame.unit_h()*(thickness+local_frame.h());

      tetrahedra[0].set( c0, c4, c5, c6 );
      tetrahedra[1].set( c0, c5, c1, c3 );
      tetrahedra[2].set( c6, c2, c3, c0 );
      tetrahedra[3].set( c7, c6, c3, c5 );
      tetrahedra[4].set( c6, c5, c0, c3 );

      for(int n=0;n<5;++n)
      {
        slicer_type slicer( 
            tetrahedra[n].p0()
          , tetrahedra[n].p1()
          , tetrahedra[n].p2()
          , tetrahedra[n].p3() 
          );

        vector3_type min_coord; 
        vector3_type max_coord; 
        compute_tetrahedron_aabb(tetrahedra[n],min_coord,max_coord);

        //--- Determine the z value to start slicing (min) and the z-value to stop slicing (max)
        int k_min = boost::numeric_cast<int>(ceil(  (min_coord(2) - min_z)/dz ));
        int k_max = boost::numeric_cast<int>(floor( (max_coord(2) - min_z)/dz ));

        for(int k = k_min;k<=k_max;++k)
        {
          real_type z = k*dz + min_z;

          int number_of_vertices = slicer.intersect(z,vertices);

          if(number_of_vertices<3)
            continue;

          //--- transform vertices into ``screen space'', that is make a parallel projection onto the I and J axes of phi
          //--- In openGL one would do it like
          //--- real_type left   = min_x - dx/2;
          //--- real_type right  = max_x + dx/2;
          //--- real_type bottom = min_y - dy/2;
          //--- real_type top    = max_y + dy/2;
          //--- gluOrtho2D( left, right, bottom, top);
          //--- glViewport(0,0,phi.I(),phi.J());
          //---
          //--- However, we can just do it straightforward,...
          for(int m=0;m<number_of_vertices;++m)
          {
            vertices[m](0) = (vertices[m](0) - min_x)/dx;
            vertices[m](1) = (vertices[m](1) - min_y)/dy;
          }

          int number_of_triangles = (number_of_vertices==3)? 1 : 2;

          for(int c=1;c<=number_of_triangles;++c)
          {
            fragment_iterator pixel(vertices[0],vertices[c],vertices[c+1]);
            while(pixel())
            {
              int i = pixel.x();
              int j = pixel.y();
              //--- compute corresponding world coordinates of pixel
              vector3_type p ( 
                  i*dx + min_x
                , j*dy + min_y
                , k*dz + min_z
                );

              //--- compute closest signed distance to triangle
              real_type value = OpenTissue::geometry::compute_distance_to_triangle(p,p_i,p_j,p_k);

              // store signed distance into phi at location  (i,j,k)
              if(
                ( value <= thickness ) && ( value < phi(i,j,k) ) 
                )
                  phi(i,j,k) = boost::numeric_cast<value_type>( value );
              ++pixel;
            }
          }

        }
      }
    }
  }

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_T4_CPU_SCAN_T4_CPU_SCAN_H
#endif
