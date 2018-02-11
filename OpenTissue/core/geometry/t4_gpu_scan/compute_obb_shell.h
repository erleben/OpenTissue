#ifndef OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_COMPUTE_OBB_SHELL_H
#define OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_COMPUTE_OBB_SHELL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_local_triangle_frame.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_angle_weighted_vertex_normals.h>

#include <cassert>

namespace OpenTissue
{
  namespace detail
  {

    /**
    * Simple Tetrahedral Shell Generation Function.
    * This shell function is based on fitting an enclosing obb around
    * each triangular face.
    *
    * @param surface    The surface mesh.
    * @param thickness  The extrusion thickness along surface normals.
    * @param shell      Upon return contains the topology of the generated tetrahedral mesh.
    * @param points     Upon return contains the coordinates of the tetrahedron nodes.
    * @param lut        Upon return contains a tetrahedron to face pointer lookup up table. Example types could be
    *
    *                    typedef std::map<int,face_type *>  lut_container;
    *                    typedef std::vector<face_type *>  lut_container;
    *
    *
    */
    template<typename surface_mesh,typename volume_mesh,typename point_container, typename lut_container>
    inline void compute_obb_shell(
      surface_mesh & surface
      , double const & thickness
      , volume_mesh & shell
      , point_container & points
      , lut_container & lut
      )
    {
      typedef typename volume_mesh::node_iterator              node_iterator;
      typedef typename volume_mesh::tetrahedron_iterator       tetrahedron_iterator;
      typedef typename surface_mesh::face_iterator             face_iterator;
      typedef typename surface_mesh::vertex_iterator           vertex_iterator;
      typedef typename surface_mesh::face_vertex_circulator    face_vertex_circulator;
      typedef typename surface_mesh::face_type                 face_type;

      typedef typename surface_mesh::math_types                math_types;
      typedef typename math_types::vector3_type                vector3_type;

      shell.clear();
      points.clear();
      lut.clear();

      size_t cnt_max_tetrahedra = 5*surface.size_faces();

      lut.resize(cnt_max_tetrahedra);

      OpenTissue::geometry::LocalTriangleFrame<vector3_type> local_frame;

      face_iterator f_end = surface.face_end();
      face_iterator f     = surface.face_begin();
      for(;f!=f_end;++f)
      {
        assert(valency( *f )==3 || !"compute_obb_shell(): Only triangular faces are supported!");
        face_vertex_circulator v(*f);
        vector3_type & p0 = v->m_coord;  ++v;
        vector3_type & p1 = v->m_coord;  ++v;
        vector3_type & p2 = v->m_coord;
        local_frame.init(p0,p1,p2);
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

        points.push_back( c0 );
        points.push_back( c1 );
        points.push_back( c2 );
        points.push_back( c3 );
        points.push_back( c4 );
        points.push_back( c5 );
        points.push_back( c6 );
        points.push_back( c7 );

        node_iterator n0 = shell.insert();
        node_iterator n1 = shell.insert();
        node_iterator n2 = shell.insert();
        node_iterator n3 = shell.insert();
        node_iterator n4 = shell.insert();
        node_iterator n5 = shell.insert();
        node_iterator n6 = shell.insert();
        node_iterator n7 = shell.insert();

        tetrahedron_iterator t0 = shell.insert( n0, n4, n5, n6 );
        tetrahedron_iterator t1 = shell.insert( n0, n5, n1, n3 );
        tetrahedron_iterator t2 = shell.insert( n6, n2, n3, n0 );
        tetrahedron_iterator t3 = shell.insert( n7, n6, n3, n5 );
        tetrahedron_iterator t4 = shell.insert( n6, n5, n0, n3 );

        lut[t0->idx()] = &(*f);
        lut[t1->idx()] = &(*f);
        lut[t2->idx()] = &(*f);
        lut[t3->idx()] = &(*f);
        lut[t4->idx()] = &(*f);
      }
    }

  } // namespace detail

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_T4_GPU_SCAN_T4_GPU_SCAN_COMPUTE_OBB_SHELL_H
#endif
