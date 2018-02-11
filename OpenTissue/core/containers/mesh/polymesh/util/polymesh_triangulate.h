#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_TRIANGULATE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_TRIANGULATE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_circumscribed_sphere.h>
#include <OpenTissue/core/containers/containers_heap.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_dihedral_angle.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_edge_flip.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/math/math_constants.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * PolyMesh Triangulation Utility Class.
    */
    template< typename mesh_type >
    class PolyMeshTriangulate
    {
    protected:

      typedef typename mesh_type::vertex_handle        vertex_handle;
      typedef typename mesh_type::math_types           math_types;
      typedef typename math_types::vector3_type        vector3_type;
      typedef typename math_types::real_type           real_type;
      typedef typename mesh_type::edge_handle          edge_handle;
      typedef geometry::Sphere<math_types>             sphere_type;

    protected:

      typedef OpenTissue::containers::Heap<edge_handle,real_type>  heap_type;

      heap_type  m_heap;  ///< Priority heap used to store edge candidates for edge-flips.

    public:

      PolyMeshTriangulate()
        : m_heap()
      {}


    protected:

      /**
      * Compute Minimum Angle.
      * This method computes the minimum ``vertex'' angle of two neighboring
      * triangles.
      *
      * The first triangle is formed by the vertices (v0,v1,v2) and the second
      * triangle is formed by the vertices (v0,v3,v1). Thus the edge given by
      * (v0,v1) is shared between the two triangles.
      *
      * @param v0  The coordinates of the first vertex.
      * @param v1  The coordinates of the second vertex.
      * @param v2  The coordinates of the third vertex.
      * @param v3  The coordinates of the fourth vertex.
      *
      * @return    The minimum vertex angle.
      */
      real_type compute_minimum_angle(
        vector3_type const & v0
        , vector3_type const & v1
        , vector3_type const & v2
        , vector3_type const & v3
        )
      {
        using std::acos;
        using std::min;

        vector3_type e20 = unit(v2-v0);
        vector3_type e10 = unit(v1-v0);
        vector3_type e30 = unit(v3-v0);
        vector3_type e21 = unit(v2-v1);
        vector3_type e32 = unit(v3-v2);

        real_type a0 = acos( e10*e20 );
        real_type a1 = acos( -e10*e21 );
        real_type a2 = math::detail::pi<real_type>() - a0 - a1;

        real_type b0 = acos( -e20*e32 );
        real_type b1 = acos(  e32*e30 );
        real_type b2 = math::detail::pi<real_type>() - b0 - b1;

        real_type min_a = min( a0 , min ( a1 , a2 ) );
        real_type min_b = min( b0 , min ( b1 , b2 ) );

        real_type min_angle = min (min_a, min_b);
        return min_angle;
      }

      /**
      * Compute Priority.
      * This method computes a priority measure, indicating how
      * much the minimum vertex angle of two neighboring
      * triangles are maximized by performaing an edge flip
      * of the shared edge.
      *
      * The first triangle is formed by the vertices (v0,v1,v2) and the second
      * triangle is formed by the vertices (v0,v3,v1). Thus the edge given by
      * (v0,v1) is shared between the two triangles.
      *
      * @param v0  The coordinates of the first vertex.
      * @param v1  The coordinates of the second vertex.
      * @param v2  The coordinates of the third vertex.
      * @param v3  The coordinates of the fourth vertex.
      *
      * @return    The priority value.
      */
      real_type compute_priority(
        vector3_type const & v0
        , vector3_type const & v1
        , vector3_type const & v2
        , vector3_type const & v3
        )
      {
        using std::min;
        using std::fabs;

        sphere_type S0,S1;
        OpenTissue::geometry::compute_circumscribed_sphere( v0, v1, v2, S0);
        OpenTissue::geometry::compute_circumscribed_sphere( v0, v2, v3, S1);
        real_type eps0  = 0.001*S0.radius();
        real_type d0 = S0.signed_distance(v3);
        real_type eps1  = 0.001*S1.radius();
        real_type d1 = S1.signed_distance(v1);
        bool flip = (d0 < -eps0) || (d1 < -eps1);

        if(!flip)
          return real_type();

        real_type i = compute_minimum_angle(v0,v1,v2,v3);
        real_type j = compute_minimum_angle(v1,v2,v3,v0);
        real_type  rel = fabs(j-i) / fabs(i);
        return rel;
      }

      /**
      * Update Priority.
      * This method updates the priorites of all edges that are
      * affected by an flipped edge, specified by its two end
      * vertices A and B.
      *
      * @param A  A handle to the first vertex.
      * @param B  A handle to the second vertex.
      */
      void update_priority( vertex_handle A, vertex_handle B, mesh_type & mesh )
      {
        edge_handle Eh = mesh.find_edge_handle(A,B);
        if(Eh.is_null())
        {
          std::cerr << "PolyMeshTriangulate::update_priority(): Argh edge handle was null?" << std::endl;
          return;
        }

        typename heap_type::heap_iterator H;
        try
        {
          H = m_heap.get_heap_iterator(Eh);
        }catch(std::invalid_argument e)
        {
          return;
        }

        typename mesh_type::edge_iterator     E = mesh.get_edge_iterator(Eh);
        typename mesh_type::halfedge_iterator e = E->get_halfedge0_iterator();

        vector3_type & v0 = e->get_destination_iterator()->m_coord;
        vector3_type & v1 = e->get_next_iterator()->get_destination_iterator()->m_coord;
        vector3_type & v2 = e->get_origin_iterator()->m_coord;
        vector3_type & v3 = e->get_twin_iterator()->get_next_iterator()->get_destination_iterator()->m_coord;

        H->priority() = compute_priority(v0,v1,v2,v3);
        m_heap.heapify(H);
      }

    public:

      /**
      * Re-triangulate Mesh.
      * This method performs an iterative re-triangulation of the mesh. The triangulation
      * is only performed on flat surfaces and is done in such a way to maximize the minimum
      * vertex angle of all the triangles. I.e. an iterative Delaunay-like triangulation is done.
      *
      * @param mesh                          The mesh that should re-triangulated.
      * @param dihedral_angle_tolerance      The maximum allowed dihedral angle (in degrees) for
      *                                      an edge-flip to occur. Default value is 5.
      */
      void operator()(mesh_type & mesh, double dihedral_angle_tolerance = 5.0)
      {
        using std::fabs;

        m_heap.clear();

        real_type dihedral_tolerance = dihedral_angle_tolerance*math::detail::pi<real_type>()/180.0;
        //std::cout << "PolyMeshTriangulate::operator(): dihedral tolerance = " << dihedral_tolerance << std::endl;

        for(typename mesh_type::edge_iterator edge = mesh.edge_begin(); edge!= mesh.edge_end(); ++edge)
        {
          if(is_boundary(*edge))
            continue;
          real_type radian;
          compute_dihedral_angle(*edge,radian);
          if(  fabs(radian) > dihedral_tolerance  )
            continue;

          edge_handle h = edge->get_handle();
          typename heap_type::heap_iterator i = m_heap.push( h );
          typename mesh_type::halfedge_iterator e = edge->get_halfedge0_iterator();
          vector3_type & v0 = e->get_destination_iterator()->m_coord;
          vector3_type & v1 = e->get_next_iterator()->get_destination_iterator()->m_coord;
          vector3_type & v2 = e->get_origin_iterator()->m_coord;
          vector3_type & v3 = e->get_twin_iterator()->get_next_iterator()->get_destination_iterator()->m_coord;
          i->priority() = compute_priority(v0,v1,v2,v3);
        }

        m_heap.heapify();

        real_type stop_tolerance = 0.01; //--- stop when improvement is less than 1%
        unsigned int cnt_flips = 0;
        bool forever = true;
        do
        {
          typename heap_type::heap_iterator H = m_heap.top();
          edge_handle h = H->get_feature();

          //std::cout << "priority = " << H->priority() << std::endl;

          if(H->priority() < stop_tolerance )
            break;

          m_heap.erase(h);

          if(!mesh.is_valid_edge_handle(h))
          {
            std::cerr << "PolyMeshTriangulate::operator(): Invalid edge in heap?" << std::endl;
            continue;
          }
          typename mesh_type::edge_iterator E     = mesh.get_edge_iterator(h);
          typename mesh_type::halfedge_iterator e = E->get_halfedge0_iterator();

          vertex_handle vh0 = e->get_destination_handle();
          vertex_handle vh1 = e->get_next_iterator()->get_destination_handle();
          vertex_handle vh2 = e->get_origin_handle();
          vertex_handle vh3 = e->get_twin_iterator()->get_next_iterator()->get_destination_handle();

          if(!edge_flip((*e)))
          {
            std::cerr << "PolyMeshTriangulate::operator(): Could not flipped edge" << std::endl;
            continue;
          }

          ++cnt_flips;

          edge_handle flipped = mesh.find_edge_handle(vh1,vh3);
          if(flipped.is_null())
          {
            std::cerr << "PolyMeshTriangulate::operator(): Could not find the flipped edge" << std::endl;
            break;
          }
          m_heap.push(flipped);

          update_priority(vh1,vh3,mesh);
          update_priority(vh0,vh1,mesh);
          update_priority(vh1,vh2,mesh);
          update_priority(vh2,vh3,mesh);
          update_priority(vh3,vh0,mesh);

        } while (forever);

        //std::cout << "PolyMeshTriangulate::operator(): |flips| = " << cnt_flips << std::endl;
      }

    };

    /**
    * PolyMesh Triangulation.
    * This is a convenience function making it easier to call the PolyMeshTriangulate functor.
    *
    * @param mesh                          The mesh that should be retriangulated.
    * @param dihedral_angle_tolerance      The maximum allowed dihedral angle (in degrees) for
    *                                      an edge-flip to occur. Default value is 5.
    */
    template< typename mesh_type >
    void triangulate( mesh_type & mesh, double dihedral_angle_tolerance = 5.0  )
    {
      PolyMeshTriangulate<mesh_type>()(mesh,dihedral_angle_tolerance);
    }

  } // namespace polymesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_TRIANGULATE_H
#endif
